#!/usr/bin/env python3
"""Robotar olive-harvest demo state machine.

States:
    WAIT_FOR_CONTROLLERS -> STOW -> DRIVE_TO_TREE -> STOP -> PRE_POSITION
    -> INSERT -> COMB -> DETACH -> RETRACT -> DONE

with an explicit FAILED outcome (reason logged, non-zero exit) on timeout,
controller/action failure, interlock violation or missing detach
acknowledgements.

Scenario inputs come from the SSOT files only: ``orchard.yaml`` (olive IDs,
detach/state topic patterns, branch + comb bank collision names) and
``physical.yaml`` (arm joint names, topic contract). Mission timing,
tolerances, arm poses and comb velocities live in ``config/harvest_demo.yaml``.
No Nav2, MoveIt, Gazebo pose queries or fruit ground truth are used for
targeting; detachment is contact-gated task logic.

Interlocks:
* the comb spins only while wheel odometry reports the base stationary;
* comb/branch contact gating is armed only during COMB;
* detach commands are published only after the configured bank/branch
  contact was observed (never in ``miss_contact`` verification mode);
* the comb is stopped before DETACH and stays stopped through RETRACT.
"""

import sys

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DRIVE_STATUS_TOPIC = "/mission/drive_status"
DRIVE_COMMAND_TOPIC = "/mission/drive_command"

ST_WAIT = "WAIT_FOR_CONTROLLERS"
ST_STOW = "STOW"
ST_DRIVE = "DRIVE_TO_TREE"
ST_STOP = "STOP"
ST_PRE = "PRE_POSITION"
ST_INSERT = "INSERT"
ST_COMB = "COMB"
ST_DETACH = "DETACH"
ST_RETRACT = "RETRACT"
ST_DONE = "DONE"
ST_FAILED = "FAILED"


class HarvestDemo(Node):
    def __init__(self):
        super().__init__("harvest_demo")

        self.declare_parameter("orchard_config_path", "")
        self.declare_parameter("physical_config_path", "")
        self.declare_parameter("miss_contact", False)
        # Poses: six joint values in physical.yaml arm.joints order.
        self.declare_parameter("stow_pose", [0.0] * 6)
        self.declare_parameter("pre_position_pose", [0.0] * 6)
        self.declare_parameter("insert_pose", [0.0] * 6)
        self.declare_parameter("miss_insert_pose", [0.0] * 6)
        self.declare_parameter("stow_duration_s", 6.0)
        self.declare_parameter("pre_position_duration_s", 6.0)
        self.declare_parameter("insert_duration_s", 4.0)
        self.declare_parameter("retract_duration_s", 6.0)
        self.declare_parameter("controller_wait_timeout_s", 90.0)
        self.declare_parameter("action_timeout_s", 40.0)
        self.declare_parameter("drive_status_timeout_s", 60.0)
        self.declare_parameter("stop_settle_s", 2.0)
        self.declare_parameter("stationary_linear_threshold", 0.02)
        self.declare_parameter("stationary_angular_threshold", 0.03)
        self.declare_parameter("comb_velocity_rad_s", 60.0)
        self.declare_parameter("comb_duration_s", 2.5)
        self.declare_parameter("comb_publish_rate_hz", 10.0)
        self.declare_parameter("detach_ack_timeout_s", 10.0)
        self.declare_parameter("mission_timeout_s", 300.0)

        orchard_path = self.get_parameter("orchard_config_path").value
        physical_path = self.get_parameter("physical_config_path").value
        if not orchard_path or not physical_path:
            raise RuntimeError(
                "harvest_demo requires orchard_config_path and physical_config_path"
            )
        with open(orchard_path, encoding="utf-8") as handle:
            orchard = yaml.safe_load(handle)["orchard"]
        with open(physical_path, encoding="utf-8") as handle:
            physical_doc = yaml.safe_load(handle)
        physical = physical_doc["robotar"]

        target = orchard["target_tree"]
        olives = target["olives"]
        self.olive_ids = [
            f"{olives['prefix']}_{i}" for i in range(target["olive_count"])
        ]
        self.detach_topics = {
            oid: orchard["topics"]["detach_pattern"].format(olive=oid)
            for oid in self.olive_ids
        }
        self.state_topics = {
            oid: orchard["topics"]["state_pattern"].format(olive=oid)
            for oid in self.olive_ids
        }
        self.branch_collision = orchard["contacts"]["branch_collision"]
        self.bank_collisions = list(orchard["contacts"]["tool_collisions"])

        self.arm_joints = [j["name"] for j in physical["arm"]["joints"]]
        controllers = physical_doc["control"]["controllers"]
        self.required_controllers = [
            controllers["joint_states"],
            controllers["drive"],
            controllers["arm"],
            controllers["comb"],
        ]
        topics = physical_doc["topics"]
        self.arm_action_name = topics["arm_action"]
        self.comb_command_topic = topics["comb_command"]
        self.comb_contact_topic = topics["comb_contact"]
        self.odom_topic = topics["odometry"]

        self.miss_contact = bool(self.get_parameter("miss_contact").value)
        self.poses = {
            "stow": list(self.get_parameter("stow_pose").value),
            "pre": list(self.get_parameter("pre_position_pose").value),
            "insert": list(self.get_parameter("insert_pose").value),
            "miss": list(self.get_parameter("miss_insert_pose").value),
        }
        for name, pose in self.poses.items():
            if len(pose) != len(self.arm_joints):
                raise RuntimeError(
                    f"pose {name!r} has {len(pose)} values, "
                    f"expected {len(self.arm_joints)}"
                )

        # ------------------------------------------------------------ comms
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, self.arm_action_name
        )
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.comb_pub = self.create_publisher(
            Float64MultiArray, self.comb_command_topic, 10
        )
        self.drive_cmd_pub = self.create_publisher(String, DRIVE_COMMAND_TOPIC, 1)
        self.odom = None            # (vx, vy, wz)
        self.odom_pose = None
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(
            String, DRIVE_STATUS_TOPIC, self._drive_status_cb,
            _latched_qos(),
        )
        self.create_subscription(
            Contacts, self.comb_contact_topic, self._contact_cb, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10
        )
        # State subscriptions exist before any detach can be published, so an
        # acknowledgement can never be missed.
        self.detached = set()
        self.state_seen = {}
        for oid in self.olive_ids:
            self.create_subscription(
                String, self.state_topics[oid],
                lambda msg, oid=oid: self._olive_state_cb(oid, msg), 10,
            )
        self.detach_pubs = {
            oid: self.create_publisher(Empty, self.detach_topics[oid], 10)
            for oid in self.olive_ids
        }

        # ------------------------------------------------------------- state
        self.state = ST_WAIT
        self.state_entered = None   # sim time of state entry
        self.drive_status = None
        self.joint_states_seen = False
        self.controllers_active = False
        self.controller_query = None
        self.traj_active = False
        self.traj_done = False
        self.traj_success = False
        self.comb_on = False
        self.comb_started = None
        self.branch_contact_seen = False
        self.stationary_since = None
        self.detach_sent_time = None
        self.fail_reason = ""
        self.exit_code = None
        self.mission_started = None

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"harvest_demo ready: {len(self.olive_ids)} target olives "
            f"{self.olive_ids}, miss_contact={self.miss_contact}"
        )

    # ------------------------------------------------------------ callbacks

    def _odom_cb(self, msg):
        self.odom = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _drive_status_cb(self, msg):
        self.drive_status = msg.data

    def _joint_states_cb(self, msg):
        self.joint_states_seen = True

    def _olive_state_cb(self, oid, msg):
        state = msg.data.strip().lower()
        self.state_seen[oid] = state
        if state == "detached":
            if oid not in self.detached:
                self.get_logger().info(f"olive {oid} reports DETACHED")
            self.detached.add(oid)

    def _contact_cb(self, msg):
        if self.state != ST_COMB:
            return  # gating armed only during COMB
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            bank_hit = any(
                any(bank in n for n in names) for bank in self.bank_collisions
            )
            branch_hit = any(self.branch_collision in n for n in names)
            if bank_hit and branch_hit and not self.branch_contact_seen:
                self.branch_contact_seen = True
                self.get_logger().info(
                    f"comb/branch contact confirmed: {names[0]} <-> {names[1]}"
                )

    # ------------------------------------------------------------ arm goals

    def _send_trajectory(self, waypoints):
        """waypoints: list of (pose, duration_from_previous_s)."""
        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joints)
        t = 0.0
        for pose, duration in waypoints:
            t += duration
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in pose]
            point.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_active = True
        self.traj_done = False
        self.traj_success = False
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("arm trajectory goal rejected")
            self.traj_done = True
            self.traj_success = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result().result
        self.traj_done = True
        self.traj_success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not self.traj_success:
            self.get_logger().error(
                f"arm trajectory failed with error_code {result.error_code}"
            )
        self.traj_active = False

    # ------------------------------------------------------------ comb / drive

    def _set_comb(self, on):
        velocity = float(self.get_parameter("comb_velocity_rad_s").value)
        msg = Float64MultiArray()
        # Counter-rotating banks: opposite signs per the comb design.
        msg.data = [velocity, -velocity] if on else [0.0, 0.0]
        self.comb_pub.publish(msg)
        self.comb_on = on

    def _is_stationary(self):
        if self.odom is None:
            return False
        vx, vy, wz = self.odom
        lin_thr = float(self.get_parameter("stationary_linear_threshold").value)
        ang_thr = float(self.get_parameter("stationary_angular_threshold").value)
        return abs(vx) <= lin_thr and abs(vy) <= lin_thr and abs(wz) <= ang_thr

    # ------------------------------------------------------------ state utils

    def _enter(self, state):
        self.state = state
        self.state_entered = self.get_clock().now()
        self.get_logger().info(f"[state] {state}")

    def _state_age(self):
        now = self.get_clock().now()
        if self.state_entered is None or now.nanoseconds == 0:
            return 0.0
        return (now - self.state_entered).nanoseconds * 1e-9

    def _fail(self, reason):
        if self.comb_on:
            self._set_comb(False)
        self.fail_reason = reason
        self.get_logger().error(f"MISSION FAILED: {reason}")
        self._enter(ST_FAILED)
        self.exit_code = 1

    def _traj_state_tick(self, next_state, timeout):
        """Common handling for states that wait on an arm trajectory."""
        if self.traj_done:
            if self.traj_success:
                self._enter(next_state)
            else:
                self._fail(f"arm trajectory failed in {self.state}")
            return
        if self._state_age() > timeout:
            self._fail(f"arm trajectory timeout in {self.state}")

    # ------------------------------------------------------------------ tick

    def _tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # /clock not flowing yet
        if self.mission_started is None:
            self.mission_started = now
        if self.state in (ST_DONE, ST_FAILED):
            return
        if (now - self.mission_started).nanoseconds * 1e-9 > float(
            self.get_parameter("mission_timeout_s").value
        ):
            self._fail("overall mission timeout")
            return

        handler = {
            ST_WAIT: self._tick_wait,
            ST_STOW: self._tick_stow,
            ST_DRIVE: self._tick_drive,
            ST_STOP: self._tick_stop,
            ST_PRE: self._tick_pre,
            ST_INSERT: self._tick_insert,
            ST_COMB: self._tick_comb,
            ST_DETACH: self._tick_detach,
            ST_RETRACT: self._tick_retract,
        }.get(self.state)
        if handler:
            handler()

        if self.comb_on:
            # Re-publish spin command (keeps the command fresh) and enforce
            # the stationary-base interlock every cycle.
            if not self._is_stationary():
                self._fail("base moved while comb was spinning (interlock)")
                return
            self._set_comb(True)

    def _controllers_are_active(self):
        if self.controllers_active:
            return True
        if not self.controller_client.service_is_ready():
            return False
        if self.controller_query is None:
            self.controller_query = self.controller_client.call_async(
                ListControllers.Request()
            )
            return False
        if not self.controller_query.done():
            return False

        try:
            result = self.controller_query.result()
        except Exception as error:
            self.get_logger().warn(f"controller state query failed: {error}")
            self.controller_query = None
            return False

        states = {
            controller.name: controller.state for controller in result.controller
        }
        inactive = [
            name for name in self.required_controllers
            if states.get(name) != "active"
        ]
        if inactive:
            self.controller_query = None
            return False

        self.controllers_active = True
        self.get_logger().info(
            f"required controllers active: {self.required_controllers}"
        )
        return True

    def _tick_wait(self):
        timeout = float(self.get_parameter("controller_wait_timeout_s").value)
        ready = (
            self.arm_client.server_is_ready()
            and self.odom is not None
            and self.drive_status is not None
            and self.joint_states_seen
            and self._controllers_are_active()
        )
        if ready:
            self.get_logger().info("controllers and odometry available")
            self._send_trajectory(
                [(self.poses["stow"],
                  float(self.get_parameter("stow_duration_s").value))]
            )
            self._enter(ST_STOW)
        elif self._state_age() > timeout:
            self._fail("controllers/odometry not available before timeout")

    def _tick_stow(self):
        if self.traj_done and self.traj_success:
            cmd = String()
            cmd.data = "start"
            self.drive_cmd_pub.publish(cmd)
            self._enter(ST_DRIVE)
        else:
            self._traj_state_tick(ST_DRIVE, float(
                self.get_parameter("action_timeout_s").value))

    def _tick_drive(self):
        if self.drive_status is None:
            return
        if self.drive_status.startswith("done"):
            self.get_logger().info(f"drive completed: {self.drive_status}")
            self.stationary_since = None
            self._enter(ST_STOP)
        elif self.drive_status.startswith("failed"):
            self._fail(f"odom drive failed: {self.drive_status}")
        elif self._state_age() > float(
            self.get_parameter("drive_status_timeout_s").value
        ):
            self._fail("drive status timeout")

    def _tick_stop(self):
        if self._is_stationary():
            if self.stationary_since is None:
                self.stationary_since = self.get_clock().now()
            settle = float(self.get_parameter("stop_settle_s").value)
            if (self.get_clock().now()
                    - self.stationary_since).nanoseconds * 1e-9 >= settle:
                self._send_trajectory(
                    [(self.poses["pre"],
                      float(self.get_parameter("pre_position_duration_s").value))]
                )
                self._enter(ST_PRE)
        else:
            self.stationary_since = None
            if self._state_age() > float(
                self.get_parameter("action_timeout_s").value
            ):
                self._fail("base did not come to rest after the drive")

    def _tick_pre(self):
        self._traj_state_tick(
            ST_INSERT, float(self.get_parameter("action_timeout_s").value))
        if self.state == ST_INSERT and not self.traj_active:
            pose = self.poses["miss"] if self.miss_contact else self.poses["insert"]
            self._send_trajectory(
                [(pose, float(self.get_parameter("insert_duration_s").value))]
            )

    def _tick_insert(self):
        self._traj_state_tick(
            ST_COMB, float(self.get_parameter("action_timeout_s").value))
        if self.state == ST_COMB and not self.traj_active:
            if not self._is_stationary():
                self._fail("base not stationary at comb start (interlock)")
                return
            self.comb_started = self.get_clock().now()
            self._set_comb(True)
            self.get_logger().info("comb spinning (counter-rotating banks)")

    def _tick_comb(self):
        duration = float(self.get_parameter("comb_duration_s").value)
        if (self.get_clock().now() - self.comb_started).nanoseconds * 1e-9 >= duration:
            self._set_comb(False)
            self.get_logger().info(
                f"comb stopped after {duration:.1f} s; "
                f"branch_contact_seen={self.branch_contact_seen}"
            )
            self._enter(ST_DETACH)

    def _tick_detach(self):
        if self.detach_sent_time is None:
            if self.miss_contact:
                if self.branch_contact_seen:
                    self._fail(
                        "unexpected comb/branch contact in miss-contact mode; "
                        "no detach commands published"
                    )
                else:
                    self._fail(
                        "miss-contact verification: no branch contact, "
                        "no detach commands published (clean negative result)"
                    )
                return
            if not self.branch_contact_seen:
                self._fail(
                    "no comb/branch contact during COMB; detach gated off, "
                    "olives left attached"
                )
                return
            for oid in self.olive_ids:
                self.detach_pubs[oid].publish(Empty())
            self.detach_sent_time = self.get_clock().now()
            self.get_logger().info(
                f"detach commands published for {self.olive_ids}"
            )
            return

        missing = [oid for oid in self.olive_ids if oid not in self.detached]
        if not missing:
            self._send_trajectory(
                [
                    (self.poses["pre"],
                     float(self.get_parameter("insert_duration_s").value)),
                    (self.poses["stow"],
                     float(self.get_parameter("retract_duration_s").value)),
                ]
            )
            self._enter(ST_RETRACT)
        elif (self.get_clock().now()
              - self.detach_sent_time).nanoseconds * 1e-9 > float(
                  self.get_parameter("detach_ack_timeout_s").value):
            self._fail(
                f"detach acknowledgement timeout; missing: {missing}, "
                f"detached: {sorted(self.detached)}"
            )

    def _tick_retract(self):
        if self.traj_done and self.traj_success:
            self._enter(ST_DONE)
            self.get_logger().info(
                f"MISSION DONE: detached={sorted(self.detached)} "
                f"({len(self.detached)}/{len(self.olive_ids)}), "
                f"final odom pose={self.odom_pose}"
            )
            self.exit_code = 0
        else:
            self._traj_state_tick(ST_DONE, float(
                self.get_parameter("action_timeout_s").value) * 2.0)


def _latched_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def main(argv=None):
    rclpy.init(args=argv)
    node = HarvestDemo()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        if node.comb_on:
            node._set_comb(False)
        node.destroy_node()
        rclpy.shutdown()
    return node.exit_code if node.exit_code is not None else 0


if __name__ == "__main__":
    sys.exit(main())
