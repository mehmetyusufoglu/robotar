#!/usr/bin/env python3
"""Robotar branch clamp-shaker demo state machine.

States:
    WAIT_FOR_CONTROLLERS -> STOW -> DRIVE_TO_TREE -> STOP -> PRE_POSITION
    -> APPROACH -> CLAMP -> VERIFY_CLAMP -> FREQUENCY_SWEEP
    -> CONTROLLED_SHAKE -> DETACH -> RELEASE -> RETRACT -> DONE

with an explicit FAILED outcome (reason logged, non-zero exit) on timeout,
controller/action failure, interlock violation or missing detach
acknowledgements, and a fail-safe jaw release when clamp verification fails
(including the deliberate ``miss_contact`` negative verification).

Scenario inputs come from the SSOT files only: ``orchard.yaml`` (olive IDs,
detach/state topic patterns, branch + jaw pad collision names, target branch
radius) and ``physical.yaml`` (arm joint names, controller names, topic
contract, shaker clamp geometry and the 10-25 Hz / 3-5 s operational
envelope). Mission timing, tolerances and arm poses live in
``config/shaker_demo.yaml``. No Nav2, MoveIt, Gazebo pose queries or fruit
ground truth are used for targeting; detachment is gated task logic.

Interlocks:
* the rotors spin only while wheel odometry reports the base stationary;
* clamp contact gating is armed from CLAMP through the shake states;
* detach commands are published only after the clamp was verified (pad /
  branch contact) AND the controlled shake ran for the configured duration
  (never in ``miss_contact`` verification mode);
* the rotors are stopped before DETACH and stay stopped through RETRACT;
* a failed clamp verification opens the jaws before the mission aborts.
"""

import math
import sys

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DRIVE_STATUS_TOPIC = "/mission/drive_status"
DRIVE_COMMAND_TOPIC = "/mission/drive_command"

ST_WAIT = "WAIT_FOR_CONTROLLERS"
ST_STOW = "STOW"
ST_DRIVE = "DRIVE_TO_TREE"
ST_STOP = "STOP"
ST_PRE = "PRE_POSITION"
ST_APPROACH = "APPROACH"
ST_CLAMP = "CLAMP"
ST_VERIFY = "VERIFY_CLAMP"
ST_SWEEP = "FREQUENCY_SWEEP"
ST_SHAKE = "CONTROLLED_SHAKE"
ST_DETACH = "DETACH"
ST_RELEASE = "RELEASE"
ST_RETRACT = "RETRACT"
ST_DONE = "DONE"
ST_FAILED = "FAILED"


class ShakerDemo(Node):
    def __init__(self):
        super().__init__("shaker_demo")

        self.declare_parameter("orchard_config_path", "")
        self.declare_parameter("physical_config_path", "")
        self.declare_parameter("miss_contact", False)
        # Poses: six joint values in physical.yaml arm.joints order.
        self.declare_parameter("stow_pose", [0.0] * 6)
        self.declare_parameter("pre_position_pose", [0.0] * 6)
        self.declare_parameter("approach_pose", [0.0] * 6)
        self.declare_parameter("clamp_pose", [0.0] * 6)
        self.declare_parameter("miss_clamp_pose", [0.0] * 6)
        self.declare_parameter("stow_duration_s", 6.0)
        self.declare_parameter("pre_position_duration_s", 6.0)
        self.declare_parameter("approach_duration_s", 5.0)
        self.declare_parameter("clamp_slide_duration_s", 4.0)
        self.declare_parameter("retract_duration_s", 6.0)
        self.declare_parameter("controller_wait_timeout_s", 90.0)
        self.declare_parameter("action_timeout_s", 40.0)
        self.declare_parameter("drive_status_timeout_s", 60.0)
        self.declare_parameter("stop_settle_s", 2.0)
        self.declare_parameter("stationary_linear_threshold", 0.02)
        self.declare_parameter("stationary_angular_threshold", 0.03)
        self.declare_parameter("clamp_interference_m", 0.004)
        self.declare_parameter("goal_tolerance_rad", 0.05)
        self.declare_parameter("clamp_action_timeout_s", 25.0)
        self.declare_parameter("clamp_verify_timeout_s", 8.0)
        self.declare_parameter("sweep_step_hz", 3.0)
        self.declare_parameter("sweep_step_duration_s", 0.9)
        self.declare_parameter("detach_ack_timeout_s", 10.0)
        self.declare_parameter("mission_timeout_s", 420.0)

        orchard_path = self.get_parameter("orchard_config_path").value
        physical_path = self.get_parameter("physical_config_path").value
        if not orchard_path or not physical_path:
            raise RuntimeError(
                "shaker_demo requires orchard_config_path and physical_config_path"
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
        self.pad_collisions = list(orchard["contacts"]["shaker_tool_collisions"])
        self.branch_radius = float(target["harvest_branch"]["radius"])

        shaker = physical["tools"]["shaker"]
        self.arm_joints = [j["name"] for j in physical["arm"]["joints"]]
        controllers = physical_doc["control"]["controllers"]
        self.required_controllers = [
            controllers["joint_states"],
            controllers["drive"],
            controllers["arm"],
            controllers["shaker_clamp"],
            controllers["shaker_rotors"],
        ]
        topics = physical_doc["topics"]
        self.arm_action_name = topics["arm_action"]
        self.clamp_action_name = topics["shaker_clamp_action"]
        self.rotor_command_topic = topics["shaker_rotor_command"]
        self.imu_topic = topics["shaker_imu"]
        self.clamp_contact_topic = topics["shaker_clamp_contact"]
        self.odom_topic = topics["odometry"]

        # Clamp goal from the SSOT: pad faces touch the branch at
        # position = branch_radius + pad_face_offset; a small interference
        # makes the clamp stall against the branch (a successful clamp).
        pad_face_offset = float(shaker["clamp"]["pad_face_offset"])
        # Open jaws to just inside the hard limit (never command a joint
        # exactly onto its limit — DART limit constraints can latch).
        self.jaw_open_position = (
            float(shaker["clamp"]["max_opening"]) / 2.0
            - float(shaker["clamp"]["open_margin"])
        )
        self.clamp_goal_position = (
            self.branch_radius + pad_face_offset
            - float(self.get_parameter("clamp_interference_m").value)
        )
        self.clamp_max_effort = float(shaker["clamp"]["max_effort"])

        rotors = shaker["rotors"]
        self.sweep_min_hz = float(rotors["min_frequency_hz"])
        self.sweep_max_hz = float(rotors["max_frequency_hz"])
        self.shake_duration_s = float(rotors["shake_seconds"])

        self.miss_contact = bool(self.get_parameter("miss_contact").value)
        self.poses = {
            "stow": list(self.get_parameter("stow_pose").value),
            "pre": list(self.get_parameter("pre_position_pose").value),
            "approach": list(self.get_parameter("approach_pose").value),
            "clamp": list(self.get_parameter("clamp_pose").value),
            "miss": list(self.get_parameter("miss_clamp_pose").value),
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
        self.clamp_client = ActionClient(
            self, GripperCommand, self.clamp_action_name
        )
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.rotor_pub = self.create_publisher(
            Float64MultiArray, self.rotor_command_topic, 10
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
            Contacts, self.clamp_contact_topic, self._contact_cb, 10
        )
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
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
        self.approach_stage = 0     # 0 jaw-open, 1 to approach, 2 slide
        self.retract_queue = []     # staged (pose, duration_s) retract goals
        self.drive_status = None
        self.joint_states_seen = False
        self.controllers_active = False
        self.controller_query = None
        self.traj_active = False
        self.traj_done = False
        self.traj_success = False
        self.clamp_goal_active = False
        self.clamp_goal_done = False
        self.clamp_goal_result = None     # GripperCommand.Result
        self.clamp_close_sent = False
        self.rotors_on = False
        self.rotor_frequency_hz = 0.0
        self.clamp_contact_seen = False
        self.clamp_verified = False
        self.imu_samples = []             # accel magnitudes, current step
        self.sweep_results = []           # (frequency_hz, response)
        self.sweep_frequency_hz = None
        self.sweep_step_started = None
        self.shake_started = None
        self.shake_completed = False
        self.stationary_since = None
        self.detach_sent_time = None
        self.abort_reason = None          # set => RELEASE leads to FAILED
        self.fail_reason = ""
        self.exit_code = None
        self.mission_started = None

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"shaker_demo ready: {len(self.olive_ids)} target olives "
            f"{self.olive_ids}, clamp goal {self.clamp_goal_position:.4f} m, "
            f"sweep {self.sweep_min_hz:.0f}-{self.sweep_max_hz:.0f} Hz, "
            f"shake {self.shake_duration_s:.1f} s, "
            f"miss_contact={self.miss_contact}"
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
        if self.state not in (ST_CLAMP, ST_VERIFY, ST_SWEEP, ST_SHAKE):
            return  # clamp gating armed only around the clamp/shake states
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            pad_hit = any(
                any(pad in n for n in names) for pad in self.pad_collisions
            )
            branch_hit = any(self.branch_collision in n for n in names)
            if pad_hit and branch_hit and not self.clamp_contact_seen:
                self.clamp_contact_seen = True
                self.get_logger().info(
                    f"pad/branch clamp contact confirmed: "
                    f"{names[0]} <-> {names[1]}"
                )

    def _imu_cb(self, msg):
        if self.state not in (ST_SWEEP, ST_SHAKE):
            return
        accel = msg.linear_acceleration
        self.imu_samples.append(
            math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2)
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
        # Position tolerance per joint: if the arm wedges against an
        # obstacle the goal ABORTS instead of silently diverging (the
        # controller's default empty tolerance would report success at the
        # trajectory end time regardless of tracking error).
        for name in self.arm_joints:
            tol = JointTolerance()
            tol.name = name
            tol.position = float(self.get_parameter("goal_tolerance_rad").value)
            goal.goal_tolerance.append(tol)
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()
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

    # ---------------------------------------------------------- clamp goals

    def _send_clamp_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.clamp_max_effort
        self.clamp_goal_active = True
        self.clamp_goal_done = False
        self.clamp_goal_result = None
        future = self.clamp_client.send_goal_async(goal)
        future.add_done_callback(self._clamp_response_cb)

    def _clamp_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("clamp goal rejected")
            self.clamp_goal_done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._clamp_result_cb)

    def _clamp_result_cb(self, future):
        self.clamp_goal_result = future.result().result
        self.clamp_goal_done = True
        self.clamp_goal_active = False

    # --------------------------------------------------------- rotors/drive

    def _set_rotors(self, frequency_hz):
        """Command counter-rotating rotors; forcing f = rotor_speed / 2pi."""
        omega = 2.0 * math.pi * frequency_hz
        msg = Float64MultiArray()
        msg.data = [omega, -omega] if frequency_hz > 0.0 else [0.0, 0.0]
        self.rotor_pub.publish(msg)
        self.rotors_on = frequency_hz > 0.0
        self.rotor_frequency_hz = frequency_hz

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
        if self.rotors_on:
            self._set_rotors(0.0)
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

    def _clamp_goal_tick(self, next_state, timeout):
        """Common handling for states that wait on a clamp action result."""
        if self.clamp_goal_done:
            if self.clamp_goal_result is not None:
                self._enter(next_state)
            else:
                self._fail(f"clamp action rejected in {self.state}")
            return
        if self._state_age() > timeout:
            self._fail(f"clamp action timeout in {self.state}")

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
            ST_APPROACH: self._tick_approach,
            ST_CLAMP: self._tick_clamp,
            ST_VERIFY: self._tick_verify,
            ST_SWEEP: self._tick_sweep,
            ST_SHAKE: self._tick_shake,
            ST_DETACH: self._tick_detach,
            ST_RELEASE: self._tick_release,
            ST_RETRACT: self._tick_retract,
        }.get(self.state)
        if handler:
            handler()

        if self.rotors_on:
            # Re-publish the rotor command (keeps it fresh) and enforce the
            # stationary-base interlock every cycle.
            if not self._is_stationary():
                self._fail("base moved while rotors were spinning (interlock)")
                return
            self._set_rotors(self.rotor_frequency_hz)

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

    # --------------------------------------------------------------- states

    def _tick_wait(self):
        timeout = float(self.get_parameter("controller_wait_timeout_s").value)
        ready = (
            self.arm_client.server_is_ready()
            and self.clamp_client.server_is_ready()
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
            ST_APPROACH, float(self.get_parameter("action_timeout_s").value))
        if self.state == ST_APPROACH and not self.traj_active:
            if not self._is_stationary():
                self._fail("base not stationary at approach start (interlock)")
                return
            # Staged approach: open jaws first, then move to the approach
            # pose, then slide onto the branch — each as its OWN single-point
            # trajectory. Multi-segment splines through these poses were
            # observed to fling wrist_pitch onto its joint limit at interior
            # waypoints, where DART's limit constraint latches the joint.
            self.approach_stage = 0
            self._send_clamp_goal(self.jaw_open_position)

    def _tick_approach(self):
        timeout = float(self.get_parameter("action_timeout_s").value) * 2.0
        if self.clamp_goal_done and self.clamp_goal_result is None:
            self._fail("jaw-open goal rejected in APPROACH")
            return
        if self.traj_done and not self.traj_success:
            self._fail("arm trajectory failed in APPROACH")
            return
        if self._state_age() > timeout:
            self._fail("approach stage timeout in APPROACH")
            return
        if self.approach_stage == 0:
            if self.clamp_goal_done:
                self.approach_stage = 1
                self._send_trajectory(
                    [(self.poses["approach"],
                      float(self.get_parameter("approach_duration_s").value))]
                )
        elif self.approach_stage == 1:
            if self.traj_done:  # success checked above
                self.approach_stage = 2
                slide_pose = (
                    self.poses["miss"] if self.miss_contact else self.poses["clamp"]
                )
                self._send_trajectory(
                    [(slide_pose,
                      float(self.get_parameter("clamp_slide_duration_s").value))]
                )
        elif self.approach_stage == 2:
            if self.traj_done:
                self._enter(ST_CLAMP)

    def _tick_clamp(self):
        if not self.clamp_close_sent:
            if not self._is_stationary():
                self._fail("base not stationary at clamp start (interlock)")
                return
            self.get_logger().info(
                f"clamping: goal {self.clamp_goal_position:.4f} m, "
                f"max_effort {self.clamp_max_effort:.1f} N"
            )
            self.clamp_close_sent = True
            self._send_clamp_goal(self.clamp_goal_position)
            return
        self._clamp_goal_tick(
            ST_VERIFY, float(self.get_parameter("clamp_action_timeout_s").value))

    def _tick_verify(self):
        timeout = float(self.get_parameter("clamp_verify_timeout_s").value)
        result = self.clamp_goal_result
        if self.clamp_contact_seen:
            self.clamp_verified = True
            self.get_logger().info(
                f"clamp verified (stalled={result.stalled}, "
                f"reached_goal={result.reached_goal}, "
                f"position={result.position:.4f})"
            )
            self._enter(ST_SWEEP)
            return
        if self._state_age() > timeout:
            if self.miss_contact:
                self.abort_reason = (
                    "miss-contact verification: no pad/branch contact, no "
                    "shake, no detach commands published (clean negative result)"
                )
            else:
                self.abort_reason = (
                    "clamp verification failed: no pad/branch contact within "
                    f"{timeout:.1f} s; detach gated off, olives left attached"
                )
            self.get_logger().warn(f"{self.abort_reason}; releasing jaws")
            self._send_clamp_goal(self.jaw_open_position)
            self._enter(ST_RELEASE)

    def _tick_sweep(self):
        step_hz = float(self.get_parameter("sweep_step_hz").value)
        step_s = float(self.get_parameter("sweep_step_duration_s").value)
        if self.sweep_frequency_hz is None:
            self.sweep_frequency_hz = self.sweep_min_hz
            self.sweep_step_started = self.get_clock().now()
            self.imu_samples = []
            self._set_rotors(self.sweep_frequency_hz)
            self.get_logger().info(
                f"sweep step: {self.sweep_frequency_hz:.1f} Hz "
                f"(rotor {2.0 * math.pi * self.sweep_frequency_hz:.1f} rad/s)"
            )
            return
        age = (self.get_clock().now() - self.sweep_step_started).nanoseconds * 1e-9
        if age < step_s:
            return
        response = _rms_deviation(self.imu_samples)
        omega = 2.0 * math.pi * self.sweep_frequency_hz
        # Housing-IMU acceleration from an eccentric rotor grows ~omega^2;
        # normalizing gives a vibration amplitude estimate (m), the
        # harvest-relevant quantity (papers: up to ~0.05 m).
        amplitude = response / (omega * omega)
        self.sweep_results.append((self.sweep_frequency_hz, amplitude))
        self.get_logger().info(
            f"sweep {self.sweep_frequency_hz:.1f} Hz: "
            f"IMU response {response:.3f} m/s^2, amplitude estimate "
            f"{amplitude * 1000.0:.2f} mm ({len(self.imu_samples)} samples)"
        )
        next_frequency = self.sweep_frequency_hz + step_hz
        if next_frequency <= self.sweep_max_hz + 1e-9:
            self.sweep_frequency_hz = next_frequency
            self.sweep_step_started = self.get_clock().now()
            self.imu_samples = []
            self._set_rotors(next_frequency)
            self.get_logger().info(f"sweep step: {next_frequency:.1f} Hz")
            return
        if not self.sweep_results or all(
            response == 0.0 for _, response in self.sweep_results
        ):
            self._set_rotors(0.0)
            self._fail("no shaker IMU response recorded during the sweep")
            return
        best_frequency, best_amplitude = max(
            self.sweep_results, key=lambda item: item[1]
        )
        self.get_logger().info(
            f"sweep complete: peak amplitude estimate "
            f"{best_amplitude * 1000.0:.2f} mm at {best_frequency:.1f} Hz; "
            f"holding for {self.shake_duration_s:.1f} s"
        )
        self._set_rotors(best_frequency)
        self.shake_started = self.get_clock().now()
        self.imu_samples = []
        self._enter(ST_SHAKE)

    def _tick_shake(self):
        age = (self.get_clock().now() - self.shake_started).nanoseconds * 1e-9
        if age < self.shake_duration_s:
            return
        response = _rms_deviation(self.imu_samples)
        frequency = self.rotor_frequency_hz
        omega = 2.0 * math.pi * frequency
        amplitude = response / (omega * omega)
        self._set_rotors(0.0)
        self.shake_completed = True
        self.get_logger().info(
            f"controlled shake done at {frequency:.1f} Hz: "
            f"IMU response {response:.3f} m/s^2, amplitude estimate "
            f"{amplitude * 1000.0:.2f} mm"
        )
        self._enter(ST_DETACH)

    def _tick_detach(self):
        if self.detach_sent_time is None:
            if not (self.clamp_verified and self.shake_completed):
                self._fail(
                    "detach gate opened without clamp verification + "
                    "completed shake; no detach commands published"
                )
                return
            for oid in self.olive_ids:
                self.detach_pubs[oid].publish(Empty())
            self.detach_sent_time = self.get_clock().now()
            self.get_logger().info(
                f"detach commands published for {self.olive_ids} "
                "(clamp verified, shake completed)"
            )
            return

        missing = [oid for oid in self.olive_ids if oid not in self.detached]
        if not missing:
            self._send_clamp_goal(self.jaw_open_position)
            self._enter(ST_RELEASE)
        elif (self.get_clock().now()
              - self.detach_sent_time).nanoseconds * 1e-9 > float(
                  self.get_parameter("detach_ack_timeout_s").value):
            self._fail(
                f"detach acknowledgement timeout; missing: {missing}, "
                f"detached: {sorted(self.detached)}"
            )

    def _tick_release(self):
        if self.clamp_goal_done:
            if self.clamp_goal_result is None:
                self._fail("jaw release goal rejected")
                return
            self.get_logger().info("jaws released")
            if self.abort_reason is not None:
                self._fail(self.abort_reason)
                return
            # Staged single-point retract goals (same spline-overshoot
            # rationale as the approach).
            slide_back = float(self.get_parameter("clamp_slide_duration_s").value)
            self.retract_queue = [
                (self.poses["approach"], slide_back),
                (self.poses["pre"], slide_back),
                (self.poses["stow"],
                 float(self.get_parameter("retract_duration_s").value)),
            ]
            pose, duration = self.retract_queue.pop(0)
            self._send_trajectory([(pose, duration)])
            self._enter(ST_RETRACT)
            return
        if self._state_age() > float(
            self.get_parameter("clamp_action_timeout_s").value
        ):
            self._fail("jaw release action timeout")

    def _tick_retract(self):
        if self.traj_done:
            if not self.traj_success:
                self._fail("arm trajectory failed in RETRACT")
                return
            if self.retract_queue:
                pose, duration = self.retract_queue.pop(0)
                self._send_trajectory([(pose, duration)])
                return
            self._enter(ST_DONE)
            self.get_logger().info(
                f"MISSION DONE: detached={sorted(self.detached)} "
                f"({len(self.detached)}/{len(self.olive_ids)}), "
                f"final odom pose={self.odom_pose}"
            )
            self.exit_code = 0
            return
        if self._state_age() > float(
            self.get_parameter("action_timeout_s").value
        ) * 3.0:
            self._fail("arm trajectory timeout in RETRACT")


def _rms_deviation(samples):
    """RMS of the sample deviations from the mean (vibration response)."""
    if len(samples) < 2:
        return 0.0
    mean = sum(samples) / len(samples)
    return math.sqrt(sum((s - mean) ** 2 for s in samples) / len(samples))


def _latched_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def main(argv=None):
    rclpy.init(args=argv)
    node = ShakerDemo()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        if node.rotors_on:
            node._set_rotors(0.0)
        node.destroy_node()
        rclpy.shutdown()
    return node.exit_code if node.exit_code is not None else 0


if __name__ == "__main__":
    sys.exit(main())
