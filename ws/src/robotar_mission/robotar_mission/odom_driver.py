#!/usr/bin/env python3
"""Closed-loop odometry driver for the short orchard-row drive.

Reads the drive target from the orchard SSOT (``orchard.yaml``:
``robot.drive_to_x``, maintaining the configured spawn y) and the speed
limits from the robot SSOT (``physical.yaml``: ``base.limits``). Publishes
Jazzy ``geometry_msgs/TwistStamped`` commands on
``/drive_controller/cmd_vel`` and closes the loop on
``/drive_controller/odom``.

The mission state machine drives this node through two std_msgs topics:

* ``/mission/drive_command`` (String): ``start`` begins the drive,
  ``cancel`` aborts it and stops the rover;
* ``/mission/drive_status`` (String, latched): ``idle``, ``driving``,
  ``done x=<measured>`` or ``failed <reason>``.

The diff-drive odometry frame originates at the spawn pose, so the odometry
target is ``drive_to_x - spawn_x`` and ``0`` for y. Conservative approach:
command speed is capped at ``harvest_linear_velocity`` and scaled down near
the target; any odometry gap longer than ``odom_timeout_s`` or total drive
longer than ``drive_timeout_s`` stops the rover and reports failure. When
commands stop, the controller's own ``cmd_vel_timeout`` halts the wheels.
"""

import math
import sys

import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STATUS_TOPIC = "/mission/drive_status"
COMMAND_TOPIC = "/mission/drive_command"


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class OdomDriver(Node):
    def __init__(self):
        super().__init__("odom_driver")

        self.declare_parameter("orchard_config_path", "")
        self.declare_parameter("physical_config_path", "")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("position_tolerance_m", 0.05)
        self.declare_parameter("drive_timeout_s", 45.0)
        self.declare_parameter("odom_timeout_s", 3.0)
        self.declare_parameter("kp_linear", 0.6)
        self.declare_parameter("kp_lateral", 1.0)
        self.declare_parameter("kp_yaw", 1.2)

        orchard_path = self.get_parameter("orchard_config_path").value
        physical_path = self.get_parameter("physical_config_path").value
        if not orchard_path:
            raise RuntimeError("odom_driver requires orchard_config_path")
        if not physical_path:
            raise RuntimeError("odom_driver requires physical_config_path")

        with open(orchard_path, encoding="utf-8") as handle:
            orchard = yaml.safe_load(handle)["orchard"]
        with open(physical_path, encoding="utf-8") as handle:
            physical_doc = yaml.safe_load(handle)
        physical = physical_doc["robotar"]

        robot = orchard["robot"]
        spawn_xyz = robot["spawn_xyz"]
        # Odometry originates at the spawn pose; convert the world-frame
        # target into the odometry frame.
        self.target_x = float(robot["drive_to_x"]) - float(spawn_xyz[0])
        self.target_y = 0.0  # maintain the configured spawn y
        self.spawn_yaw = float(robot.get("spawn_yaw", 0.0))
        if abs(self.spawn_yaw) > 1e-6:
            raise RuntimeError("odom_driver supports only spawn_yaw == 0")

        limits = physical["base"]["limits"]
        self.max_linear = float(limits["harvest_linear_velocity"])
        self.max_angular = float(limits["max_angular_velocity"])

        topics = physical_doc["topics"]
        self.cmd_topic = topics["cmd_vel"]
        self.odom_topic = topics["odometry"]

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.tolerance = float(self.get_parameter("position_tolerance_m").value)
        self.drive_timeout = float(self.get_parameter("drive_timeout_s").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_lateral = float(self.get_parameter("kp_lateral").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)

        self.state = "idle"
        self.pose = None          # (x, y, yaw) latest odometry
        self.last_odom_time = None
        self.drive_start_time = None
        self.zero_pulses_left = 0

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, latched)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._command_cb, 1)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self._publish_status("idle")
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"odom_driver ready: target odom x={self.target_x:.3f} m, "
            f"holding y={self.target_y:.3f}, vmax={self.max_linear:.2f} m/s"
        )

    # ------------------------------------------------------------------ I/O

    def _publish_status(self, status):
        self.state = status.split(" ", 1)[0]
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (x, y, yaw)
        self.last_odom_time = self.get_clock().now()

    def _command_cb(self, msg):
        command = msg.data.strip().lower()
        if command == "start":
            if self.pose is None:
                self._publish_status("failed no odometry received yet")
                return
            if self.state != "driving":
                self.drive_start_time = self.get_clock().now()
                self._publish_status("driving")
                self.get_logger().info("drive started")
        elif command == "cancel":
            if self.state == "driving":
                self._stop_and_report("failed cancelled by mission")
        else:
            self.get_logger().warn(f"unknown drive command {command!r}")

    # -------------------------------------------------------------- control

    def _publish_cmd(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    def _stop_and_report(self, status):
        self._publish_cmd(0.0, 0.0)
        self.zero_pulses_left = int(self.rate_hz * 0.6)  # keep publishing zeros
        self._publish_status(status)
        self.get_logger().info(status)

    def _tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # sim time not running yet

        if self.zero_pulses_left > 0:
            self._publish_cmd(0.0, 0.0)
            self.zero_pulses_left -= 1

        if self.state != "driving":
            return

        if self.pose is None or self.last_odom_time is None:
            return
        if (now - self.last_odom_time) > Duration(seconds=self.odom_timeout):
            self._stop_and_report("failed odometry timeout")
            return
        if (now - self.drive_start_time) > Duration(seconds=self.drive_timeout):
            self._stop_and_report("failed drive timeout")
            return

        x, y, yaw = self.pose
        error_x = self.target_x - x
        error_y = self.target_y - y

        if abs(error_x) <= self.tolerance:
            self._stop_and_report(f"done x={x:.3f} y={y:.3f}")
            return

        vx = _clamp(self.kp_linear * error_x, self.max_linear)
        # Hold the spawn row: drift in y steers back, heading stays near 0.
        wz = _clamp(self.kp_lateral * error_y - self.kp_yaw * yaw,
                    self.max_angular)
        self._publish_cmd(vx, wz)


def main(argv=None):
    rclpy.init(args=argv)
    node = OdomDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
