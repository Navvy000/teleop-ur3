#!/usr/bin/env python3
"""
Improved Cartesian Twist teleoperation node.

Enhancements:
    1. QoS tuned for real-time latency (depth 1, best-effort).
    2. Additional max_linear_speed and max_angular_speed parameters.
    3. Independent linear and angular speed limiting.
    4. Better error handling and logging.
    5. publish_zero_twist() extracted as a dedicated helper.
    6. Clearer command-frame documentation.
    7. Safer axis index checks (warn instead of warn_once).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math


def clip_deadzone(value: float, deadzone: float) -> float:
    """Clamp small magnitudes to zero to remove stick drift."""
    if abs(value) < deadzone:
        return 0.0
    return value


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Limit the value to the provided range."""
    return max(min(value, max_val), min_val)


class UR3eCartesianTeleop(Node):
    """
        Teleoperation node that drives the end-effector in Cartesian space.

        Data flow:
            /joy  ->  parse axes -> build TwistStamped -> /ur3e_servo/cartesian_twist
            MoveIt Servo (Twist mode) converts the twist to JointTrajectory -> scaled_joint_trajectory_controller

        Key characteristics:
            - Uses the tool0 frame as the command frame (frame_id = tool0).
            - RB button acts as a deadman switch: hold to move, release to send a zero twist brake.
            - Linear velocities are expressed in m/s, angular velocities in rad/s.
            - Independent limits for linear and angular speed magnitudes.
            - QoS tuned for low latency.
    """

    def __init__(self):
        super().__init__('ur3e_cartesian_teleop')

        p = self.declare_parameter

    # --------------------
    # Core topic configuration
    # --------------------
        self.joy_topic = p('joy_topic', '/joy').value
        self.twist_topic = p('twist_topic', '/ur3e_servo/cartesian_twist').value

    # Tool frame / command frame name.
        self.command_frame = p('command_frame', 'tool0').value

    # --------------------
    # Gamepad and safety parameters
    # --------------------
        self.enable_button = p('enable_button', 5).value
        self.deadzone = p('deadzone', 0.15).value
        self.linear_scale = p('linear_scale', 0.1).value
        self.angular_scale = p('angular_scale', 0.8).value
        
        self.max_linear_speed = p('max_linear_speed', 0.5).value
        self.max_angular_speed = p('max_angular_speed', 1.0).value

    # --------------------
    # Axis mapping
    # --------------------
        self.axis_left_x  = p('axis_left_x', 0).value
        self.axis_left_y  = p('axis_left_y', 1).value
        self.axis_right_x = p('axis_right_x', 3).value
        self.axis_right_y = p('axis_right_y', 4).value
        self.axis_lt      = p('axis_lt', 2).value
        self.axis_rt      = p('axis_rt', 5).value
        self.axis_dpad_x  = p('axis_dpad_x', 6).value
        self.dpad_angular_speed = p('dpad_angular_speed', 0.8).value

        self.last_pressed = False

    # --------------------
    # Real-time QoS profile
    # --------------------
        real_time_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    # --------------------
    # Subscriptions and publishers
    # --------------------
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, real_time_qos
        )
        self.twist_pub = self.create_publisher(
            TwistStamped, self.twist_topic, real_time_qos
        )

        self.get_logger().info(
            f"UR3eCartesianTeleop started: subscribing {self.joy_topic}, publishing TwistStamped to {self.twist_topic}, "
            f"command_frame={self.command_frame}, "
            f"max_linear_speed={self.max_linear_speed} m/s, "
            f"max_angular_speed={self.max_angular_speed} rad/s"
        )

    # ------------------------------------------------------------------
    # Helper: safely fetch axis values (return 0 when index is out of range)
    # ------------------------------------------------------------------
    def _get_axis(self, axes, index: int, name: str) -> float:
        if index < 0:
            return 0.0
        if index >= len(axes):
            self.get_logger().warn(
                f'Axis index {index} ({name}) exceeds Joy.axes length {len(axes)}. Please check axis_* parameter settings.'
            )
            return 0.0
        return axes[index]

    # ------------------------------------------------------------------
    # Helper: publish a zero twist as an emergency brake
    # ------------------------------------------------------------------
    def publish_zero_twist(self):
        """Publish a zero-velocity TwistStamped message as a brake."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame

        self.twist_pub.publish(twist)
        self.get_logger().debug("Sent zero twist brake command")

    # ------------------------------------------------------------------
    def publish_twist(self, vx: float, vy: float, vz: float,
                      wx: float, wy: float, wz: float):
        """Publish a TwistStamped command."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame

        twist.twist.linear.x  = vx
        twist.twist.linear.y  = vy
        twist.twist.linear.z  = vz
        twist.twist.angular.x = wx
        twist.twist.angular.y = wy
        twist.twist.angular.z = wz

        self.twist_pub.publish(twist)

    # ------------------------------------------------------------------
    # Joy Callback
    # ------------------------------------------------------------------
    def joy_callback(self, msg: Joy):
        pressed = False
        if 0 <= self.enable_button < len(msg.buttons):
            pressed = msg.buttons[self.enable_button] > 0

        if not pressed:
            if self.last_pressed:
                self.publish_zero_twist()
            self.last_pressed = False
            return

        axes = msg.axes

        raw_lx = self._get_axis(axes, self.axis_left_x,  "axis_left_x")
        raw_ly = self._get_axis(axes, self.axis_left_y,  "axis_left_y")
        raw_rx = self._get_axis(axes, self.axis_right_x, "axis_right_x")
        raw_ry = self._get_axis(axes, self.axis_right_y, "axis_right_y")
        raw_lt = self._get_axis(axes, self.axis_lt,      "axis_lt")
        raw_rt = self._get_axis(axes, self.axis_rt,      "axis_rt")
        raw_dpad_x = self._get_axis(axes, self.axis_dpad_x, "axis_dpad_x")

        lx = clip_deadzone(raw_lx, self.deadzone)
        ly = clip_deadzone(raw_ly, self.deadzone)
        rx = clip_deadzone(raw_rx, self.deadzone)
        ry = clip_deadzone(raw_ry, self.deadzone)
        lt = clip_deadzone(raw_lt, self.deadzone)
        rt = clip_deadzone(raw_rt, self.deadzone)

        # D-pad usually snaps to -1/0/1; treat as digital for yaw.
        dpad_left = raw_dpad_x < -0.5
        dpad_right = raw_dpad_x > 0.5

        # ----------------------------
        # Map joystick axes to velocity components
        # ----------------------------
        # Left stick mapping in tool0 frame:
        #   push forward (ly < 0) -> +Y translation
        #   push left    (lx < 0) -> +X translation
        vx = -lx * self.linear_scale
        vy = -ly * self.linear_scale
        vz = (rt - lt) * self.linear_scale

        # Right stick -> roll/pitch (swapped):
        #   up/down controls roll (X), left/right controls pitch (Y)
        # D-pad -> yaw
        wx = -ry * self.angular_scale   # roll around tool0 X (from vertical axis)
        wy =  rx * self.angular_scale   # pitch around tool0 Y (from horizontal axis)

        if dpad_left and not dpad_right:
            wz = -float(self.dpad_angular_speed)
        elif dpad_right and not dpad_left:
            wz = float(self.dpad_angular_speed)
        else:
            wz = 0.0

        # ----------------------------
        # Speed limiting
        # ----------------------------
        linear_speed = math.sqrt(vx**2 + vy**2 + vz**2)
        angular_speed = math.sqrt(wx**2 + wy**2 + wz**2)

        if linear_speed > self.max_linear_speed:
            scale = self.max_linear_speed / linear_speed
            vx *= scale
            vy *= scale
            vz *= scale

        if angular_speed > self.max_angular_speed:
            scale = self.max_angular_speed / angular_speed
            wx *= scale
            wy *= scale
            wz *= scale

        if (abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(vz) < 1e-4 and
            abs(wx) < 1e-4 and abs(wy) < 1e-4 and abs(wz) < 1e-4):
            self.last_pressed = True
            return

        self.publish_twist(vx, vy, vz, wx, wy, wz)
        self.last_pressed = True


def main(args=None):
    rclpy.init(args=args)
    node = UR3eCartesianTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
