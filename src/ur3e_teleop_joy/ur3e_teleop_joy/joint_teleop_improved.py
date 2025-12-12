#!/usr/bin/env python3
"""
Improved joint-space JointJog teleoperation node.

Enhancements:
    1. Added max_joint_speed parameter to clamp velocity range.
    2. Fixed QoS profile (depth 1, real-time friendly).
    3. Better validation for joint_axes length vs. joint_names.
    4. publish_zero_jog() and publish_joint_jog() extracted as helpers.
    5. Velocity threshold tightened to 1e-4 for more responsive detection.
    6. Added clamp() utility to keep velocities inside limits.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog


def clip_deadzone(value: float, deadzone: float) -> float:
    """Zero-out small magnitudes to suppress controller wobble."""
    if abs(value) < deadzone:
        return 0.0
    return value


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Limit the value to the inclusive range."""
    return max(min(value, max_val), min_val)


class UR3eJointTeleop(Node):
    """
        Gamepad-driven jog controller operating directly in joint space.

        Data flow:
            /joy  ->  parse axes -> build JointJog -> /ur3e_servo/joint_jog
            MoveIt Servo converts JointJog into JointTrajectory -> scaled_joint_trajectory_controller

        Key characteristics:
            - RB button acts as a deadman switch: hold to move, release to send a zero command.
            - Per-joint axis mapping with -1 meaning "disabled".
            - Every joint velocity is clamped to [-max_speed, max_speed].
            - QoS tuned for low latency (depth 1).
            - Guarded axis index checks.
    """

    def __init__(self):
        super().__init__('ur3e_joint_teleop')

        p = self.declare_parameter

    # ---- Parameters ----
        self.joy_topic = p('joy_topic', '/joy').value
        self.enable_button = p('enable_button', 5).value
        self.deadzone = p('deadzone', 0.15).value
        self.joint_scale = p('joint_scale', 1.0).value
        
    # Maximum joint speed (rad/s)
        self.max_joint_speed = p('max_joint_speed', 2.0).value

        self.joint_names = p(
            'joint_names',
            [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint',
            ],
        ).value

        default_axes = [1, 0, 4, 3, -1, -1]
        self.joint_axes = p('joint_axes', default_axes).value
        
        # Validate axis mapping length against joint count
        if len(self.joint_axes) != len(self.joint_names):
            self.get_logger().warn(
                f"joint_axes length {len(self.joint_axes)} != joint_names length {len(self.joint_names)}; "
                f"using the shorter length"
            )

        self.joint_jog_topic = p('joint_jog_topic', '/ur3e_servo/joint_jog').value

        # Track whether the enable button was pressed last frame
        self.last_pressed = False

        # Real-time friendly QoS: depth 1, best-effort
        real_time_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Communication wiring
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, real_time_qos
        )
        self.joint_jog_pub = self.create_publisher(
            JointJog, self.joint_jog_topic, real_time_qos
        )

        self.get_logger().info(
            f"UR3eJointTeleop started: subscribing {self.joy_topic}, publishing JointJog to {self.joint_jog_topic}, "
            f"max_joint_speed={self.max_joint_speed} rad/s"
        )

    def joy_callback(self, msg: Joy):
        """
        Handle incoming Joy messages from the controller.

        Logic:
          1. If the enable button is not pressed:
             - When the last frame was pressed, send one zero command as a brake.
             - Otherwise do nothing.
          2. If the enable button is held:
             - Read all joint axes, apply deadzone and scaling.
             - Clamp to [-max_joint_speed, max_joint_speed].
             - If everything is near zero, skip publishing to save bandwidth.
             - Otherwise send a JointJog command.
        """
        # Check the deadman button (RB by default)
        pressed = False
        if 0 <= self.enable_button < len(msg.buttons):
            pressed = msg.buttons[self.enable_button] > 0

        # =====================================================================
        #     Safety: release RB -> send one zero-velocity brake command
        # =====================================================================
        if not pressed:
            if self.last_pressed:
                self.publish_zero_jog()
            self.last_pressed = False
            return

        # =====================================================================
        #     Active state: RB held, perform normal jog control
        # =====================================================================
        axes = msg.axes
        num_joints = len(self.joint_names)
        num_axes_config = len(self.joint_axes)
        
        # Use the shorter length to avoid out-of-range access
        num_controlled = min(num_joints, num_axes_config)
        
        velocities = [0.0] * num_joints

        for i in range(num_controlled):
            axis_index = self.joint_axes[i]
            
            # Negative index disables the joint
            if axis_index < 0:
                velocities[i] = 0.0
                continue
            
            # Validate axis index against Joy message
            if axis_index >= len(axes):
                self.get_logger().warn_once(
                    f'axis_index {axis_index} (joint {i}) exceeds Joy.axes length {len(axes)}; '
                    f'please check joint_axes parameter'
                )
                velocities[i] = 0.0
                continue

            raw = axes[axis_index]
            v = clip_deadzone(raw, self.deadzone)
            
            # Apply scaling and clamp to the max speed
            v = v * self.joint_scale
            v = clamp(v, -self.max_joint_speed, self.max_joint_speed)
            
            velocities[i] = v

        # All velocities near zero: skip publishing, keep last_pressed=True for braking logic
        if all(abs(v) < 1e-4 for v in velocities):
            self.last_pressed = True
            return

        # =====================================================================
        #     Publish a JointJog command
        # =====================================================================
        self.publish_joint_jog(velocities)
        self.last_pressed = True

    def publish_zero_jog(self):
        """Publish a zero-velocity JointJog message as a brake."""
        zero_jog = JointJog()
        zero_jog.header.stamp = self.get_clock().now().to_msg()
        zero_jog.joint_names = list(self.joint_names)
        zero_jog.velocities = [0.0] * len(self.joint_names)
        # Critical: keep displacements empty and duration=0 so Servo treats this as velocity only
        zero_jog.displacements = []
        zero_jog.duration = 0.0
        self.joint_jog_pub.publish(zero_jog)
        self.get_logger().debug("Sent zero JointJog brake command")

    def publish_joint_jog(self, velocities: list):
        """
        Publish a JointJog command.

        Args:
            velocities: Joint velocity list [rad/s]
        """
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.joint_names = list(self.joint_names)
        jog.velocities = velocities
        # Keep displacements empty and duration=0 to mark this as a pure velocity command
        jog.displacements = []
        jog.duration = 0.0
        self.joint_jog_pub.publish(jog)


def main(args=None):
    rclpy.init(args=args)
    node = UR3eJointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
