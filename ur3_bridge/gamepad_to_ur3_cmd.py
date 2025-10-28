#!/usr/bin/env python3
import math
import time
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

def clamp(x, lo, hi): 
    return lo if x < lo else hi if x > hi else x

class GamepadToUR3Cmd(Node):
    """
    Read /joy (sensor_msgs/Joy) and publish /ur3_cmd (Float64MultiArray len=6).
    - Joint velocity mode (+ integration to target angles)
    - Deadzone, axis invert, trigger normalization
    - Enable(LB), E-Stop(B), Home(Y)
    """
    def __init__(self):
        super().__init__('gamepad_to_ur3_cmd')
        # ------------ Params (defaults tailored to your F310 readings) ------------
        self.declare_parameter('cmd_topic', '/ur3_cmd')
        self.declare_parameter('publish_rate', 50.0)          # Hz
        self.declare_parameter('deadzone', 0.08)               # for sticks
        self.declare_parameter('max_vel', 0.5)                 # rad/s per joint
        # axes index mapping (your real indices) + invert so“左/上”为负
        self.declare_parameter('axes.j1.index', 0)  # LX
        self.declare_parameter('axes.j1.invert', True)
        self.declare_parameter('axes.j2.index', 1)  # LY
        self.declare_parameter('axes.j2.invert', True)
        self.declare_parameter('axes.j3.index', 3)  # RX
        self.declare_parameter('axes.j3.invert', True)
        self.declare_parameter('axes.j4.index', 4)  # RY
        self.declare_parameter('axes.j4.invert', True)
        # triggers: raw 1 (released) -> -1 (pressed). Normalize to [0,1].
        self.declare_parameter('axes.j5.index', 2)  # LT = ABS_Z
        self.declare_parameter('axes.j6.index', 5)  # RT = ABS_RZ
        # buttons
        self.declare_parameter('btn.enable', 4)     # LB
        self.declare_parameter('btn.estop', 1)      # B
        self.declare_parameter('btn.home', 3)       # Y
        # joint limits (rad) — rough; adjust later to UR3 true limits if needed
        lim = 2.0 * math.pi
        self.declare_parameter('limits', [-lim, -lim, -lim, -lim, -lim, -lim,
                                           lim,  lim,  lim,  lim,  lim,  lim])  # [min1..6, max1..6]

        # ------------ Load params ------------
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.rate_hz = float(self.get_parameter('publish_rate').value)
        self.dead = float(self.get_parameter('deadzone').value)
        self.max_vel = float(self.get_parameter('max_vel').value)
        self.ax = {
            0: (int(self.get_parameter('axes.j1.index').value), bool(self.get_parameter('axes.j1.invert').value)),
            1: (int(self.get_parameter('axes.j2.index').value), bool(self.get_parameter('axes.j2.invert').value)),
            2: (int(self.get_parameter('axes.j3.index').value), bool(self.get_parameter('axes.j3.invert').value)),
            3: (int(self.get_parameter('axes.j4.index').value), bool(self.get_parameter('axes.j4.invert').value)),
            4: (int(self.get_parameter('axes.j5.index').value), False),  # LT normalize
            5: (int(self.get_parameter('axes.j6.index').value), False),  # RT normalize
        }
        lims = [float(v) for v in self.get_parameter('limits').value]
        self.jmin = lims[:6]; self.jmax = lims[6:]

        self.btn_enable = int(self.get_parameter('btn.enable').value)
        self.btn_estop  = int(self.get_parameter('btn.estop').value)
        self.btn_home   = int(self.get_parameter('btn.home').value)

        # ------------ ROS I/O ------------
        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, QoSProfile(depth=10))
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, QoSProfile(depth=10))

        # ------------ State ------------
        self.joy_axes: List[float] = []
        self.joy_buttons: List[int] = []
        self.targets = [0.0]*6
        self.enabled = False
        self.estop_latched = False

        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info('gamepad_to_ur3_cmd ready '
                               f'(rate={self.rate_hz}Hz, deadzone={self.dead}, max_vel={self.max_vel})')
        self.get_logger().info('Controls: LB=enable (hold), B=E-Stop, Y=Home')
        self.get_logger().info(f'Publishing to {self.cmd_topic}')

    # ------------ Helpers ------------
    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.dead:
            return 0.0
        # optional rescale outside DZ to keep full range:
        # return math.copysign((abs(v)-self.dead)/(1.0-self.dead), v)
        return v

    def trig_norm(self, raw: float) -> float:
        # raw: 1 (released) -> -1 (pressed)  => map to [0,1]
        v = (1.0 - raw) * 0.5
        return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)

    # ------------ Callbacks ------------
    def joy_cb(self, msg: Joy):
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)

        # Enable is “hold-to-run”
        self.enabled = (self.btn_enable < len(self.joy_buttons) and self.joy_buttons[self.btn_enable] == 1)

        # E-Stop (latching)
        if self.btn_estop < len(self.joy_buttons) and self.joy_buttons[self.btn_estop] == 1:
            self.estop_latched = True
        # Home
        if self.btn_home < len(self.joy_buttons) and self.joy_buttons[self.btn_home] == 1:
            self.targets = [0.0]*6
            self.get_logger().info('HOME → targets set to zeros')

    def tick(self):
        # No /joy yet → nothing to do
        if not self.joy_axes:
            return

        # Compute joint velocities from axes
        vels = [0.0]*6
        # sticks (j1..j4)
        for j in range(4):
            idx, inv = self.ax[j]
            if idx < len(self.joy_axes):
                val = self.apply_deadzone(self.joy_axes[idx])
                if inv:
                    val = -val
                vels[j] = val * self.max_vel

        # triggers (j5..j6)
        for j in [4, 5]:
            idx, _ = self.ax[j]
            if idx < len(self.joy_axes):
                v = self.trig_norm(self.joy_axes[idx])   # 0..1
                vels[j] = v * self.max_vel

        # Apply enable & estop
        if self.estop_latched:
            vels = [0.0]*6
        elif not self.enabled:
            vels = [0.0]*6

        # Integrate to target angles
        for i in range(6):
            self.targets[i] += vels[i] * self.dt
            self.targets[i] = clamp(self.targets[i], self.jmin[i], self.jmax[i])

        # Publish
        msg = Float64MultiArray()
        msg.data = self.targets
        self.pub.publish(msg)

        # Occasionally log
        # self.get_logger().info(f'targets: {[round(x,3) for x in self.targets]}')

def main():
    rclpy.init()
    node = GamepadToUR3Cmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
