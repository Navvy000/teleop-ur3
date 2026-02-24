#!/usr/bin/env python3
"""Bridge Virtuose output to UR3e MoveIt Servo Twist commands."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import TwistStamped
from raptor_api_interfaces.msg import OutVirtuoseSpeed, OutVirtuoseStatus


@dataclass
class VirtuoseState:
    speed: OutVirtuoseSpeed | None = None
    status: OutVirtuoseStatus | None = None
    last_speed_ns: int = 0


class VirtuoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('ur3e_virtuose_bridge')

        p = self.declare_parameter
        # Topics
        self.speed_topic = p('speed_topic', '/out_virtuose_speed').value
        self.status_topic = p('status_topic', '/out_virtuose_status').value
        self.twist_topic = p('twist_topic', '/ur3e_servo/cartesian_twist').value
        self.command_frame = p('command_frame', 'tool0').value

        # Safety
        self.require_state = p('require_state', [5]).value
        self.deadman_mask = p('deadman_mask', 2).value
        self.enable_deadman = p('enable_deadman', True).value
        self.enable_status_gate = p('enable_status_gate', True).value
        self.timeout_ms = float(p('timeout_ms', 50.0).value)

        # Mapping & scaling
        self.linear_map = p('linear_map', [0, 1, 2]).value
        self.angular_map = p('angular_map', [0, 1, 2]).value
        self.linear_sign = p('linear_sign', [1.0, 1.0, 1.0]).value
        self.angular_sign = p('angular_sign', [1.0, 1.0, 1.0]).value
        self.linear_scale = float(p('linear_scale', 0.2).value)
        self.angular_scale = float(p('angular_scale', 0.5).value)
        self.max_linear_speed = float(p('max_linear_speed', 0.25).value)
        self.max_angular_speed = float(p('max_angular_speed', 1.0).value)

        real_time_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.state = VirtuoseState()

        self.speed_sub = self.create_subscription(
            OutVirtuoseSpeed, self.speed_topic, self._on_speed, real_time_qos
        )
        self.status_sub = self.create_subscription(
            OutVirtuoseStatus, self.status_topic, self._on_status, real_time_qos
        )
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, real_time_qos)

        self.timer = self.create_timer(0.02, self._publish_cycle)  # 50 Hz

        self.get_logger().info(
            f"Virtuose bridge running. speed_topic={self.speed_topic}, status_topic={self.status_topic}, "
            f"twist_topic={self.twist_topic}, frame={self.command_frame}"
        )

    def _on_speed(self, msg: OutVirtuoseSpeed) -> None:
        self.state.speed = msg
        self.state.last_speed_ns = self.get_clock().now().nanoseconds

    def _on_status(self, msg: OutVirtuoseStatus) -> None:
        self.state.status = msg

    def _publish_cycle(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if not self._is_enabled(now_ns):
            self._publish_zero()
            return

        if self.state.speed is None:
            self._publish_zero()
            return

        cmd = self._map_speed(self.state.speed)
        self.twist_pub.publish(cmd)

    def _is_enabled(self, now_ns: int) -> bool:
        if self.state.last_speed_ns == 0:
            return False
        age_ms = (now_ns - self.state.last_speed_ns) / 1e6
        if age_ms > self.timeout_ms:
            return False

        status = self.state.status
        if not status:
            return not self.enable_status_gate

        if status.emergency_stop:
            return False

        if self.enable_status_gate and self.require_state:
            if int(status.state) not in [int(s) for s in self.require_state]:
                return False

        if self.enable_deadman:
            return (status.buttons & int(self.deadman_mask)) != 0

        return True

    def _map_speed(self, msg: OutVirtuoseSpeed) -> TwistStamped:
        lin = [msg.virtuose_speed.linear.x,
               msg.virtuose_speed.linear.y,
               msg.virtuose_speed.linear.z]
        ang = [msg.virtuose_speed.angular.x,
               msg.virtuose_speed.angular.y,
               msg.virtuose_speed.angular.z]

        linear = self._apply_map(lin, self.linear_map, self.linear_sign, self.linear_scale)
        angular = self._apply_map(ang, self.angular_map, self.angular_sign, self.angular_scale)

        linear = self._limit_vector(linear, self.max_linear_speed)
        angular = self._limit_vector(angular, self.max_angular_speed)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]
        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]
        return twist

    def _apply_map(self, vec: List[float], idx_map: List[int], sign_map: List[float], scale: float) -> List[float]:
        out = [0.0, 0.0, 0.0]
        for i in range(3):
            src_i = idx_map[i] if i < len(idx_map) else i
            sign = sign_map[i] if i < len(sign_map) else 1.0
            if 0 <= src_i < len(vec):
                out[i] = vec[src_i] * sign * scale
        return out

    @staticmethod
    def _limit_vector(vec: List[float], max_norm: float) -> List[float]:
        if max_norm <= 0:
            return vec
        norm = (vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2) ** 0.5
        if norm <= max_norm or norm == 0:
            return vec
        scale = max_norm / norm
        return [v * scale for v in vec]

    def _publish_zero(self) -> None:
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        self.twist_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = VirtuoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
