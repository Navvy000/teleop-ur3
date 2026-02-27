#!/usr/bin/env python3
"""Bridge APF /virtual_force to Virtuose impedance force feedback."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform

from raptor_api_interfaces.msg import InVirtuoseForce, OutVirtuoseStatus
from raptor_api_interfaces.srv import VirtuoseImpedance


@dataclass
class ForceState:
    force: Vector3 | None = None
    last_force_ns: int = 0
    status: OutVirtuoseStatus | None = None
    client_id: int = 0


class VirtuoseForceBridge(Node):
    def __init__(self) -> None:
        super().__init__('ur3e_virtuose_force_bridge')

        p = self.declare_parameter

        # Topics
        self.force_topic = p('force_topic', '/virtual_force').value
        self.status_topic = p('status_topic', '/out_virtuose_status').value
        self.impedance_service = p('impedance_service', '/virtuose_impedance').value

        # Virtuose connection parameters
        self.channel = p('channel', 'SimpleChannelUDP').value
        self.ff_device_ip_address = p('ff_device_ip_address', '').value
        self.ff_device_param_file = p('ff_device_param_file', '').value
        self.local_ip_address = p('local_ip_address', '').value

        self.speed_factor = float(p('speed_factor', 1.0).value)
        self.force_factor = float(p('force_factor', 1.0).value)
        self.max_force = float(p('max_force', 12.0).value)
        self.max_torque = float(p('max_torque', 1.5).value)
        self.power_enable = bool(p('power_enable', True).value)

        self.base_translation = p('base_translation', [0.0, 0.0, 0.0]).value
        self.base_rotation = p('base_rotation', [0.0, 0.0, 0.0, 1.0]).value

        # Safety/gating
        self.require_state = p('require_state', [4]).value
        self.deadman_mask = int(p('deadman_mask', 2).value)
        self.enable_deadman = bool(p('enable_deadman', True).value)
        self.enable_status_gate = bool(p('enable_status_gate', True).value)
        self.timeout_ms = float(p('timeout_ms', 50.0).value)

        # Scaling and filtering
        self.force_scale = float(p('force_scale', 0.2).value)
        self.torque_scale = float(p('torque_scale', 0.2).value)
        self.force_sign = p('force_sign', [1.0, 1.0, 1.0]).value
        self.torque_sign = p('torque_sign', [1.0, 1.0, 1.0]).value
        self.lowpass_alpha = float(p('lowpass_alpha', 0.8).value)

        real_time_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.state = ForceState()

        self.force_sub = self.create_subscription(
            Vector3, self.force_topic, self._on_force, real_time_qos
        )
        self.status_sub = self.create_subscription(
            OutVirtuoseStatus, self.status_topic, self._on_status, real_time_qos
        )
        self.force_pub = self.create_publisher(InVirtuoseForce, 'in_virtuose_force', real_time_qos)

        self.impedance_client = self.create_client(VirtuoseImpedance, self.impedance_service)
        self.init_timer = self.create_timer(1.0, self._ensure_impedance)
        self.loop_timer = self.create_timer(0.02, self._publish_cycle)  # 50 Hz

        self.filtered_force = [0.0, 0.0, 0.0]

        self.get_logger().info(
            f"Virtuose force bridge ready. force_topic={self.force_topic}, impedance_service={self.impedance_service}"
        )

    def _on_force(self, msg: Vector3) -> None:
        self.state.force = msg
        self.state.last_force_ns = self.get_clock().now().nanoseconds

    def _on_status(self, msg: OutVirtuoseStatus) -> None:
        self.state.status = msg

    def _ensure_impedance(self) -> None:
        if self.state.client_id != 0:
            return
        if not self.impedance_client.service_is_ready():
            self.get_logger().info('Waiting for virtuose_impedance service...')
            return

        req = VirtuoseImpedance.Request()
        req.channel = str(self.channel)
        req.ff_device_ip_address = str(self.ff_device_ip_address)
        req.ff_device_param_file = str(self.ff_device_param_file)
        req.local_ip_address = str(self.local_ip_address)
        req.speed_factor = float(self.speed_factor)
        req.force_factor = float(self.force_factor)
        req.max_force = float(self.max_force)
        req.max_torque = float(self.max_torque)
        req.power_enable = bool(self.power_enable)

        base = Transform()
        base.translation.x = float(self.base_translation[0])
        base.translation.y = float(self.base_translation[1])
        base.translation.z = float(self.base_translation[2])
        base.rotation.x = float(self.base_rotation[0])
        base.rotation.y = float(self.base_rotation[1])
        base.rotation.z = float(self.base_rotation[2])
        base.rotation.w = float(self.base_rotation[3])
        req.base_frame = base

        future = self.impedance_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            self.get_logger().error('Impedance service call timed out')
            return

        resp = future.result()
        if not resp or not resp.success or resp.client_id == 0:
            self.get_logger().error('Impedance service failed; check device and parameters')
            return

        self.state.client_id = int(resp.client_id)
        self.get_logger().info(f'Impedance enabled. client_id={self.state.client_id}')

    def _publish_cycle(self) -> None:
        if not self._is_enabled():
            self._publish_zero()
            return

        if self.state.force is None:
            self._publish_zero()
            return

        f = self._map_force(self.state.force)
        self._publish_force(f)

    def _is_enabled(self) -> bool:
        if self.state.client_id == 0:
            return False
        if self.state.last_force_ns == 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        age_ms = (now_ns - self.state.last_force_ns) / 1e6
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
            return (status.buttons & self.deadman_mask) != 0

        return True

    def _map_force(self, msg: Vector3) -> List[float]:
        raw = [msg.x, msg.y, msg.z]
        mapped = [
            raw[0] * self.force_sign[0] * self.force_scale,
            raw[1] * self.force_sign[1] * self.force_scale,
            raw[2] * self.force_sign[2] * self.force_scale,
        ]
        mapped = self._limit_vector(mapped, self.max_force)

        # Low-pass filter
        alpha = self.lowpass_alpha
        self.filtered_force = [
            alpha * self.filtered_force[0] + (1 - alpha) * mapped[0],
            alpha * self.filtered_force[1] + (1 - alpha) * mapped[1],
            alpha * self.filtered_force[2] + (1 - alpha) * mapped[2],
        ]
        return self.filtered_force

    def _publish_force(self, force_xyz: List[float]) -> None:
        msg = InVirtuoseForce()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.client_id = self.state.client_id
        msg.virtuose_force.force.x = float(force_xyz[0])
        msg.virtuose_force.force.y = float(force_xyz[1])
        msg.virtuose_force.force.z = float(force_xyz[2])
        msg.virtuose_force.torque.x = 0.0
        msg.virtuose_force.torque.y = 0.0
        msg.virtuose_force.torque.z = 0.0
        self.force_pub.publish(msg)

    def _publish_zero(self) -> None:
        if self.state.client_id == 0:
            return
        msg = InVirtuoseForce()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.client_id = self.state.client_id
        self.force_pub.publish(msg)

    @staticmethod
    def _limit_vector(vec: List[float], max_norm: float) -> List[float]:
        if max_norm <= 0:
            return vec
        norm = (vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2) ** 0.5
        if norm <= max_norm or norm == 0:
            return vec
        scale = max_norm / norm
        return [v * scale for v in vec]


def main() -> None:
    rclpy.init()
    node = VirtuoseForceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
