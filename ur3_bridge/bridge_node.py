#!/usr/bin/env python3
import math
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class UR3Bridge(Node):
    """ROS 2 → CoppeliaSim one-way control node"""

    def __init__(self):
        super().__init__('ur3_bridge')

        # Parameter definitions
        self.declare_parameter('cmd_topic', '/ur3_cmd')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.joint_names = list(self.get_parameter('joint_names').value)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim Remote API')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            raise

        # Retrieve 6 joint handles
        self.joints = []
        for name in self.joint_names:
            path = f'/UR3/{name}'
            try:
                handle = self.sim.getObject(path)
                self.joints.append(handle)
            except Exception as e:
                self.get_logger().error(f'Cannot find object {path}: {e}')
                raise
        self.get_logger().info(f'Found joints: {self.joint_names}')

        # Subscribe to topic
        self.sub = self.create_subscription(
            Float64MultiArray,
            self.cmd_topic,
            self.cmd_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {self.cmd_topic} (Float64MultiArray len=6, unit: rad)')

    # Callback function: receive and send joint targets
    def cmd_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) != len(self.joints):
            self.get_logger().warn(f'Array length mismatch {len(data)}')
            return
        try:
            for j, angle in zip(self.joints, data):
                self.sim.setJointTargetPosition(j, float(angle))
            self.get_logger().info(f'Sent target angles {[round(a, 3) for a in data]}')
        except Exception as e:
            self.get_logger().error(f'Failed to send targets: {e}')


def main():
    rclpy.init()
    node = UR3Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
