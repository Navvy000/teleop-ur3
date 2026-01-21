#!/usr/bin/env python3
"""
Add obstacles to MoveIt 2 Planning Scene using the /planning_scene topic.

Workflow:
    1. Load obstacle parameters from config/obstacles.yaml inside this package.
    2. Build a moveit_msgs/CollisionObject (single box).
    3. Wrap it into moveit_msgs/PlanningScene with is_diff=True.
    4. Publish on the /planning_scene topic.
    5. RViz MoveIt Planning Scene reflects the same obstacle.
"""

import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class AddObstacles(Node):
    def __init__(self) -> None:
        super().__init__("ur3e_add_obstacles")

    # QoS: reliable publisher keeping the latest message
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

    # Publisher for MoveIt /planning_scene topic
        self._pub = self.create_publisher(
            PlanningScene, "planning_scene", qos
        )

        self.get_logger().info("AddObstacles node started.")

    # Read obstacle configuration
        obstacles = self._load_obstacles_yaml()
        if obstacles is None:
            self.get_logger().error("Failed to load obstacles.yaml, exiting.")
            rclpy.shutdown()
            return

        if len(obstacles) == 0:
            self.get_logger().error("No obstacles found in obstacles.yaml.")
            rclpy.shutdown()
            return

    # Wait until /planning_scene has subscribers (typically move_group)
        self._wait_for_subscribers()

    # Build CollisionObjects for all entries
        collision_objects = []
        for obj_id, cfg in obstacles.items():
            try:
                collision_objects.append(self._make_box_collision_object(obj_id, cfg))
            except Exception as e:
                self.get_logger().error(f"Failed to construct CollisionObject for '{obj_id}': {e}")

    # Populate a PlanningScene diff message
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)

    # Publish once
        self._pub.publish(scene)
        self.get_logger().info(
            f"Published PlanningScene diff with {len(collision_objects)} object(s): {list(obstacles.keys())}"
        )

    # Wait briefly to ensure the message is sent
        time.sleep(1.0)
        self.get_logger().info("AddObstacles node finished, shutting down.")
        rclpy.shutdown()

    # ----------------- Helper functions -----------------

    def _load_obstacles_yaml(self):
        """Read config/obstacles.yaml."""
        try:
            pkg_share = get_package_share_directory("ur3e_haptic_scene")
            yaml_path = os.path.join(pkg_share, "config", "obstacles.yaml")
            self.get_logger().info(f"Loading obstacles from: {yaml_path}")

            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)

            if not data or "obstacles" not in data:
                self.get_logger().error("YAML file has no 'obstacles' root key.")
                return None

            return data["obstacles"]
        except Exception as e:
            self.get_logger().error(f"Exception while reading YAML: {e}")
            return None

    def _wait_for_subscribers(self):
        """Wait for a subscriber on /planning_scene (move_group)."""
        self.get_logger().info("Waiting for subscribers on 'planning_scene' ...")
        while rclpy.ok() and self._pub.get_subscription_count() == 0:
            self.get_logger().info("  still waiting...")
            time.sleep(0.5)
        self.get_logger().info("Found subscriber(s) on 'planning_scene'.")

    def _make_box_collision_object(self, obj_id: str, cfg: dict) -> CollisionObject:
        """Construct a box CollisionObject based on YAML configuration."""

        size = cfg.get("size", [0.1, 0.1, 0.1])
        pose_cfg = cfg.get("pose", {})
        xyz = pose_cfg.get("xyz", [0.5, 0.0, 0.2])
        rpy = pose_cfg.get("rpy", [0.0, 0.0, 0.0])
        frame = cfg.get("frame", "base_link")

    # SolidPrimitive (BOX)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)  # [x, y, z]

    # Pose: rpy is zero so the quaternion defaults to (0, 0, 0, 1)
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # CollisionObject
        obj = CollisionObject()
        obj.id = obj_id
        obj.header.frame_id = frame
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        self.get_logger().info(
            f"Constructed CollisionObject '{obj_id}' in frame '{frame}' "
            f"at xyz={xyz}, size={size}"
        )

        return obj


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AddObstacles()
    # rclpy.spin is not strictly needed because the constructor already shut down.
    # Keep this fallback in case the constructor returns early.
    if rclpy.ok():
        rclpy.spin(node)
    # Normally we never reach this point
    try:
        node.destroy_node()
    except Exception:
        pass


if __name__ == "__main__":
    main()

