#!/usr/bin/env python3
"""
virtual_force_node.py

Compute the minimum distance between the UR3e end-effector (tool0) and a
box-shaped obstacle in real time, generate a virtual force vector F = (Fx, Fy, Fz)
when the distance lies within 0.15 m ~ 0.02 m, and publish it on the
/virtual_force topic (geometry_msgs/Vector3).

Assumptions:
- Obstacle parameters are sourced from ur3e_haptic_scene/config/obstacles.yaml box1:
    size: [sx, sy, sz]
    pose.xyz: [cx, cy, cz]
    frame: base_link
- The box is axis-aligned with the base_link frame (rpy = [0,0,0]).
- Virtual force direction points from obstacle surface to the end-effector (repulsive field).
"""

import math
import os
import yaml
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped

import tf2_ros


def clamp(value: float, min_v: float, max_v: float) -> float:
    """Clamp a scalar between min_v and max_v."""
    return max(min_v, min(max_v, value))


class VirtualForceNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3e_virtual_force")

        # ---- Configuration ----
        # Safety and stop distances (meters)
        self.d_safe = 0.15  # 15 cm begins to generate virtual force
        self.d_stop = 0.02  # 2 cm hits the maximum force

        # Peak magnitude for the virtual force (N or unitless scale)
        self.F_max = 35.0

        # Frames for obstacle and end-effector (defaults: base_link, tool0)
        self.base_frame = "base_link"
        self.ee_frame = "tool0"

        # TF buffer + listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for the virtual force publisher
        force_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.force_pub = self.create_publisher(Vector3, "virtual_force", force_qos)

        # Load obstacle configuration
        success = self._load_obstacle_config()
        if not success:
            self.get_logger().error("Failed to load obstacle config. Node will still run but F=0.")
            # Allow the node to continue running; it will simply output zero force

        # Timer: 50 Hz (20 ms)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self._timer_callback)

        self.get_logger().info("VirtualForceNode started. Listening to TF and publishing /virtual_force.")

    # ----------------------------------------------------------------------
    # Configuration loading
    # ----------------------------------------------------------------------
    def _load_obstacle_config(self) -> bool:
        """Load the box1 obstacle definition from obstacles.yaml."""
        try:
            pkg_share = get_package_share_directory("ur3e_haptic_scene")
            yaml_path = os.path.join(pkg_share, "config", "obstacles.yaml")
            self.get_logger().info(f"Loading obstacle config from: {yaml_path}")

            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)

            if not data or "obstacles" not in data:
                self.get_logger().error("YAML missing 'obstacles' root key.")
                return False

            obstacles = data["obstacles"]
            if "box1" not in obstacles:
                self.get_logger().error("No 'box1' entry in obstacles.yaml.")
                return False

            box_cfg = obstacles["box1"]
            self.box_size = box_cfg.get("size", [0.1, 0.1, 0.1])
            pose_cfg = box_cfg.get("pose", {})
            self.box_center = pose_cfg.get("xyz", [0.5, 0.0, 0.2])
            self.box_frame = box_cfg.get("frame", "base_link")

            # Current implementation expects box_frame == base_frame; no transform applied
            if self.box_frame != self.base_frame:
                self.get_logger().warn(
                    f"box frame '{self.box_frame}' != base_frame '{self.base_frame}'. "
                    "Current implementation assumes they match; add TF conversion if needed."
                )

            self.get_logger().info(
                f"Loaded box1: center={self.box_center}, size={self.box_size}, frame={self.box_frame}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Exception while reading obstacles.yaml: {e}")
            return False

    # ----------------------------------------------------------------------
    # Main loop
    # ----------------------------------------------------------------------
    def _timer_callback(self) -> None:
        """Timer callback: fetch end-effector pose, compute d and F(d), publish force."""
        # Lookup end-effector position in base frame
        transform = self._lookup_transform(self.base_frame, self.ee_frame)
        if transform is None:
            # TF unavailable; publish zero force
            self._publish_force(0.0, 0.0, 0.0)
            return

        px = transform.transform.translation.x
        py = transform.transform.translation.y
        pz = transform.transform.translation.z

        # Compute distance and nearest point from tool to the box
        d, nearest_point = self._point_to_box_distance(
            px, py, pz,
            self.box_center,
            self.box_size,
        )

        # Configuration might have failed; bail out if data missing
        if d is None:
            self._publish_force(0.0, 0.0, 0.0)
            return

        # Map distance d to scaling factor s(d)
        s = self._scaling_from_distance(d)

        # Final force magnitude
        F_mag = self.F_max * s

        if d <= 0.0 or F_mag <= 0.0:
            # Inside the box or too far away (zero scaling)
            Fx = Fy = Fz = 0.0
        else:
            # Direction vector from nearest point towards the end-effector
            vx = px - nearest_point[0]
            vy = py - nearest_point[1]
            vz = pz - nearest_point[2]

            norm_v = math.sqrt(vx * vx + vy * vy + vz * vz)
            if norm_v < 1e-6:
                Fx = Fy = Fz = 0.0
            else:
                ux = vx / norm_v
                uy = vy / norm_v
                uz = vz / norm_v

                Fx = F_mag * ux
                Fy = F_mag * uy
                Fz = F_mag * uz

        # Publish force vector
        self._publish_force(Fx, Fy, Fz)

        # Lightweight debug logging (avoid spamming)
        self.get_logger().info(
            f"d = {d:.3f} m, s = {s:.2f}, F = ({Fx:.2f}, {Fy:.2f}, {Fz:.2f})"
        )

    # ----------------------------------------------------------------------
    # TF lookup
    # ----------------------------------------------------------------------
    def _lookup_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        """Lookup TF transform: target_frame <- source_frame."""
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # latest available
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed ({target_frame} <- {source_frame}): {e}"
            )
            return None

    # ----------------------------------------------------------------------
    # Geometry: point-to-axis-aligned box distance
    # ----------------------------------------------------------------------
    def _point_to_box_distance(
        self,
        px: float,
        py: float,
        pz: float,
        center: list,
        size: list,
    ) -> Tuple[Optional[float], Optional[Tuple[float, float, float]]]:
        """Return the distance and closest point from (px, py, pz) to an axis-aligned box.

        Assumes:
        - Box aligned with base_frame axes (rpy=[0,0,0]).
        - center, size are lists [x, y, z].
        """
        if center is None or size is None or len(center) != 3 or len(size) != 3:
            self.get_logger().error("Invalid box center/size. Cannot compute distance.")
            return None, None

        cx, cy, cz = center
        sx, sy, sz = size
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

    # Box extents around the center
        min_x, max_x = cx - hx, cx + hx
        min_y, max_y = cy - hy, cy + hy
        min_z, max_z = cz - hz, cz + hz

    # Project the point onto the axis-aligned bounding box (clamp)
        nx = clamp(px, min_x, max_x)
        ny = clamp(py, min_y, max_y)
        nz = clamp(pz, min_z, max_z)

        dx = px - nx
        dy = py - ny
        dz = pz - nz

        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        return d, (nx, ny, nz)

    # ----------------------------------------------------------------------
    # Distance-to-force scaling
    # ----------------------------------------------------------------------
    def _scaling_from_distance(self, d: float) -> float:
        """Map distance d to a [0,1] scaling factor s(d)."""
        if d >= self.d_safe:
            return 0.0
        if d <= self.d_stop:
            return 1.0

        # Linear interpolation: s ramps from 0 to 1 as d goes d_safe -> d_stop
        return (self.d_safe - d) / (self.d_safe - self.d_stop)

    # ----------------------------------------------------------------------
    # Publish virtual force
    # ----------------------------------------------------------------------
    def _publish_force(self, Fx: float, Fy: float, Fz: float) -> None:
        msg = Vector3()
        msg.x = float(Fx)
        msg.y = float(Fy)
        msg.z = float(Fz)
        self.force_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down VirtualForceNode.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
