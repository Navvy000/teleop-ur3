#!/usr/bin/env python3
"""
virtual_force_node.py

Compute an Artificial Potential Field (APF) virtual guidance force in real time.

This node implements a *unified object model*:
- The YAML file provides a set of homogeneous box objects.
- Exactly one object can be selected as the current target via `/selected_target` (std_msgs/String).
- The selected target contributes an attractive term F_att.
- All other objects contribute repulsive terms F_rep.

The published `/virtual_force` is the total force: F = F_att + sum(F_rep).

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
from geometry_msgs.msg import Point

import tf2_ros

from visualization_msgs.msg import Marker
from std_msgs.msg import String


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

        # Attractive force parameters
        # Desired behavior (your requirement):
        # - Far away: no attraction
        # - As EE approaches target from d_att_start -> d_att_peak: attraction increases linearly
        # - Very close (d_att_peak -> d_att_stop): attraction decreases linearly to 0
        # This creates a local "soft capture" effect and avoids strong pulling when near the target,
        # which plays nicely with MoveIt Servo collision checking.
        p = self.declare_parameter
        self.d_att_start = float(p("d_att_start", 0.20).value)  # meters: attraction starts (e.g. 20cm)
        self.d_att_peak = float(p("d_att_peak", 0.10).value)    # meters: attraction peak (e.g. 10cm)
        self.d_att_stop = float(p("d_att_stop", 0.02).value)    # meters: attraction goes to 0 (e.g. 2cm)
        self.F_att_peak = float(p("F_att_peak", 35.0).value)    # peak attractive magnitude

        # Keep backward-compatibility: if older launch files set F_att_max, use it as the peak.
        try:
            self.F_att_peak = float(p("F_att_max", self.F_att_peak).value)
        except Exception:
            pass

        # Visualization: arrow length scale (meters per unit of force)
        # Example: F=35 and vis_scale=0.01 -> arrow length ~0.35m
        self.vis_scale = 0.01

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

        # Selected target id topic (explicit target selection)
        self.selected_target_topic = "/selected_target"
        self.target_id = None  # type: Optional[str]
        self.target_sub = self.create_subscription(
            String,
            self.selected_target_topic,
            self._on_selected_target,
            force_qos,
        )

        # Publisher for RViz visualization markers
        self.marker_pub = self.create_publisher(Marker, "virtual_force_marker", force_qos)

        # Separate markers for debugging (att/rep/total)
        self.marker_att_pub = self.create_publisher(Marker, "virtual_force_marker_att", force_qos)
        self.marker_rep_pub = self.create_publisher(Marker, "virtual_force_marker_rep", force_qos)
        self.marker_total_pub = self.create_publisher(Marker, "virtual_force_marker_total", force_qos)

        # Load obstacle configuration
        success = self._load_obstacle_config()
        if not success:
            self.get_logger().error("Failed to load obstacle config. Node will still run but F=0.")
            # Allow the node to continue running; it will simply output zero force

        # Timer: 50 Hz (20 ms)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self._timer_callback)

        # Throttled logging
        self._last_status_log_time = self.get_clock().now()

        self.get_logger().info(
            "VirtualForceNode started. Listening to TF and publishing /virtual_force (total). "
            "Selected target can be set via /selected_target (std_msgs/String)."
        )

    def _on_selected_target(self, msg: String) -> None:
        """Callback for explicit target selection."""
        target = (msg.data or "").strip()
        if not target:
            self.get_logger().warn("Received empty target id on /selected_target; ignoring")
            return

        if not hasattr(self, "objects") or not self.objects:
            self.get_logger().warn(f"Received target '{target}' but no objects loaded yet")
            self.target_id = target
            return

        if target not in self.objects:
            self.get_logger().warn(
                f"Unknown target id '{target}'. Known ids: {sorted(self.objects.keys())}"
            )
            return

        if target != self.target_id:
            self.target_id = target
            self.get_logger().info(f"Selected target updated: {self.target_id}")

    # ----------------------------------------------------------------------
    # Configuration loading
    # ----------------------------------------------------------------------
    def _load_obstacle_config(self) -> bool:
        """Load obstacle definitions from obstacles.yaml.

        Current milestone goal (1.1): load ALL obstacles (e.g. box1/box2/box3) and compute
        a combined repulsive force.

        Note:
        - We keep a simple assumption here: obstacle boxes are axis-aligned in base_link.
        - We intentionally ignore targets in this milestone.
        """
        try:
            pkg_share = get_package_share_directory("ur3e_haptic_scene")
            yaml_path = os.path.join(pkg_share, "config", "obstacles.yaml")
            self.get_logger().info(f"Loading obstacle config from: {yaml_path}")

            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)

            if not data or "obstacles" not in data:
                self.get_logger().error("YAML missing 'obstacles' root key.")
                return False

            obstacles_cfg = data["obstacles"]

            loaded = []
            # Unified object model: all entries are homogeneous boxes.
            # APF role assignment is dynamic: selected target_id -> attractive; others -> repulsive.
            self.objects = {}
            self.index_to_id = {}

            for obs_id, cfg in obstacles_cfg.items():
                if not isinstance(cfg, dict):
                    continue

                size = cfg.get("size", None)
                pose_cfg = cfg.get("pose", {})
                center = pose_cfg.get("xyz", None)
                frame = cfg.get("frame", self.base_frame)
                index = cfg.get("index", None)

                if size is None or center is None:
                    self.get_logger().warn(f"Obstacle '{obs_id}' missing 'size' or 'pose.xyz'; skipping")
                    continue
                if len(size) != 3 or len(center) != 3:
                    self.get_logger().warn(f"Obstacle '{obs_id}' invalid 'size'/'xyz' length; skipping")
                    continue

                # Current implementation expects obs frame == base_frame; no transform applied
                if frame != self.base_frame:
                    self.get_logger().warn(
                        f"Obstacle '{obs_id}' frame '{frame}' != base_frame '{self.base_frame}'. "
                        "Current implementation assumes they match; add TF conversion if needed."
                    )

                obj_id = str(obs_id)
                self.objects[obj_id] = {
                    "id": obj_id,
                    "frame": str(frame),
                    "center": (float(center[0]), float(center[1]), float(center[2])),
                    "size": (float(size[0]), float(size[1]), float(size[2])),
                    "index": int(index) if index is not None else None,
                }

                if index is not None:
                    idx = int(index)
                    if idx in self.index_to_id:
                        self.get_logger().error(
                            f"Duplicate object index detected in YAML: index={idx} used by '{obj_id}' and '{self.index_to_id[idx]}'"
                        )
                        return False
                    self.index_to_id[idx] = obj_id

                loaded.append(obj_id)

            if not self.objects:
                self.get_logger().error("No objects loaded from obstacles.yaml.")
                return False

            self.get_logger().info(f"Loaded {len(self.objects)} object(s): {loaded}")
            if self.index_to_id:
                self.get_logger().info(f"Object indices: {dict(sorted(self.index_to_id.items()))}")

            # Default target: prefer 'target1' if present, otherwise smallest index, otherwise None.
            if self.target_id is None:
                if "target1" in self.objects:
                    self.target_id = "target1"
                elif self.index_to_id:
                    self.target_id = self.index_to_id[min(self.index_to_id.keys())]
                else:
                    self.target_id = None
            if self.target_id is not None:
                self.get_logger().info(f"Initial target_id = {self.target_id}")
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
            # Keep markers consistent: publish zero-length arrows in a safe pose.
            self._publish_force_marker(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._publish_force_marker_att(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._publish_force_marker_rep(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._publish_force_marker_total(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        px = transform.transform.translation.x
        py = transform.transform.translation.y
        pz = transform.transform.translation.z

        # APF forces
        Fx_rep = Fy_rep = Fz_rep = 0.0
        Fx_att = Fy_att = Fz_att = 0.0
        Fx = Fy = Fz = 0.0
        # Min distance bookkeeping:
        # - min_d_rep/min_obs_id_rep: nearest *repulsive* obstacle (excludes current target)
        # - min_d_all/min_obj_id_all: nearest object overall (includes current target)
        min_d_rep = None
        min_obs_id_rep = None
        min_d_all = None
        min_obj_id_all = None

        if not hasattr(self, "objects") or not self.objects:
            self._publish_force(0.0, 0.0, 0.0)
            self._publish_force_marker(px, py, pz, 0.0, 0.0, 0.0)
            return

        # ----------------------------
        # First pass: compute distances to ALL objects (for correct nearest reporting)
        # ----------------------------
        distances = {}  # obj_id -> (d, nearest_point)
        for obj_id, obj in self.objects.items():
            d, nearest_point = self._point_to_box_distance(
                px, py, pz,
                list(obj["center"]),
                list(obj["size"]),
            )
            if d is None or nearest_point is None:
                continue
            distances[obj_id] = (d, nearest_point)
            if min_d_all is None or d < min_d_all:
                min_d_all = d
                min_obj_id_all = obj_id

        # ----------------------------
        # Second pass: Repulsive force from all objects except current target
        # ----------------------------
        close_repulsive = []  # list[(obj_id, d)] for any repulsive obstacle within d_safe
        for obj_id, (d, nearest_point) in distances.items():
            if self.target_id is not None and obj_id == self.target_id:
                continue

            if min_d_rep is None or d < min_d_rep:
                min_d_rep = d
                min_obs_id_rep = obj_id

            if d < float(self.d_safe):
                close_repulsive.append((obj_id, d))

            s = self._scaling_from_distance(d)
            F_mag = self.F_max * s

            # NOTE: ignore inside-box direction handling for now (milestone 1.2 is deferred)
            if d <= 0.0 or F_mag <= 0.0:
                continue

            vx = px - nearest_point[0]
            vy = py - nearest_point[1]
            vz = pz - nearest_point[2]
            norm_v = math.sqrt(vx * vx + vy * vy + vz * vz)
            if norm_v < 1e-6:
                continue

            ux = vx / norm_v
            uy = vy / norm_v
            uz = vz / norm_v

            Fx_rep += F_mag * ux
            Fy_rep += F_mag * uy
            Fz_rep += F_mag * uz

        # ----------------------------
        # Attractive: towards selected target (if any)
        # ----------------------------
        if self.target_id is not None and self.target_id in self.objects:
            # IMPORTANT: use EE-to-target *surface* distance (point-to-AABB distance) as the
            # attraction distance metric. This avoids large targets delaying attraction until the
            # EE is unrealistically close to the *center* of the object.
            t = self.objects[self.target_id]
            tx, ty, tz = t["center"]

            # Attraction distance uses box surface distance (same geometry as repulsion):
            # d_target == 0 means the EE is on/inside the target AABB.
            d_target = None
            if self.target_id in distances:
                d_target = float(distances[self.target_id][0])
            else:
                d_tmp, _np_tmp = self._point_to_box_distance(
                    px, py, pz,
                    list(t["center"]),
                    list(t["size"]),
                )
                if d_tmp is not None:
                    d_target = float(d_tmp)

            # Direction: keep pointing to the target center (stable even when d_target ~ 0).
            ex = tx - px
            ey = ty - py
            ez = tz - pz
            e_center_norm = math.sqrt(ex * ex + ey * ey + ez * ez)
            if e_center_norm >= 1e-6 and d_target is not None:
                ux = ex / e_center_norm
                uy = ey / e_center_norm
                uz = ez / e_center_norm

                # Piecewise attractive magnitude ("hat" profile) based on surface distance:
                # - d_target >= d_att_start: 0
                # - d_att_peak <= d_target < d_att_start: linear increase (approaching target)
                # - d_att_stop <= d_target < d_att_peak: linear decrease to 0 (very close)
                # - d_target < d_att_stop: 0
                d_start = float(self.d_att_start)
                d_peak = float(self.d_att_peak)
                d_stop = float(self.d_att_stop)
                F_peak = float(self.F_att_peak)

                # Safety: enforce a sensible ordering to avoid division by zero.
                # If misconfigured, fall back to “no attraction”.
                if not (d_start > d_peak > d_stop >= 0.0):
                    F_mag = 0.0
                else:
                    if d_target >= d_start:
                        F_mag = 0.0
                    elif d_target >= d_peak:
                        # ramp up as distance shrinks: 0 at d_start -> F_peak at d_peak
                        F_mag = F_peak * (d_start - d_target) / (d_start - d_peak)
                    elif d_target >= d_stop:
                        # ramp down as distance shrinks further: F_peak at d_peak -> 0 at d_stop
                        F_mag = F_peak * (d_target - d_stop) / (d_peak - d_stop)
                    else:
                        F_mag = 0.0

                if F_mag > 0.0:
                    Fx_att = F_mag * ux
                    Fy_att = F_mag * uy
                    Fz_att = F_mag * uz

        # Total
        Fx = Fx_rep + Fx_att
        Fy = Fy_rep + Fy_att
        Fz = Fz_rep + Fz_att

        # Publish force vector for rqt_plot
        self._publish_force(Fx, Fy, Fz)

        # Publish RViz arrow markers
        self._publish_force_marker(px, py, pz, Fx, Fy, Fz)
        self._publish_force_marker_att(px, py, pz, Fx_att, Fy_att, Fz_att)
        self._publish_force_marker_rep(px, py, pz, Fx_rep, Fy_rep, Fz_rep)
        self._publish_force_marker_total(px, py, pz, Fx, Fy, Fz)

        # Throttled status logging (1 Hz)
        now = self.get_clock().now()
        if (now - self._last_status_log_time).nanoseconds > int(1e9):
            mag = math.sqrt(Fx * Fx + Fy * Fy + Fz * Fz)
            # Distance to target object (EE -> target box surface) for debugging.
            d_target = None  # type: Optional[float]
            if self.target_id is not None and self.target_id in distances:
                d_target = float(distances[self.target_id][0])

            # Any repulsive obstacle within d_safe (15cm by default)
            close_repulsive_sorted = sorted(close_repulsive, key=lambda x: x[1])
            close_repulsive_s = (
                "none"
                if not close_repulsive_sorted
                else ", ".join([f"{oid}:{dist:.3f}m" for oid, dist in close_repulsive_sorted])
            )

            if min_d_all is None:
                self.get_logger().info(f"|F|={mag:.2f}, no valid object distance")
            else:
                d_target_s = "none" if d_target is None else f"{d_target:.3f}m"
                nearest_all_s = f"{min_obj_id_all} (d={min_d_all:.3f}m)"
                self.get_logger().info(
                    f"target={self.target_id}, d_target={d_target_s}, "
                    f"nearest_all={nearest_all_s}, "
                    f"close_rep(<{float(self.d_safe):.2f}m)=[{close_repulsive_s}], "
                    f"F_rep=({Fx_rep:.2f},{Fy_rep:.2f},{Fz_rep:.2f}), "
                    f"F_att=({Fx_att:.2f},{Fy_att:.2f},{Fz_att:.2f}), "
                    f"F=({Fx:.2f},{Fy:.2f},{Fz:.2f}), |F|={mag:.2f}"
                )
            self._last_status_log_time = now

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

    def _publish_force_marker(
        self,
        px: float,
        py: float,
        pz: float,
        Fx: float,
        Fy: float,
        Fz: float,
    ) -> None:
        """Publish a single arrow marker to visualize the virtual force in RViz."""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "virtual_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow defined by two points (start at EEF, end at EEF + scaled force)
        start = Point()
        start.x = float(px)
        start.y = float(py)
        start.z = float(pz)

        end = Point()
        end.x = float(px + self.vis_scale * Fx)
        end.y = float(py + self.vis_scale * Fy)
        end.z = float(pz + self.vis_scale * Fz)

        marker.points = [start, end]

        # Geometry scale: shaft diameter and head diameter/length
        marker.scale.x = 0.01  # shaft diameter
        marker.scale.y = 0.02  # head diameter
        marker.scale.z = 0.04  # head length

        # Color (cyan) - backward compatible marker
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Make it persist until overwritten; we re-publish every cycle.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

    def _publish_force_marker_att(self, px: float, py: float, pz: float, Fx: float, Fy: float, Fz: float) -> None:
        marker = self._make_marker(px, py, pz, Fx, Fy, Fz, ns="virtual_force_att", marker_id=1, color=(0.0, 1.0, 0.0, 1.0))
        self.marker_att_pub.publish(marker)

    def _publish_force_marker_rep(self, px: float, py: float, pz: float, Fx: float, Fy: float, Fz: float) -> None:
        marker = self._make_marker(px, py, pz, Fx, Fy, Fz, ns="virtual_force_rep", marker_id=2, color=(1.0, 0.0, 0.0, 1.0))
        self.marker_rep_pub.publish(marker)

    def _publish_force_marker_total(self, px: float, py: float, pz: float, Fx: float, Fy: float, Fz: float) -> None:
        marker = self._make_marker(px, py, pz, Fx, Fy, Fz, ns="virtual_force_total", marker_id=3, color=(0.0, 0.3, 1.0, 1.0))
        self.marker_total_pub.publish(marker)

    def _make_marker(
        self,
        px: float,
        py: float,
        pz: float,
        Fx: float,
        Fy: float,
        Fz: float,
        *,
        ns: str,
        marker_id: int,
        color: tuple,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = float(px)
        start.y = float(py)
        start.z = float(pz)

        end = Point()
        end.x = float(px + self.vis_scale * Fx)
        end.y = float(py + self.vis_scale * Fy)
        end.z = float(pz + self.vis_scale * Fz)
        marker.points = [start, end]

        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.04

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker


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
