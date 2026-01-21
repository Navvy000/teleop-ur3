#!/usr/bin/env python3
"""target_selector.py

Keyboard-based explicit target selection for the APF/haptic guidance stage.

Design goals (engineering):
- Deterministic mapping from numeric input -> object id, suitable for reproducible experiments.
- Non-blocking ROS 2 node: stdin reading happens in a background thread.
- Minimal dependencies (stdlib + rclpy + PyYAML).

Runtime behaviour:
- Loads object definitions from `ur3e_haptic_scene/config/obstacles.yaml`.
- Builds a mapping: `index (int) -> object_id (str)`.
- When user types a number (e.g. 2) and presses Enter:
    publishes `std_msgs/String` on `/selected_target` with `data=object_id`.

Notes:
- We intentionally do not validate whether the chosen object is a "target" vs "obstacle".
  At this stage, all objects are homogeneous boxes; semantic role is assigned by selection.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import threading
from dataclasses import dataclass
from typing import Dict, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String


@dataclass(frozen=True)
class SceneObject:
    object_id: str
    index: int


class TargetSelector(Node):
    def __init__(self) -> None:
        super().__init__("target_selector")

        p = self.declare_parameter
        self.yaml_path = p("yaml_path", "").value
        self.output_topic = p("selected_target_topic", "/selected_target").value
        self.publish_initial_target = p("publish_initial_target", False).value
        self.initial_target_index = int(p("initial_target_index", 1).value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(String, self.output_topic, qos)

        self.index_to_id: Dict[int, str] = self._load_index_map()
        self._stop_event = threading.Event()

        self._print_menu()

        if self.publish_initial_target:
            self._publish_by_index(self.initial_target_index)

        # Start background stdin reader thread.
        # (daemon=False) so interpreter shutdown is not interrupted mid-read.
        self._thread = threading.Thread(target=self._stdin_loop, daemon=False)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            if hasattr(self, "_thread") and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()

    def _print_menu(self) -> None:
        pairs = ", ".join(
            [f"{idx}->{self.index_to_id[idx]}" for idx in sorted(self.index_to_id.keys())]
        )
        self.get_logger().info(
            "TargetSelector ready. Select target by typing the object index. "
            f"Publishing to {self.output_topic}. Available: {pairs}. "
            "(You can also type 'q' to quit.)"
        )

    # -----------------------
    # YAML loading
    # -----------------------
    def _resolve_yaml_path(self) -> str:
        if self.yaml_path:
            return self.yaml_path

        pkg_share = get_package_share_directory("ur3e_haptic_scene")
        return os.path.join(pkg_share, "config", "obstacles.yaml")

    def _load_index_map(self) -> Dict[int, str]:
        yaml_path = self._resolve_yaml_path()
        self.get_logger().info(f"Loading scene objects from: {yaml_path}")

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        if not data or "obstacles" not in data or not isinstance(data["obstacles"], dict):
            raise RuntimeError("YAML must contain a dict root key 'obstacles:'")

        objects_cfg: dict = data["obstacles"]

        seen_indices = set()
        index_to_id: Dict[int, str] = {}

        for object_id, cfg in objects_cfg.items():
            if not isinstance(cfg, dict):
                continue

            if "index" not in cfg:
                raise RuntimeError(
                    f"Object '{object_id}' is missing required field 'index'. "
                    "Please add explicit indices for deterministic selection."
                )

            idx = int(cfg["index"])
            if idx in seen_indices:
                raise RuntimeError(f"Duplicate object index detected: index={idx}")

            seen_indices.add(idx)
            index_to_id[idx] = str(object_id)

        if not index_to_id:
            raise RuntimeError("No objects with 'index' found in YAML")

        return index_to_id

    # -----------------------
    # Publishing
    # -----------------------
    def _publish_target(self, object_id: str) -> None:
        msg = String()
        msg.data = object_id
        self.pub.publish(msg)
        self.get_logger().info(f"Selected target: id='{object_id}'")

    def _publish_by_index(self, idx: int) -> None:
        obj_id = self.index_to_id.get(idx)
        if obj_id is None:
            self.get_logger().warn(
                f"Invalid selection index={idx}. Valid indices: {sorted(self.index_to_id.keys())}"
            )
            return
        self._publish_target(obj_id)

    # -----------------------
    # Stdin reader thread
    # -----------------------
    def _stdin_loop(self) -> None:
        """Stdin reader loop running in a background thread.

        If stdin is a TTY, we also accept single-key digits without requiring Enter.
        This makes it work better inside a `ros2 launch ...` terminal.
        """

        if not sys.stdin or not sys.stdin.isatty():
            self.get_logger().warn(
                "stdin is not a TTY (non-interactive). Keyboard target selection disabled. "
                "You can still publish /selected_target manually."
            )
            return

        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)

        def _restore() -> None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
            except Exception:
                pass

        try:
            # cbreak-like: read chars immediately without waiting for newline
            new_attr = termios.tcgetattr(fd)
            new_attr[3] = new_attr[3] & ~(termios.ICANON | termios.ECHO)
            termios.tcsetattr(fd, termios.TCSADRAIN, new_attr)

            while not self._stop_event.is_set() and rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue

                ch = sys.stdin.read(1)
                if not ch:
                    continue

                if ch in {"q", "Q"}:
                    self.get_logger().info("Received quit command on stdin.")
                    # Let the main thread handle shutdown.
                    self._stop_event.set()
                    return

                if ch.isdigit():
                    self._publish_by_index(int(ch))
                    continue

                # Fallback: allow whitespace and ignore everything else.
        except Exception as e:
            self.get_logger().warn(f"stdin handling failed; falling back to line input: {e}")
        finally:
            _restore()

        # Line-based fallback (requires Enter)
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception as e:
                self.get_logger().error(f"stdin read exception: {e}")
                return

            if not line:
                self.get_logger().warn("EOF on stdin; target selection disabled.")
                return

            if self._stop_event.is_set() or not rclpy.ok():
                return

            line = line.strip()
            if not line:
                continue

            if line.lower() in {"q", "quit", "exit"}:
                self.get_logger().info("Received quit command on stdin.")
                self._stop_event.set()
                return

            try:
                idx = int(line)
            except ValueError:
                self.get_logger().warn(
                    f"Invalid input '{line}'. Type an integer index (e.g. 1) or 'q' to quit."
                )
                continue

            self._publish_by_index(idx)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TargetSelector] = None
    try:
        node = TargetSelector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
