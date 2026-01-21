#!/usr/bin/env python3
"""staging_pose_executor.py

One-shot "pre-teleop" staging pose executor for UR3e.

Motivation
----------
When the system starts, the robot may be in a near-singular or awkward initial
configuration (e.g. upright/straight). Sending Cartesian Twist commands to MoveIt
Servo from this configuration can immediately hit singularity/joint-limit logic,
causing Servo to halt.

This node performs an automatic initialization step:
  1) Wait until MoveIt `move_group` is available.
  2) Send a joint-space goal to MoveGroup (Plan + Execute).
  3) Exit.

Design choices
--------------
- Uses the stable MoveIt action interface `moveit_msgs/action/MoveGroup` because
  `moveit_py` / `moveit_commander` may not be available in all Jazzy installs.
- Joint goal is passed as ROS parameters for reproducibility.

Parameters
----------
- move_group_action_name (string): default '/move_action'
- group_name (string): default 'ur_manipulator'
- joint_names (string[]): UR joint names
- joint_positions (double[]): target positions in radians
- goal_tolerance (double): default 1e-3
- allowed_planning_time (double): default 5.0
- max_velocity_scaling (double): default 0.5
- max_acceleration_scaling (double): default 0.5
- execute (bool): default true. If false, only plans (still sends request).
- timeout_sec (double): default 30.0

Notes
-----
This is intended to run once at startup. It can be launched via a TimerAction
from bringup launch files.
"""

from __future__ import annotations

from typing import List
import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState


class StagingPoseExecutor(Node):
    def __init__(self) -> None:
        super().__init__("staging_pose_executor")

        p = self.declare_parameter
        self.move_group_action_name = str(p("move_group_action_name", "/move_action").value)
        self.group_name = str(p("group_name", "ur_manipulator").value)

        self.joint_names: List[str] = [str(x) for x in p(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        ).value]

        self.joint_positions: List[float] = [float(x) for x in p(
            "joint_positions",
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        ).value]

        self.goal_tolerance = float(p("goal_tolerance", 1e-3).value)
        self.allowed_planning_time = float(p("allowed_planning_time", 5.0).value)
        self.max_velocity_scaling = float(p("max_velocity_scaling", 0.5).value)
        self.max_acceleration_scaling = float(p("max_acceleration_scaling", 0.5).value)
        self.execute = bool(p("execute", True).value)
        self.timeout_sec = float(p("timeout_sec", 30.0).value)
        self.joint_states_timeout_sec = float(p("joint_states_timeout_sec", 10.0).value)

        if len(self.joint_names) != len(self.joint_positions):
            raise RuntimeError(
                f"joint_names size ({len(self.joint_names)}) != joint_positions size ({len(self.joint_positions)})"
            )

        self._client = ActionClient(self, MoveGroup, self.move_group_action_name)

        self._got_joint_state = False
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        # We just need one valid update so MoveIt has state.
        if msg.name:
            self._got_joint_state = True

    def build_goal(self) -> MoveGroup.Goal:
        # Joint constraints
        joint_constraints: List[JointConstraint] = []
        for name, pos in zip(self.joint_names, self.joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.goal_tolerance
            jc.tolerance_below = self.goal_tolerance
            jc.weight = 1.0
            joint_constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.goal_constraints = [goal_constraints]

        # Planning options
        req.allowed_planning_time = self.allowed_planning_time
        req.max_velocity_scaling_factor = self.max_velocity_scaling
        req.max_acceleration_scaling_factor = self.max_acceleration_scaling

        # Construct MoveGroup action goal
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = (not self.execute)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        # Let the execution manager use reasonable defaults.
        goal.planning_options.planning_scene_diff.is_diff = True

        return goal

    def run(self) -> int:
        self.get_logger().info(
            f"Waiting for first /joint_states (timeout {self.joint_states_timeout_sec}s) ..."
        )
        start = self.get_clock().now()
        # Wait using rclpy spinning in main() so subscriptions can be processed.
        while rclpy.ok() and not self._got_joint_state:
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > self.joint_states_timeout_sec:
                self.get_logger().warn(
                    "Did not receive /joint_states in time; proceeding anyway (MoveIt may still succeed)."
                )
                break
            # Yield to executor
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f"StagingPoseExecutor: waiting for MoveGroup action '{self.move_group_action_name}' ..."
        )

        if not self._client.wait_for_server(timeout_sec=self.timeout_sec):
            self.get_logger().error(
                f"MoveGroup action server '{self.move_group_action_name}' not available after {self.timeout_sec}s"
            )
            return 2

        goal = self.build_goal()
        self.get_logger().info(
            "Sending staging joint goal (rad): "
            + ", ".join([f"{n}={p:.3f}" for n, p in zip(self.joint_names, self.joint_positions)])
        )

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=self.timeout_sec)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Timed out waiting for MoveGroup goal handle")
            return 3

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return 3

        self.get_logger().info("MoveGroup goal accepted; waiting for result ...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        result = result_future.result()

        if result is None:
            self.get_logger().error("Timed out waiting for MoveGroup result")
            return 4

        # MoveIt error code: 1 means SUCCESS.
        error_code = int(result.result.error_code.val)
        if error_code == 1:
            self.get_logger().info("Staging pose execution SUCCESS")
            return 0

        self.get_logger().error(f"Staging pose FAILED: moveit error_code={error_code}")
        return 4


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StagingPoseExecutor()
    try:
        exit_code = int(node.run())
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
        exit_code = 130
    except Exception as e:
        node.get_logger().error(f"Exception: {e}\n{traceback.format_exc()}")
        exit_code = 1
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
