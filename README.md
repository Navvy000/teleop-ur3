# UR3e APF Targeted Teleop (ROS 2 Jazzy)

This workspace is for joystick teleoperation of a **UR3e** arm in **Gazebo simulation**, with an additional **APF (Artificial Potential Field) virtual-force guidance** layer based on scene obstacles.

The repository contains 3 ROS packages:

- `ur3e_teleop_bringup`: integrated bringup (simulation + MoveIt + Servo + joystick nodes + optional staging/initial pose) and Servo configs.
- `ur3e_teleop_joy`: joystick teleop nodes (publishes `/ur3e_servo/joint_jog` or `/ur3e_servo/cartesian_twist` to MoveIt Servo).
- `ur3e_haptic_scene`: injects `obstacles.yaml` into the MoveIt PlanningScene, target selection, and publishes virtual force `/virtual_force`.


## Dependencies

- Ubuntu 24.04 + **ROS 2 Jazzy**
- MoveIt 2
- UR Gazebo simulation dependencies (e.g. `ur_simulation_gz`; this repo does not vendor those sources)


## Build (colcon)

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rookie/ur3e_teleop_ws
colcon build --merge-install --symlink-install
```

### About install layout (important)

Don’t mix different install layouts in the same workspace:

- If you use `--merge-install`, keep using it going forward.
- If you’re told to “delete install and rebuild”, it’s usually because `install/` was created with a different layout previously.


## Environment setup

In every new terminal, source Jazzy first, then source this workspace overlay (order matters):

```bash
source /opt/ros/jazzy/setup.bash
source /home/rookie/ur3e_teleop_ws/install/setup.bash
```

If you use a custom helper command (e.g. `ros2on`), make sure it’s equivalent to the two lines above and in the same order.


## Quick start (recommended entry)

Integrated bringup (simulation + MoveIt/Servo + teleop + APF + staging):

```bash
ros2 launch ur3e_teleop_bringup ur3e_world_moveit_teleop_apf.launch.py
```

This launch is composed of two layers:

1) **Bringup layer**: `ur3e_world_moveit_teleop.launch.py`
  - Starts Gazebo + MoveIt + Servo + `joy_node` + `cartesian_teleop`
  - Switches Servo to Twist mode via `/ur3e_servo/switch_command_type` (`command_type: 1`)
  - Optionally runs `ur3e_haptic_scene/add_obstacles` to inject `obstacles.yaml` into the PlanningScene

2) **APF + staging layer**: `ur3e_world_moveit_teleop_apf.launch.py`
  - Optionally starts `staging_pose_executor`: performs a one-shot joint initialization via MoveIt (plan+execute) before teleop
  - Starts the virtual-force node (on by default) and the target selector (off by default)


## Modes

### 1) Cartesian Teleop (default in integrated bringup)

Launch: `ur3e_teleop_bringup/launch/ur3e_cartesian_teleop.launch.py` (or reused by the integrated entry).

Data flow:

`/joy` → `cartesian_teleop` → `/ur3e_servo/cartesian_twist` → `moveit_servo/servo_node` → controller

In `ur3e_world_moveit_teleop.launch.py`, the default `cartesian_teleop` parameters are set in the launch file:

- `enable_button: 5` (RB deadman switch)
- `deadzone: 0.15`
- `linear_scale: 0.1` (m/s)
- `angular_scale: 0.8` (rad/s)
- `command_frame: tool0`

### 2) JointJog Teleop

Launch: `ur3e_teleop_bringup/launch/ur3e_joint_teleop.launch.py`

Data flow:

`/joy` → `joint_teleop` → `/ur3e_servo/joint_jog` → `moveit_servo/servo_node` → controller

Default joint axis mapping (configured in launch): `joint_axes: [1, 0, 4, 3, -1, -1]`.

The `ur3e_teleop_joy` package also provides a more robust variant: `ur3e_teleop_joy/joint_teleop_improved.py` (includes speed clamping and stricter parameter checks).


## Joystick mapping notes (Logitech F310 reference)

This repo’s defaults assume a typical Logitech F310-style mapping, and both teleop modes use a deadman switch:

- Hold **RB** to enable motion (`enable_button: 5`).
- Release **RB** to stop (teleop node publishes a single “zero command” to brake).

### JointJog axis mapping

The default mapping in the joint teleop launch is:

- `joint_axes: [1, 0, 4, 3, -1, -1]`

Meaning:

- Joint 0 (shoulder_pan)  ← axis 1
- Joint 1 (shoulder_lift) ← axis 0
- Joint 2 (elbow)         ← axis 4
- Joint 3 (wrist_1)       ← axis 3
- Joint 4 (wrist_2)       ← disabled
- Joint 5 (wrist_3)       ← disabled

If your gamepad mapping is different, the fastest way to confirm is:

```bash
ros2 topic echo /joy
```

Move/press one control at a time and watch which `axes[]` / `buttons[]` entries change.


## Staging pose (initial pose)

Goal: before you start teleop, move the arm away from “upright/straight arm/wrist aligned” problematic initial configurations into a more stable joint posture, so Servo is less likely to hit singularities or limits right away.

Implementation: `ur3e_teleop_bringup/scripts/staging_pose_executor.py`

- Acts as an Action client to `move_group`’s `moveit_msgs/action/MoveGroup`
- Default action name: `/move_action`
- Sends a joint-constraint goal, optionally plan-only or plan+execute

Config file: `ur3e_teleop_bringup/config/staging_pose.yaml`

The YAML must follow ROS 2 parameter structure (indentation is strict):

```yaml
staging_pose_executor:
  ros__parameters:
    joint_names: [...]
    joint_positions: [...]
```

If fields like `joint_positions` are indented outside `ros__parameters`, the node won’t read your parameters; it will look like “the file was edited but the pose didn’t change”.

In `ur3e_world_moveit_teleop_apf.launch.py`, you can control it with:

- `run_staging` (default `true`)
- `staging_delay` (default `10.0` seconds)
- `staging_yaml` (defaults to the package’s `config/staging_pose.yaml`)


## APF / virtual-force guidance

### Scene objects

File: `ur3e_haptic_scene/config/obstacles.yaml`

- `add_obstacles.py` publishes all boxes to `/planning_scene` (PlanningScene diff)
- `target_selector.py` uses the `index` field to build a “number → object_id” mapping

### Target selection

Topic: `/selected_target` (`std_msgs/String`)

- `target_selector.py` publishes when you type a number on the keyboard (you can also publish this topic yourself)

### Virtual force

Node: `ur3e_haptic_scene/scripts/virtual_force_node.py`

- Publishes: `/virtual_force` (`geometry_msgs/Vector3`)
- Also publishes RViz visualization markers: `virtual_force_marker*`
- TF: by default uses `base_link` (base) and `tool0` (end effector) to compute distances


## Nodes & topics (as implemented)

| Module | Node / executable | Main subscriptions | Main publications |
|------|-------------|----------|----------|
| Joystick input | `joy/joy_node` | — | `/joy` |
| Teleop (Cartesian) | `ur3e_teleop_joy/cartesian_teleop` | `/joy` | `/ur3e_servo/cartesian_twist` |
| Teleop (joint) | `ur3e_teleop_joy/joint_teleop` | `/joy` | `/ur3e_servo/joint_jog` |
| Servo | `moveit_servo/servo_node` | `/ur3e_servo/cartesian_twist` or `/ur3e_servo/joint_jog` | Controller trajectory topic (defined by Servo config) |
| Staging | `ur3e_teleop_bringup/staging_pose_executor.py` | `/joint_states` | `/move_action` (action client) |
| Obstacle injection | `ur3e_haptic_scene/add_obstacles` | — | `/planning_scene` |
| Target selection | `ur3e_haptic_scene/target_selector` | stdin | `/selected_target` |
| Virtual force | `ur3e_haptic_scene/virtual_force` | TF, `/selected_target` | `/virtual_force`, `virtual_force_marker*` |


## Verify data flow (quick debugging)

When you press/hold RB and move the stick, you should see the corresponding command topic updating.

Cartesian mode:

```bash
ros2 topic echo /ur3e_servo/cartesian_twist
```

Joint mode:

```bash
ros2 topic echo /ur3e_servo/joint_jog
```

If those topics are active but the robot doesn’t move, the issue is usually downstream (Servo configuration, controller readiness, collisions/singularity stops, etc.).


## FAQ / troubleshooting

### 1) `Package 'ur3e_teleop_bringup' not found`
This is almost always because the overlay wasn’t sourced correctly. Check the order:

```bash
source /opt/ros/jazzy/setup.bash
source /home/rookie/ur3e_teleop_ws/install/setup.bash
```

### 2) colcon install layout conflict (merged / isolated)
This means `install/` was created with a different layout. Either keep using one layout consistently, or:

```bash
rm -rf build install log
```

Then rebuild (slower, but cleanest).

### 3) staging pose “file edited but pose is wrong / inconsistent”
First check whether `staging_pose.yaml` is strictly indented under `ros__parameters`.

Also, `ur3e_world_moveit_teleop_apf.launch.py` allows overriding the parameter file path via `staging_yaml`; if you pass a custom path, make sure the launched system is using the YAML you intended.

### 4) staging planning fails (MoveIt error_code != 1)
A common cause is that the target configuration is in collision in the PlanningScene (e.g. with boxes injected from `obstacles.yaml`). Suggestions:

- Temporarily set `publish_obstacles:=false` to verify the staging pipeline works
- Or change the staging target joints to a more conservative pose and try again

### 5) joystick device not found (`/dev/input/js0`)
If `joy_node` can’t open the device, your joystick may be on a different path.

Quick check:

```bash
ls -la /dev/input/js*
```

### 6) no motion even though the system is running
Common checklist:

1. Confirm the deadman switch works (RB / `enable_button: 5`) by watching `/joy`.
2. Confirm teleop is actually publishing (`/ur3e_servo/cartesian_twist` or `/ur3e_servo/joint_jog`).
3. If teleop publishes but there’s still no motion, open the Servo / controller terminal output and look for collision / singularity / timeout messages.


## Directory structure

```text
src/
  ur3e_teleop_bringup/
    launch/
      ur3e_world_moveit_teleop.launch.py
      ur3e_world_moveit_teleop_apf.launch.py
      ur3e_cartesian_teleop.launch.py
      ur3e_joint_teleop.launch.py
    config/
      ur3e_servo_cartesian.yaml
      ur3e_servo_joint.yaml
      ur3e_servo_kinematics.yaml
      staging_pose.yaml
    scripts/
      staging_pose_executor.py
  ur3e_teleop_joy/
    ur3e_teleop_joy/
      joint_teleop_improved.py
  ur3e_haptic_scene/
    ur3e_haptic_scene/scripts/
      add_obstacles.py
      target_selector.py
      virtual_force_node.py
    config/
      obstacles.yaml
```
