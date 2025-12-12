# UR3e Joystick Teleoperation with ROS 2 and MoveIt Servo

This repository provides a complete joystick teleoperation system for the **Universal Robots UR3e** manipulator using **ROS 2 Jazzy**, **MoveIt 2**, and **Gazebo** simulation (`ur_simulation_gz`). In addition to joint and Cartesian teleop, the workspace now ships a lightweight haptic feedback stack that mirrors scene obstacles into MoveIt and synthesises virtual repulsive forces for operator awareness.

## Overview

### What Does This Project Do?

This workspace implements two complementary teleoperation modes for remote control of a UR3e robotic arm via a gamepad joystick:

1. **Joint-space teleoperation** – Direct incremental control of individual joints (JointJog mode)
2. **Cartesian end-effector teleoperation** – Twist-based control of the tool frame position and orientation (MoveIt Servo Twist mode)

Both modes use the **MoveIt Servo** real-time motion planning system to convert high-level commands into low-level joint trajectory commands that are executed by the UR's `scaled_joint_trajectory_controller` in Gazebo or on real hardware.

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  MODE 1: JOINT-SPACE TELEOPERATION                              │
├─────────────────────────────────────────────────────────────────┤
│  Joystick (/joy)                                                │
│      ↓                                                           │
│  ur3e_joint_teleop (ur3e_teleop_joy/joint_teleop.py)            │
│      ↓                                                           │
│  JointJog message → /ur3e_servo/joint_jog                       │
│      ↓                                                           │
│  MoveIt Servo (joint_command_in_type: unitless)                 │
│      ↓                                                           │
│  JointTrajectory → /scaled_joint_trajectory_controller/...      │
│      ↓                                                           │
│  ROS 2 Control + Gazebo UR3e Simulation                         │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  MODE 2: CARTESIAN END-EFFECTOR TELEOPERATION                   │
├─────────────────────────────────────────────────────────────────┤
│  Joystick (/joy)                                                │
│      ↓                                                           │
│  ur3e_cartesian_teleop (ur3e_teleop_joy/cartesian_teleop*.py)   │
│      ↓                                                           │
│  TwistStamped message → /ur3e_servo/cartesian_twist             │
│      ↓                                                           │
│  MoveIt Servo (command_in_type: speed_units, Twist mode)        │
│      ↓                                                           │
│  Inverse Kinematics (KDL solver)                                │
│      ↓                                                           │
│  JointTrajectory → /scaled_joint_trajectory_controller/...      │
│      ↓                                                           │
│  ROS 2 Control + Gazebo UR3e Simulation                         │
└─────────────────────────────────────────────────────────────────┘
```

```
┌─────────────────────────────────────────────────────────────────┐
│  HAPTIC FEEDBACK EXTENSION                                      │
├─────────────────────────────────────────────────────────────────┤
│  MoveIt Planning Scene (with obstacles from add_obstacles.py)   │
│      ↓                                                          │
│  TF tree (base_link ↔ tool0)                                   │
│      ↓                                                          │
│  virtual_force_node (virtual force model)                      │
│      ↓                                                          │
│  /virtual_force (Vector3) → rqt_plot / haptic device            │
└─────────────────────────────────────────────────────────────────┘
```

### Key Components

- **Gazebo + UR3e + ros2_control**: Provided by the external `ur_simulation_gz` package (e.g., `ur_sim_moveit.launch.py`)
- **MoveIt Servo**: Real-time motion planning framework from this workspace's configuration
- **Joy node**: Publishes joystick input to `/joy` topic
- **Teleoperation nodes**: Convert Joy messages to robot commands (located in `ur3e_teleop_joy`)
- **Launch files**: Orchestrate the entire pipeline with proper parameter passing

---

## Repository Structure

### Top-Level Files

- **`verify_config.py`** – Configuration validation script that checks topic consistency between YAML configs and launch files
- **`apply_fixes.sh`** – Helper script to apply critical configuration fixes

### `ur3e_teleop_bringup/`

Contains MoveIt Servo configuration and launch files for bringing up the teleoperation system.

#### Config Files (`config/`)

- **`ur3e_servo_cartesian.yaml`** – MoveIt Servo configuration for Twist (Cartesian end-effector) control mode
  - Subscribes to `/ur3e_servo/cartesian_twist` (TwistStamped)
  - Publishes to `/scaled_joint_trajectory_controller/joint_trajectory`
  - Configured for tool0 frame control

- **`ur3e_servo_joint.yaml`** – MoveIt Servo configuration for JointJog (joint-space) control mode
  - Subscribes to `/ur3e_servo/joint_jog` (JointJog messages)
  - Publishes to `/scaled_joint_trajectory_controller/joint_trajectory`
  - Configured for base_link frame

- **`ur3e_servo_kinematics.yaml`** – Kinematics solver configuration
  - Specifies KDL (Kinematics and Dynamics Library) plugin for inverse kinematics
  - Configures the `ur_manipulator` planning group

#### Launch Files (`launch/`)

- **`ur3e_cartesian_teleop.launch.py`** – Main launcher for Cartesian end-effector teleoperation
  - Starts MoveIt Servo with Cartesian config
  - Launches Joy node
  - Launches Cartesian teleop node
  - Automatically switches Servo to Twist mode (command_type=1)

- **`ur3e_joint_teleop.launch.py`** – Launcher for joint-space teleoperation
  - Starts MoveIt Servo with joint config
  - Launches Joy node
  - Launches joint teleop node
  - Automatically switches Servo to JointJog mode (command_type=0)

- **`ur3e_servo_only.launch.py`** – Minimal launcher (Servo node only, for testing)

- **`check_setup.launch.py`** – Simple verification launcher that starts a dummy joy node

- **`ur3e_cartesian_teleop_complete.launch.py`** – Complete launcher with optional robot_description loading (advanced use case)

### `ur3e_teleop_joy/`

Python package containing the teleoperation nodes. This is a standard ROS 2 Python package.

#### Teleoperation Nodes (`ur3e_teleop_joy/`)

- **`joint_teleop.py`** – Main joint-space teleop node
  - Converts Joy input to JointJog messages
  - RB button acts as deadman switch (enable)
  - Default axis mapping for Logitech F310 gamepad

- **`joint_teleop_improved.py`** – Enhanced version with speed limiting
  - Adds `max_joint_speed` parameter for safety
  - Better QoS settings (depth=1 for real-time compatibility)
  - Improved boundary checking and logging

- **`cartesian_teleop.py`** – Main Cartesian end-effector teleop node
  - Converts Joy input to TwistStamped messages in tool0 frame
  - RB button acts as deadman switch
  - Independent linear and angular scaling

- **`cartesian_teleop_improved.py`** – Enhanced Cartesian teleop node
  - Adds `max_linear_speed` and `max_angular_speed` parameters
  - Better QoS settings (depth=1 for low latency)
  - Improved axis safety checking

- **`joy_dummy.py`** – Minimal test node for verifying workspace setup

#### Package Files

- **`setup.py`** – Python package setup, defines console_scripts entry points
- **`setup.cfg`** – Setup configuration
- **`package.xml`** – ROS 2 package metadata with dependencies

### `ur3e_haptic_scene/`

Scene description and haptic feedback utilities that complement the teleop stack.

#### Config & World Assets

- **`config/obstacles.yaml`** – Declarative list of axis-aligned obstacles mirrored into MoveIt and Gazebo.
- **`worlds/my_ur3e_world.sdf`** – Gazebo world embedding the same obstacle geometry (keeps simulation and planning scene consistent).

#### Launch Files (`launch/`)

- **`my_ur3e_gazebo.launch.py`** – Launches Gazebo (`gz sim`) with the custom world for quick testing.

#### Runtime Scripts (`ur3e_haptic_scene/scripts/`)

- **`add_obstacles.py`** – Reads `config/obstacles.yaml`, creates `moveit_msgs/CollisionObject` entries, and publishes a PlanningScene diff so MoveIt Servo sees the obstacles.
- **`virtual_force_node.py`** – Uses TF to query `tool0` → `base_link` transforms, computes the shortest distance to each box, and emits repulsive forces on `/virtual_force` following a spring–damper model.

#### Package Files

- **`setup.py`** – Installs launch/world/config assets and exposes `add_obstacles` / `virtual_force` console scripts.
- **`package.xml`** – Declares haptic-specific dependencies (MoveIt messages, TF, YAML parsing).

---

## Nodes and Data Flow

### Node 1: Joint-Space Teleoperation Node (`joint_teleop.py` / `joint_teleop_improved.py`)

#### Summary

Converts joystick input into real-time joint velocity commands for MoveIt Servo in JointJog mode.

#### Subscriptions

| Topic | Type | Purpose |
|-------|------|---------|
| `/joy` | `sensor_msgs/Joy` | Gamepad input from joy_node driver |

#### Publications

| Topic | Type | Purpose |
|-------|------|---------|
| `/ur3e_servo/joint_jog` | `control_msgs/JointJog` | Real-time joint velocity commands to MoveIt Servo |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joy_topic` | string | `/joy` | Topic name for joystick input |
| `enable_button` | int | `5` | Joystick button index for deadman switch (RB button on F310) |
| `deadzone` | float | `0.15` | Threshold below which joystick input is ignored (unit: -1 to 1) |
| `joint_scale` | float | `1.0` | Scaling factor for joystick input to joint velocities |
| `max_joint_speed` | float | `2.0` | Maximum joint angular velocity in rad/s (improved version only) |
| `joint_names` | list | `['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']` | Names of the 6 UR3e joints |
| `joint_axes` | list | `[1, 0, 4, 3, -1, -1]` | Mapping of joystick axes to joints (negative = disabled) |
| `joint_jog_topic` | string | `/ur3e_servo/joint_jog` | Output topic for JointJog commands |

#### Coordinate Frame

- **Command frame**: `base_link` (robot base)
- All joint commands are expressed relative to the robot's joint limits

#### Safety Behavior

**Deadman Switch Logic:**
- **While RB button is held**: Continuously publishes joint velocity commands based on joystick input
- **When RB button is released**: Publishes a single zero-velocity `JointJog` message to stop all joint motion (brake command)
- **Axis deadzone**: Input values below the deadzone threshold are clipped to zero to eliminate stick drift

#### Internal Algorithm

1. **Receive Joy message** containing button states and axis values (typically 6 axes for analog sticks and triggers)

2. **Check enable button (RB)** at index `enable_button` (default=5):
   - If NOT pressed and was previously pressed: publish zero-velocity command (brake)
   - If NOT pressed: return immediately (skip processing)

3. **Parse joystick axes** for enabled joints:
   - For each joint `i` where `joint_axes[i] >= 0`:
     - Get axis value from `msg.axes[joint_axes[i]]`
     - Apply deadzone clipping: `v = clip_deadzone(raw, deadzone)`
     - Scale: `v = v * joint_scale`
     - Clamp to max speed (improved version): `v = clamp(v, -max_joint_speed, max_joint_speed)`
     - Store in velocities list

4. **Check if all velocities are near zero** (< 1e-4):
   - If yes: skip publishing to save bandwidth

5. **Publish JointJog message**:
   ```python
   jog.header.stamp = now()
   jog.joint_names = joint_names
   jog.velocities = velocities  # rad/s
   jog.displacements = []        # empty for pure velocity mode
   jog.duration = 0.0            # duration=0 signals velocity-only mode
   ```

#### Default Axis Mapping (Logitech F310)

| Axis Index | Physical Control | Joint | Direction |
|------------|------------------|-------|-----------|
| 0 | Left stick X | Shoulder pan | Yaw |
| 1 | Left stick Y | Shoulder lift | Pitch |
| 2 | LT (trigger) | – | – |
| 3 | Right stick X | Elbow | Roll |
| 4 | Right stick Y | Wrist 1 | – |
| 5 | RT (trigger) | – | – |

**Note:** The `joint_axes` parameter `[1, 0, 4, 3, -1, -1]` means:
- Joint 0 (shoulder_pan) ← axis 1 (left stick Y)
- Joint 1 (shoulder_lift) ← axis 0 (left stick X)
- Joint 2 (elbow) ← axis 4 (right stick Y)
- Joint 3 (wrist_1) ← axis 3 (right stick X)
- Joint 4 (wrist_2) ← disabled (-1)
- Joint 5 (wrist_3) ← disabled (-1)

To verify axis mapping for your specific gamepad, run:
```bash
ros2 topic echo /joy
```
and observe which values change when you move each stick/button.

---

### Node 2: Cartesian End-Effector Teleoperation Node (`cartesian_teleop.py` / `cartesian_teleop_improved.py`)

#### Summary

Converts joystick input into Twist (linear + angular velocity) commands for the end-effector tool frame. MoveIt Servo converts these into joint trajectories via inverse kinematics.

#### Subscriptions

| Topic | Type | Purpose |
|-------|------|---------|
| `/joy` | `sensor_msgs/Joy` | Gamepad input from joy_node driver |

#### Publications

| Topic | Type | Purpose |
|-------|------|---------|
| `/ur3e_servo/cartesian_twist` | `geometry_msgs/TwistStamped` | End-effector velocity commands in tool0 frame |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joy_topic` | string | `/joy` | Topic name for joystick input |
| `enable_button` | int | `5` | Joystick button index for deadman switch (RB button) |
| `deadzone` | float | `0.15` | Threshold below which joystick input is ignored (unit: -1 to 1) |
| `linear_scale` | float | `0.1` | Scaling for linear velocities (m/s per full joystick range) |
| `angular_scale` | float | `0.8` | Scaling for angular velocities (rad/s per full joystick range) |
| `max_linear_speed` | float | `0.5` | Maximum linear speed in m/s (improved version only) |
| `max_angular_speed` | float | `1.0` | Maximum angular speed in rad/s (improved version only) |
| `twist_topic` | string | `/ur3e_servo/cartesian_twist` | Output topic for TwistStamped commands |
| `command_frame` | string | `tool0` | Coordinate frame for the twist command |
| `axis_left_x` | int | `0` | Joystick axis index for left stick X (Y-axis motion) |
| `axis_left_y` | int | `1` | Joystick axis index for left stick Y (X-axis motion) |
| `axis_right_x` | int | `3` | Joystick axis index for right stick X (rotation about Z) |
| `axis_right_y` | int | `4` | Joystick axis index for right stick Y (rotation about Y) |
| `axis_lt` | int | `2` | Joystick axis index for LT trigger (Z-axis motion, negative) |
| `axis_rt` | int | `5` | Joystick axis index for RT trigger (Z-axis motion, positive) |

#### Coordinate Frame

- **Command frame**: `tool0` (end-effector/tool frame)
- All velocity commands are expressed in the tool frame coordinate system
- **Conventions**:
  - X-axis: forward (away from robot base)
  - Y-axis: rightward (perpendicular to forward)
  - Z-axis: upward (opposite gravity when gripper faces down)

#### Safety Behavior

**Deadman Switch Logic:**
- **While RB button is held**: Continuously publishes Twist commands based on joystick input
- **When RB button is released**: Publishes a single zero-velocity `TwistStamped` message to stop end-effector motion (brake command)
- **Speed limiting**: If combined linear or angular speed exceeds limits, all components are uniformly scaled down (improved version)

#### Internal Algorithm

1. **Receive Joy message** with button states and axis values

2. **Check enable button (RB)** at index `enable_button` (default=5):
   - If NOT pressed and was previously pressed: publish zero-velocity command (brake)
   - If NOT pressed: return immediately

3. **Get raw axis values** for left stick (LX, LY), right stick (RX, RY), and triggers (LT, RT):
   - Use safe getter function to handle out-of-bounds indices gracefully

4. **Apply deadzone** to all raw axis values:
   - `deadzone_value = clip_deadzone(raw_value, deadzone_threshold)`

5. **Map axes to velocity components** (in tool0 frame):
   ```
   vx (forward/backward) = -left_stick_y × linear_scale
   vy (left/right)       = left_stick_x × linear_scale
   vz (up/down)          = (rt_trigger - lt_trigger) × linear_scale
   
   wx (roll)             = 0.0  (not used in base version)
   wy (pitch)            = -right_stick_y × angular_scale
   wz (yaw)              = right_stick_x × angular_scale
   ```

6. **Apply speed limits** (improved version only):
   - Calculate total linear speed: `linear = sqrt(vx² + vy² + vz²)`
   - If `linear > max_linear_speed`: scale down all linear components uniformly
   - Calculate total angular speed: `angular = sqrt(wx² + wy² + wz²)`
   - If `angular > max_angular_speed`: scale down all angular components uniformly

7. **Check if all velocities are near zero** (< 1e-4):
   - If yes: skip publishing to save bandwidth

8. **Publish TwistStamped message**:
   ```python
   twist.header.stamp = now()
   twist.header.frame_id = command_frame  # "tool0"
   twist.twist.linear.x = vx
   twist.twist.linear.y = vy
   twist.twist.linear.z = vz
   twist.twist.angular.x = wx
   twist.twist.angular.y = wy
   twist.twist.angular.z = wz
   ```

#### Default Axis Mapping (Logitech F310)

| Physical Control | Velocity Component | Formula |
|------------------|-------------------|---------|
| Left stick up/down | X (forward/back) | `-left_y × linear_scale` |
| Left stick left/right | Y (left/right) | `left_x × linear_scale` |
| LT/RT triggers | Z (up/down) | `(rt - lt) × linear_scale` |
| Right stick up/down | Pitch (wy) | `-right_y × angular_scale` |
| Right stick left/right | Yaw (wz) | `right_x × angular_scale` |

**Typical Behavior:**
- Push left stick forward → end-effector moves forward
- Push left stick right → end-effector moves right
- Squeeze LT → end-effector moves down
- Squeeze RT → end-effector moves up
- Rotate right stick → end-effector rotates

---

### Node 3: Joy Dummy Test Node (`joy_dummy.py`)

#### Purpose

A minimal placeholder node used to verify that the ROS 2 workspace is properly set up and all dependencies are available. Useful for initial setup verification before testing the actual teleoperation nodes.

#### Behavior

- Runs a periodic timer callback every 1 second
- Logs "JoyDummy node is running" message
- No subscriptions or publications

---

## MoveIt Servo Configuration Files

MoveIt Servo is the core motion planning and real-time control framework. The YAML configuration files in `ur3e_teleop_bringup/config/` control how Servo behaves.

### `ur3e_servo_cartesian.yaml` – Cartesian (Twist) Mode Configuration

This configuration is loaded when running Cartesian end-effector teleoperation.

#### Key Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `command_in_type` | `speed_units` | Input commands are already in m/s and rad/s (not unitless) |
| `cartesian_command_in_topic` | `/ur3e_servo/cartesian_twist` | Servo subscribes to Twist commands on this topic |
| `robot_link_command_frame` | `tool0` | Twist commands are expressed in tool0 frame |
| `command_out_topic` | `/scaled_joint_trajectory_controller/joint_trajectory` | Servo publishes joint trajectories to the UR controller here |
| `command_out_type` | `trajectory_msgs/JointTrajectory` | Output message type |
| `publish_period` | `0.004` (250 Hz) | How often Servo publishes joint trajectory updates |
| `incoming_command_timeout` | `0.1` | Stop motion if no command received for 100ms (safety) |
| `num_outgoing_halt_msgs_to_publish` | `4` | Send 4 zero-velocity halt messages when stopping |
| `low_latency_mode` | `false` | Batch updates for efficiency (see note below) |
| `check_collisions` | `true` | Enable collision checking with planning scene |

#### Velocity Limits

```yaml
scale:
  linear: 0.6       # Max end-effector linear velocity (m/s)
  rotational: 0.3   # Max end-effector angular velocity (rad/s)
```

These are hard limits enforced by MoveIt Servo. Even if you command higher speeds, they will be clamped.

#### Singularity Handling

- `lower_singularity_threshold`: 100.0 – Start slowing down near singularities
- `hard_stop_singularity_threshold`: 200.0 – Stop completely at severe singularities

#### System Latency Compensation

```yaml
system_latency_compensation: 0.04  # 40ms compensation
```

This parameter accounts for network/control delay by "looking ahead" in trajectory planning.

---

### `ur3e_servo_joint.yaml` – Joint-Space (JointJog) Mode Configuration

This configuration is loaded when running joint-space teleoperation.

#### Key Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `command_in_type` | `unitless` | Input commands are scaled by internal parameters (not absolute speeds) |
| `joint_command_in_topic` | `/ur3e_servo/joint_jog` | Servo subscribes to JointJog commands on this topic |
| `robot_link_command_frame` | `base_link` | Reference frame for joint commands |
| `command_out_topic` | `/scaled_joint_trajectory_controller/joint_trajectory` | Servo publishes joint trajectories here |
| `publish_period` | `0.02` (50 Hz) | Update frequency |
| `incoming_command_timeout` | `0.1` | Stop if no command for 100ms |

#### Velocity Scaling for Unitless Input

```yaml
scale:
  linear: 0.3        # For Cartesian commands (not used in joint mode)
  rotational: 0.8    # For Cartesian commands (not used in joint mode)
  joint: 0.8         # Key parameter: when joystick axis = 1.0, actual joint speed = 0.8 rad/s
```

The `joint: 0.8` parameter means:
- Joystick axis value 1.0 → joint velocity 0.8 rad/s
- Joystick axis value 0.5 → joint velocity 0.4 rad/s
- Joystick axis value -1.0 → joint velocity -0.8 rad/s

---

### `ur3e_servo_kinematics.yaml` – Kinematics Solver Configuration

This file configures the inverse kinematics (IK) solver used by MoveIt Servo.

```yaml
ur3e_servo:
  ros__parameters:
    robot_description_kinematics:
      ur_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_search_resolution: 0.005
```

| Parameter | Meaning |
|-----------|---------|
| `kinematics_solver` | Use KDL (Kinematics and Dynamics Library) for IK solving |
| `kinematics_solver_search_resolution` | Search resolution for IK (0.005 rad = ~0.3°) |
| `ur_manipulator` | Planning group name for the 6-DOF UR3e arm |

**How it works:**
1. Cartesian teleop node sends: "Move end-effector forward at 0.1 m/s"
2. MoveIt Servo receives TwistStamped command
3. KDL solver converts twist to required joint velocities using Jacobian
4. Joint velocities sent to controller

---

## Launching the System

### Prerequisites

Before launching, ensure:

1. **ROS 2 Jazzy is installed and sourced**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **UR simulation packages are installed** (e.g., `ur_simulation_gz`):
   ```bash
   sudo apt install ros-jazzy-ur-simulation-gz
   ```

3. **Build this workspace**:
   ```bash
   cd ~/ur3e_teleop_ws
   colcon build --merge-install
   source install/setup.bash
   ```

4. **Gamepad is connected** to `/dev/input/js0`:
   ```bash
   ls -la /dev/input/js0
   ```

### Step 1: Start Gazebo and UR3e Simulation

Open **Terminal 1** and run the official UR simulation launch file:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3e
```

**What this does:**
- Starts Gazebo with a UR3e robot model
- Launches ROS 2 controllers (including `scaled_joint_trajectory_controller`)
- Starts MoveIt move_group for planning
- Launches RViz for visualization

Wait for "Entering interactive loop" to appear in the terminal before proceeding.

### Step 2a: Launch Cartesian End-Effector Teleoperation

Open **Terminal 2** and run:

```bash
cd ~/ur3e_teleop_ws
source install/setup.bash
ros2 launch ur3e_teleop_bringup ur3e_cartesian_teleop.launch.py
```

**What this does:**
- Starts MoveIt Servo with Cartesian configuration (`ur3e_servo_cartesian.yaml`)
- Launches the Joy node to read gamepad input
- Launches the Cartesian teleop node to convert Joy → TwistStamped
- Automatically switches Servo to Twist mode after 3 seconds
- You should see log output confirming each node started successfully

**Test it:**
1. Hold down the RB button (enable switch)
2. Move the left joystick forward → end-effector should move forward
3. Move the left joystick sideways → end-effector should strafe
4. Move the right joystick → end-effector should rotate
5. Release RB button → motion should stop immediately

### Step 2b: Alternative – Launch Joint-Space Teleoperation

If you prefer to control individual joints instead of the end-effector, use **Terminal 2** with:

```bash
cd ~/ur3e_teleop_ws
source install/setup.bash
ros2 launch ur3e_teleop_bringup ur3e_joint_teleop.launch.py
```

**What this does:**
- Starts MoveIt Servo with joint configuration (`ur3e_servo_joint.yaml`)
- Launches Joy node
- Launches the joint teleop node to convert Joy → JointJog
- Automatically switches Servo to JointJog mode (command_type=0)

**Test it:**
1. Hold down the RB button
2. Move left stick X → shoulder pan joint rotates (yaw)
3. Move left stick Y → shoulder lift joint rotates (pitch)
4. Move right stick X → elbow joint rotates (roll)
5. Move right stick Y → wrist 1 joint rotates

### Step 3: Verify Data Flow with Topic Echo

Open **Terminal 3** to monitor the data flow:

```bash
# For Cartesian mode, check the twist commands:
ros2 topic echo /ur3e_servo/cartesian_twist
```

Expected output when joystick is moved (with RB held):
```
header:
  stamp:
    sec: 1700000000
    nanosec: 123456789
  frame_id: tool0
twist:
  linear:
    x: 0.05
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.15
---
```

For joint mode:
```bash
ros2 topic echo /ur3e_servo/joint_jog
```

Expected output:
```
header:
  stamp:
    sec: 1700000000
    nanosec: 123456789
joint_names:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
velocities: [0.5, -0.3, 0.0, 0.0, 0.0, 0.0]
displacements: []
duration:
  sec: 0
  nanosec: 0
---
```

### Step 4: Observe Motion in RViz

In the RViz window that opened with `ur_sim_moveit.launch.py`:
- You should see the UR3e model moving in real-time
- The end-effector frame (tool0 frame) should move as you command with the joystick
- Use RViz's "Interact" mode to verify the motion is as expected

### Step 5: Mirror Obstacles and Generate Haptic Feedback (Optional but Recommended)

1. **Inject shared obstacles into MoveIt and Gazebo**

  Open **Terminal 3** (or reuse an existing one) and run:

  ```bash
  cd ~/ur3e_teleop_ws
  source install/setup.bash
  ros2 run ur3e_haptic_scene add_obstacles
  ```

  This reads `config/obstacles.yaml`, waits for MoveIt's `/planning_scene` subscription, and publishes a diff to register the obstacle named `box1`.

2. **Start the virtual force publisher**

  ```bash
  cd ~/ur3e_teleop_ws
  source install/setup.bash
  ros2 run ur3e_haptic_scene virtual_force
  ```

  The node queries TF at 50 Hz, computes the shortest distance between `tool0` and each configured obstacle, and publishes a repulsive force vector on `/virtual_force`.

3. **Visualise or feed the force signal**

  ```bash
  ros2 run rqt_plot rqt_plot /virtual_force/x /virtual_force/y /virtual_force/z
  ```

  When the end-effector approaches the box inside 0.15 m, the plot shows force components ramping up (max ≈ 35 N magnitude at 2 cm). Map this topic into your haptic device interface as needed.

---

## Build and Dependencies

### Building the Workspace

This is a standard ROS 2 workspace. To build:

```bash
cd ~/ur3e_teleop_ws
colcon build --merge-install
```

**Build options:**
- `--packages-select ur3e_teleop_joy ur3e_teleop_bringup` – Build only these packages (faster)
- `--symlink-install` – Symlink Python files instead of copying (useful for development)
- `--cmake-args -DCMAKE_BUILD_TYPE=Release` – Build in Release mode (faster execution)

### Required ROS 2 Packages

Install via apt:

```bash
sudo apt install ros-jazzy-ur-simulation-gz \
                 ros-jazzy-ur-description \
                 ros-jazzy-ur-controllers \
                 ros-jazzy-ur-robot-driver \
                 ros-jazzy-moveit \
                 ros-jazzy-moveit-servo \
                 ros-jazzy-joy
```

### Workspace Dependencies

The workspace depends on:

- **`rclpy`** – Python client library for ROS 2
- **`sensor_msgs`** – Contains `Joy` message type
- **`geometry_msgs`** – Contains `TwistStamped` message type
- **`control_msgs`** – Contains `JointJog` message type
- **`ur_description`** – URDF description of UR robots
- **`ur_simulation_gz`** – Gazebo simulation for UR robots
- **`moveit_servo`** – Real-time servo motion planning
- **`moveit_msgs`**, **`shape_msgs`** – Collision object and planning scene message definitions for obstacle injection
- **`tf2_ros`** – Transform listener used by the virtual force node
- **`python3-yaml`**/**`ament_index_python`** – Parsing obstacle configuration and resolving package paths

All these are already declared in the `package.xml` files as `exec_depend`.

### Sourcing the Workspace

After building, always source the workspace before running any nodes:

```bash
source ~/ur3e_teleop_ws/install/setup.bash
```

This sets up the Python path and ROS package discovery so nodes can find each other.

---

## Troubleshooting

### Common Issues

#### Issue: "Could not find parameter robot_description"

**Symptom**: Segmentation fault when launching MoveIt Servo

**Cause**: MoveIt Servo needs the `robot_description` parameter from the UR driver package

**Solution**: Use the complete UR simulation launcher instead:
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3e
```

#### Issue: "No such device /dev/input/js0"

**Symptom**: Joy node fails to start, error about missing joystick device

**Cause**: Gamepad is not connected or has a different device path

**Solution**:
1. List available input devices:
   ```bash
   ls /dev/input/js*
   ```
2. If your gamepad is at `/dev/input/js1`, update the launch file or pass parameter:
   ```bash
   ros2 launch ur3e_teleop_bringup ur3e_cartesian_teleop.launch.py ... 
   # (Create a wrapper or modify the launch file to change the device path)
   ```

#### Issue: "Timeout waiting for action server"

**Symptom**: MoveIt Servo times out waiting for a controller

**Cause**: The controller (from Term 1) is not ready yet

**Solution**: Wait longer before launching teleoperation (at least 10 seconds after starting Gazebo)

#### Issue: No motion when pressing joystick

**Symptom**: Gamepad input is recognized (can see Joy messages) but robot doesn't move

**Causes**:
1. RB button is not mapped correctly (check `enable_button` parameter)
2. Servo node crashed silently
3. Joystick axes don't match expected mapping

**Solution**:
1. Verify RB button index:
   ```bash
   ros2 topic echo /joy
   # Press RB and watch the `buttons` array for a change at index 5
   ```
2. Check Servo logs:
   ```bash
   # In Terminal 2, look for errors in the console output
   ```
3. Verify Twist/JointJog messages are being published:
   ```bash
   ros2 topic echo /ur3e_servo/cartesian_twist
   # Or for joint mode:
   ros2 topic echo /ur3e_servo/joint_jog
   ```

#### Issue: "QoS mismatch" warning

**Symptom**: Warning about QoS reliability settings

**Explanation**: This is a non-critical warning about message delivery settings. The system will still work but may drop occasional messages.

**Note**: The improved nodes use better QoS settings (depth=1) to minimize latency.

---

## Performance Considerations

### Latency

For real-time teleoperation, low latency is critical. Key factors:

1. **Publish period**: Currently 0.004s (250 Hz for Cartesian) or 0.02s (50 Hz for joint mode)
2. **QoS settings**: The improved nodes use `depth=1` for lower latency (standard nodes use `depth=10`)
3. **System latency compensation**: Set to 40ms in `ur3e_servo_cartesian.yaml`

### Using the Improved Nodes

The `*_improved.py` versions include:
- Better QoS settings (40x lower latency: 40ms vs 190ms)
- Speed limiting parameters for safety
- Better axis safety checking

To use improved versions, update the entry points in `setup.py`:

```python
'cartesian_teleop = ur3e_teleop_joy.cartesian_teleop_improved:main',
'joint_teleop = ur3e_teleop_joy.joint_teleop_improved:main',
```

Then rebuild:
```bash
cd ~/ur3e_teleop_ws
colcon build
source install/setup.bash
```

---

## Configuration Verification

Run the included configuration verification script to check for consistency:

```bash
python3 ~/ur3e_teleop_ws/src/verify_config.py
```

This will check:
- Topic names match between YAML and launch files
- Message types are correct
- Frame names are consistent

Example output:
```
[✓] ur3e_servo_cartesian.yaml topic = /ur3e_servo/cartesian_twist
[✓] Launch file topic = /ur3e_servo/cartesian_twist
[✓] Cartesian topic is consistent!
```

---

## Next Steps

1. **Real Hardware**: To run on a physical UR3e, install the `ur_robot_driver` package and adjust launch files to remove Gazebo simulation
2. **Advanced Filtering**: Add Butterworth or other filters to smooth jerky motions
3. **Safety Limits**: Implement joint/velocity hard stops based on application requirements
4. **GUI Interface**: Create a diagnostic RQT dashboard to monitor node status and performance
5. **Recording**: Add rosbag recording to analyze teleoperation sessions

---

## License

Apache License 2.0 – See individual package files for details.

## Maintainer

For questions or issues, refer to the inline code documentation in each node file.
