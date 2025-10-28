# Teleop-UR3 (ROS 2 Jazzy + CoppeliaSim 4.10)

Gamepad → ROS 2 `/joy` → Mapping Node `/ur3_cmd` → UR3Bridge → CoppeliaSim  
Control a UR3 robot in CoppeliaSim using a Logitech F310 (XInput mode).

---

## 1. Prerequisites
- Ubuntu 24.04
- ROS 2 **Jazzy** installed at `/opt/ros/jazzy`
- Python 3.12 venv at `~/ros2venv`
- CoppeliaSim **4.10** with ZMQ Remote API (default rpcPort=23000)
- Logitech F310 gamepad (XInput)

---

## 2. Repository Layout (this repo = one ROS 2 Python package)
```
ur3_bridge/            # Python package (nodes)
resource/              # ament index resource (keep)
test/                  # tests (optional)
scenes/ur3.ttt         # sample CoppeliaSim scene
scripts/bringup.sh     # env loader
package.xml, setup.py, setup.cfg, README.md, LICENSE, requirements.txt
```

> **Note**: This repository is a **package-level** repo.  
> Users should clone it into their own workspace `src/` and build with `colcon`.

---

## 3. Clone and Build

### 3.1 Clone the repository
```bash
mkdir -p ~/ros2_ws_1/src
cd ~/ros2_ws_1/src
git clone https://github.com/Navvy000/teleop-ur3.git ur3_bridge_repo
cd ~/ros2_ws_1
colcon build
source install/setup.bash
```

### 3.2 Python dependencies (venv)
```bash
python3 -m venv ~/ros2venv
source ~/ros2venv/bin/activate
pip install -r ~/ros2_ws_1/src/ur3_bridge/requirements.txt
```

---

## 4. Run (three terminals)
> In **each terminal**, first load environment:
```bash
cd ~/ros2_ws_1/src/ur3_bridge
source scripts/bringup.sh
```

**Terminal A — UR3 Bridge (ROS → CoppeliaSim)**
```bash
ros2 run ur3_bridge bridge_node
```

**Terminal B — Gamepad → /joy**
```bash
ros2 run joy joy_node --ros-args   -p dev:=/dev/input/by-id/<your-F310-joystick-path>   -p deadzone:=0.05   -p autorepeat_rate:=50.0
```

**Terminal C — /joy → /ur3_cmd**
```bash
ros2 run ur3_bridge gamepad_to_ur3_cmd
```

Open CoppeliaSim and load **`scenes/ur3.ttt`**, press **▶ Run** (ZMQ server message should appear).

---

## 5. Controls (default)
- **LB**: enable (hold-to-run)
- **B**: E-Stop (latching)  
- **Start**: reset E-Stop (if implemented)
- **Y**: home (set targets to zeros)
- Sticks map to 6 joints (axes indices & inversion set for F310 XInput).  
  If directions feel inverted, adjust `axes.*.invert` params.

---

## 6. Notes
- `PYTHONPATH` is merged so both **venv** packages and **ROS 2** Python libs are importable.
- Do **not** add ROS 2 system packages to `requirements.txt`.
- If `/joy` has no output, check gamepad path under `/dev/input/by-id/`.

---

## 7. Versions
- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, CoppeliaSim 4.10, Logitech F310 (XInput)

---

**Author:** Zhenyu Yuan  
**Date:** October 2025  
**Project:** MSc Computer Science (AI) – University of Nottingham  
**License:** Apache 2.0
