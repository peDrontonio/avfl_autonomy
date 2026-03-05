# AVFL Autonomy

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL-green?logo=ardupilot&logoColor=white)](https://ardupilot.org/)
[![Docker](https://img.shields.io/badge/Docker-Container-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-Ultralytics-00FFFF?logo=yolo&logoColor=white)](https://docs.ultralytics.com/)

## Overview

**AVFL Autonomy** is an autonomous flight stack for UAVs (Unmanned Aerial Vehicles) developed as part of a research internship project. The system integrates [ArduPilot](https://ardupilot.org/) as the flight controller with [ROS 2 Humble](https://docs.ros.org/en/humble/) as the robotics middleware, enabling fully autonomous drone missions on real hardware. This branch (`jetson`) is configured to run on a Jetson companion computer connected to the flight controller via USB.

### Key Features

- **Autonomous Mission Execution** — Missions are structured as Finite State Machines (FSMs), enabling modular, readable, and extensible behavior logic with clearly defined state transitions.
- **Computer Vision Pipeline** — Integrates OpenCV for ArUco marker detection (Mission 1) and YOLOv11 for real-time object detection via a bottom-facing camera (Mission 2).
- **Navigation Services** — A custom navigation layer provides position-based control, GPS waypoint navigation, yaw control, and telemetry services, abstracting ArduPilot's low-level interface into high-level ROS 2 services.
- **Micro-ROS Agent** — Communication between ArduPilot (on the flight controller) and ROS 2 (on the Jetson) is handled by the Micro-ROS Agent over a USB serial connection.
- **Dockerized Environment** — The runtime stack (ROS 2, Micro-ROS Agent, ArduPilot messages, and all dependencies) runs inside a Docker container on the Jetson, ensuring reproducibility and eliminating dependency conflicts.

### Missions

| Mission | Description | Vision |
|---|---|---|
| **Mission 1 — ArUco Gate Passing** | The drone takes off, searches for a gate composed of 4 ArUco markers, aligns and centers itself relative to the gate, flies through it, and lands. | OpenCV ArUco detection + pose estimation |
| **Mission 2 — Helipad Landing** | The drone takes off, navigates via GPS to a target area, searches for the helipad using a downward-facing camera, descends while centralizing, and performs a precision landing. | YOLOv11 real-time object detection |

The architecture is designed to be extensible: new autonomous missions can be developed by creating additional FSM nodes and reusing the existing navigation services.

### Architecture

This branch contains one main ROS 2 package:

| Package | Role |
|---|---|
| `drone_navigate` | Core navigation and control package providing reusable ROS 2 services (`navigate`, `get_telemetry`, `set_yaw_rate`, `navigate_global`), custom messages/services, mission FSM nodes, and the computer vision nodes (ArUco detector, YOLO detector). |

Communication between ArduPilot (running on the flight controller) and ROS 2 (running on the Jetson) is handled by the Micro-ROS Agent over USB serial (`/dev/ttyACM1`). The agent exposes all ArduPilot topics and services as standard ROS 2 interfaces.

---

## Table of Contents

- [Setup](#setup)
- [Running the System](#running-the-system)
  - [Step 1 — Micro-ROS Agent](#step-1--micro-ros-agent)
  - [Step 2 — Navigation Services](#step-2--navigation-services)
  - [Step 3 — Mission](#step-3--mission)
- [Mission 1 — ArUco Gate Passing](#mission-1--aruco-gate-passing)
- [Mission 2 — Helipad Landing](#mission-2--helipad-landing)
- [Useful Reference](#useful-reference)
- [Project Structure](#project-structure)

---

## Setup

### Docker (Jetson)

The environment runs inside a Docker container on the Jetson. There are three scripts in the `docker/` folder:

| Script | Purpose |
|---|---|
| `./build_jetson.sh` | Builds the Docker image for the Jetson. **Only needed once** (or when the Dockerfile changes). |
| `./run_jetson.sh` | Starts the container in detached mode (runs in background). |
| `./run_jetson.sh exec` | Opens an additional terminal in the **already running** container. |

```bash
cd src/avfl_autonomy/docker
./build_jetson.sh        # first time only
./run_jetson.sh          # start container (runs in background)
./run_jetson.sh exec     # open terminals inside the container
```

> **Note:** The Jetson container **does not include Gazebo or SITL** — it is built exclusively for real drone operation. The `drone_navigate` workspace is automatically mounted inside the container at `/root/avfl_ws/src/drone_navigate`.

### Building the Workspace

Inside the container:

```bash
cd ~/avfl_ws
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** The container's `.bashrc` already sources all required workspaces (`/opt/ros/humble`, `ardupilot_msgs`, `micro_ros_agent`, and `avfl_ws`), so every new terminal is ready automatically.

---

## Running the System

To run the real drone, the components must be started **manually and in the correct order**. The Micro-ROS Agent must be started first to bridge communication between ArduPilot (running on the flight controller) and ROS 2 (running on the Jetson).

### Startup Order

```
1. Micro-ROS Agent  →  2. Navigation Services  →  3. Mission
```

> **⚠️ Important:** The Micro-ROS Agent **must be started manually** in a separate terminal **before** any other ROS 2 node. Without it, there is no communication with ArduPilot and no drone topics/services will be available.

---

### Step 1 — Micro-ROS Agent

The Micro-ROS Agent creates the bridge between ArduPilot (on the flight controller) and ROS 2 via USB. It **must be started first** and kept running in a dedicated terminal.

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 2000000
```

| Parameter | Default | Description |
|---|---|---|
| `--dev` | `/dev/ttyACM1` | USB port connected to the flight controller |
| `-b` | `2000000` | Baud rate for USB communication (2M) |

Wait until you see ArduPilot topics appearing (e.g., `/ap/...`). You can verify in another terminal with:

```bash
ros2 topic list
```

> **Note:** The USB device may appear as `/dev/ttyACM0` or `/dev/ttyACM1` depending on your setup. Check with `ls /dev/ttyACM*` and adjust the `--dev` parameter accordingly.

---

### Step 2 — Navigation Services

In a **new terminal**, start the navigation services. Here we **do not** use `use_sim_time` (the default is already `false`):

```bash
ros2 launch drone_navigate avfl.launch.py
```

This launch file starts:
- Navigation services (`/avfl/navigate`, `/avfl/navigate_global`)
- Telemetry (`/get_telemetry`)
- Yaw control (`/set_yaw_rate`)
- ArUco detector (real camera)

> **Alternative:** If you want to include the Micro-ROS Agent in the same launch (without starting it manually), use:
> ```bash
> ros2 launch drone_navigate avfl_with_dds.launch.py serial_port:=/dev/ttyACM1 baud_rate:=2000000
> ```
> However, running the agent separately is **recommended** for easier debugging and independent restarts.

---

### Step 3 — Mission

In a **new terminal**, launch the desired mission and then trigger the start service.

---

## Mission 1 — ArUco Gate Passing

The drone takes off, searches for a gate with 4 ArUco markers, centers itself, and flies through.

**FSM:** `Takeoff → Searching → Aligning → Centering → Advancing → Landing`

Gate layout (ArUco IDs 0–3):
```
  2 --- 3
  |     |
  0 --- 1
```

> **Important:** Before running the commands below, make sure the workspace was built and sourced, and the Micro-ROS Agent is already running.

Open **3 terminals** inside the container (use `./run_jetson.sh exec` for each new terminal):

| Terminal | Command |
|---|---|
| 1 — Micro-ROS | `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 2000000` |
| 2 — Nav + ArUco | `ros2 launch drone_navigate avfl.launch.py` |
| 3 — Mission | `ros2 launch drone_navigate mission1.launch.py` |

After all nodes are running, trigger the mission:

```bash
ros2 service call /start_mission1 std_srvs/srv/Trigger
```

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `takeoff_alt` | `10.0` | Takeoff altitude (m) |

Example: `ros2 launch drone_navigate mission1.launch.py takeoff_alt:=5.0`

---

## Mission 2 — Helipad Landing

The drone takes off, navigates via GPS to the helipad area, searches for the helipad using the bottom camera (YOLO), and lands on it.

**FSM:** `Takeoff → GoingToBase → Search_Base → Scan_for_Base → Descend_and_Centralize → Landing`

If the helipad is lost during descent, the FSM returns to `Scan_for_Base`. A safety `ReturnBase` (RTL) state is triggered after max retries.

> **Important:** Before running the commands below, make sure the workspace was built and sourced, and the Micro-ROS Agent is already running.

Open **3 terminals** inside the container (use `./run_jetson.sh exec` for each new terminal):

| Terminal | Command |
|---|---|
| 1 — Micro-ROS | `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 2000000` |
| 2 — Nav + ArUco | `ros2 launch drone_navigate avfl.launch.py` |
| 3 — Mission | `ros2 launch drone_navigate mission2.launch.py base_lat:=<LAT> base_lon:=<LON>` |

After all nodes are running, trigger the mission:

```bash
ros2 service call /start_mission2 std_srvs/srv/Trigger
```

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `base_lat` | `0.0` | Base GPS latitude |
| `base_lon` | `0.0` | Base GPS longitude |
| `takeoff_alt` | `10.0` | Takeoff altitude (m) |
| `search_alt` | `8.0` | Search altitude (m) |

Example:
```bash
ros2 launch drone_navigate mission2.launch.py \
    base_lat:=-35.363257 base_lon:=149.165400 takeoff_alt:=10.0 search_alt:=8.0
```

---

## Useful Reference

<details>
<summary><strong>FastDDS Configuration (UDP)</strong></summary>

The `fastdds_udp.xml` file at the repository root configures DDS to use UDP transport. This may be needed for communication between multiple devices on the same network. To enable it, set the environment variable before running any ROS 2 node:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/avfl_ws/src/drone_navigate/../fastdds_udp.xml
```

</details>

<details>
<summary><strong>Manual Commands</strong></summary>

> **Note:** To arm and take off, the EKF must be using GPS. Check the Micro-ROS Agent logs to verify communication is active.

**Pre-arm check:**
```bash
ros2 service call /ap/prearm_check std_srvs/srv/Trigger
```

**Arm motors:**
```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
```

**Takeoff** (altitude in meters):
```bash
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.0}"
```

**Land** (mode 9 = Land — the drone cannot be re-armed after landing this way):
```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"
```

</details>

<details>
<summary><strong>Navigate Service (<code>/avfl/navigate</code>)</strong></summary>

Navigates to a position using **local coordinates** (meters) with velocity control.

```bash
ros2 service call /avfl/navigate drone_navigate/srv/Navigate \
    "{x: 0.0, y: 0.0, z: 2.0, yaw: nan, yaw_rate: 0.0, speed: 0.5, frame_id: 'map', auto_arm: true}"
```

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `x` | float | Target X position (meters) |
| `y` | float | Target Y position (meters) |
| `z` | float | Target Z position / altitude (meters) |
| `yaw` | float | Target yaw angle in radians. Use `nan` to maintain current heading |
| `yaw_rate` | float | Yaw rotation rate in rad/s. Yaw rotation only happens if **both** `yaw` is set (not `nan`) **and** `yaw_rate > 0` |
| `speed` | float | Navigation speed in m/s (capped by `max_velocity`, default 1.0 m/s) |
| `frame_id` | string | Reference frame (see below) |
| `auto_arm` | bool | If `true`, automatically switches to GUIDED mode and arms before navigating |

**`frame_id` values for `/avfl/navigate`:**

| Value | Behavior |
|---|---|
| `map` | **Absolute** position in the local EKF frame. `x`, `y`, `z` are treated as final coordinates. |
| `body` | **Relative** offset from the drone's current position, rotated by the drone's current yaw. `x` = forward, `y` = left, `z` = up. |

**Examples:**

```bash
# Go to absolute position (1, 2, 5) in the map frame, auto-arm
ros2 service call /avfl/navigate drone_navigate/srv/Navigate \
    "{x: 1.0, y: 2.0, z: 5.0, yaw: nan, yaw_rate: 0.0, speed: 0.5, frame_id: 'map', auto_arm: true}"

# Move 3m forward and 2m up relative to the drone (body frame)
ros2 service call /avfl/navigate drone_navigate/srv/Navigate \
    "{x: 3.0, y: 0.0, z: 2.0, yaw: nan, yaw_rate: 0.0, speed: 0.5, frame_id: 'body', auto_arm: false}"

# Go to position (0, 0, 2) while rotating to yaw=1.57 rad (90°) at 0.5 rad/s
ros2 service call /avfl/navigate drone_navigate/srv/Navigate \
    "{x: 0.0, y: 0.0, z: 2.0, yaw: 1.57, yaw_rate: 0.5, speed: 0.5, frame_id: 'map', auto_arm: false}"
```

</details>

<details>
<summary><strong>Navigate Global Service (<code>/avfl/navigate_global</code>)</strong></summary>

Navigates to a position using **GPS coordinates** (lat/lon) via the ArduPilot GlobalPosition interface.

```bash
ros2 service call /avfl/navigate_global drone_navigate/srv/NavigateGlobal \
    "{lat: -35.363661, lon: 149.166230, z: 10.0, yaw: 0.0, yaw_rate: 0.0, speed: 2.0, frame_id: 'rel', auto_arm: true}"
```

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `lat` | float64 | Target latitude in degrees |
| `lon` | float64 | Target longitude in degrees |
| `z` | float | Target altitude in meters (interpretation depends on `frame_id`) |
| `yaw` | float | Target yaw angle in radians. Use `0` to maintain current heading |
| `yaw_rate` | float | Yaw rotation rate in rad/s. Yaw rotation only happens if **both** `yaw ≠ 0` **and** `yaw_rate > 0` |
| `speed` | float | Navigation speed in m/s (default 2.0 m/s) |
| `frame_id` | string | Altitude reference frame (see below) |
| `auto_arm` | bool | If `true`, automatically switches to GUIDED mode and arms before navigating |

**`frame_id` values for `/avfl/navigate_global`:**

| Value | Behavior |
|---|---|
| `rel` / `relative` | **Relative altitude** — `z` is added to the current altitude. E.g., if the drone is at 5m and `z=3.0`, it flies to 8m. |
| `map` (or any other) | **Absolute altitude** — `z` is the final target altitude. |

**Examples:**

```bash
# Fly to GPS coordinates, maintaining current altitude + 10m
ros2 service call /avfl/navigate_global drone_navigate/srv/NavigateGlobal \
    "{lat: -35.363257, lon: 149.165400, z: 10.0, yaw: 0.0, yaw_rate: 0.0, speed: 2.0, frame_id: 'rel', auto_arm: true}"

# Fly to GPS coordinates at an absolute altitude of 15m
ros2 service call /avfl/navigate_global drone_navigate/srv/NavigateGlobal \
    "{lat: -35.363257, lon: 149.165400, z: 15.0, yaw: 0.0, yaw_rate: 0.0, speed: 2.0, frame_id: 'map', auto_arm: false}"
```

</details>

<details>
<summary><strong>Flight Modes</strong></summary>

| Mode | Name |
|:---:|:---:|
| 0 | Stabilize |
| 1 | Acro |
| 2 | Alt Hold |
| 3 | Auto |
| 4 | Guided |
| 5 | Loiter |
| 6 | Return to Launch (RTL) |
| 7 | Circle |
| 8 | Position |
| 9 | Land |
| 11 | Drift |
| 13 | Sport |
| 14 | Flip |
| 15 | Autotune |
| 16 | PosHold |
| 17 | Brake |
| 18 | Throw |
| 19 | Avoid ADSB |
| 20 | Guided No GPS |

</details>

---

## Project Structure

```
avfl_autonomy/
├── docker/                  # Docker build & run scripts (Jetson)
│   ├── build_jetson.sh      # Build the Docker image
│   ├── dockerfile.jetson    # Dockerfile for Jetson (no Gazebo)
│   └── run_jetson.sh        # Run/exec the container
├── docs/                    # Documentation
├── fastdds_udp.xml          # FastDDS UDP transport config
└── drone_navigate/          # Navigation & control package
    ├── drone_navigate/      # Python modules (services)
    ├── launch/              # Launch files
    │   ├── avfl.launch.py             # Nav + ArUco (real drone)
    │   ├── avfl_with_dds.launch.py    # Nav + ArUco + Micro-ROS Agent
    │   ├── drone_services.launch.py   # Navigation services only
    │   ├── mission1.launch.py         # Mission 1 nodes
    │   └── mission2.launch.py         # Mission 2 nodes
    ├── msg/                 # Custom message definitions
    ├── scripts/             # Utility scripts (aruco, yolo, etc.)
    ├── src/                 # Navigation/detection nodes
    └── srv/                 # Custom service definitions
```
