# AVFL Autonomy

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL-green?logo=ardupilot&logoColor=white)](https://ardupilot.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Autonomous flight stack for drones built on top of ArduPilot and ROS 2. This repository contains mission packages for autonomous tasks such as ArUco gate detection and traversal, and landing on a helipad using YOLO-based detection.

All development and simulation is done inside a Docker container to ensure a consistent, reproducible environment across machines, avoiding dependency conflicts and simplifying setup.

<!-- TODO: Add a screenshot or GIF of the simulation here -->
<!-- ![Simulation Demo](docs/demo.gif) -->

---

## Table of Contents

- [Setup](#setup)
- [Launching the World](#launching-the-world)
- [Mission 1 — ArUco Gate Passing](#mission-1--aruco-gate-passing)
- [Mission 2 — Mobile Base Landing](#mission-2--mobile-base-landing)
- [Useful Reference](#useful-reference)
- [Project Structure](#project-structure)

---

## Setup

### Docker

The entire environment runs inside a Docker container. There are three scripts in the `docker/` folder:

| Script | Purpose |
|---|---|
| `./build.sh` | Builds the Docker image. **Only needed once** (or when the Dockerfile changes). |
| `./run.sh` | Starts a new container. The container is removed on exit (`--rm`). |
| `./run.sh exec` | Opens an additional terminal in the **already running** container. |

```bash
cd src/avfl_autonomy/docker
./build.sh          # first time only
./run.sh            # start container (Terminal 1)
./run.sh exec       # additional terminals (from a new host terminal)
```

### Building the Workspace

Inside the container:

```bash
cd ~/internship_ws
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Add `source ~/internship_ws/install/setup.bash` to your `~/.bashrc` so every new terminal is ready automatically.

---

## Launching the World

Starts Gazebo (server + GUI), ArduPilot SITL, ROS↔Gazebo bridges, and RViz.

```bash
ros2 launch drone_gazebo simple_world.launch.py
```

To disable RViz:

```bash
ros2 launch drone_gazebo simple_world.launch.py rviz:=false
```

Wait until the SITL and bridges are fully initialized before launching any mission.

> **Important:** The drone can only arm and take off once the EKF is using GPS. Look for the GPS confirmation message in the SITL terminal logs before starting any mission.

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

Open **4 terminals** inside the Docker container (use `./run.sh exec` for each new terminal):

| Terminal | Command |
|---|---|
| 1 — Gazebo | `ros2 launch drone_gazebo simple_world.launch.py` |
| 2 — Navigation | `ros2 launch drone_navigate drone_services.launch.py use_sim_time:=true` |
| 3 — Mission | `ros2 launch drone_gazebo mission1.launch.py` |
| 4 — Start | `ros2 service call /start_mission1 std_srvs/srv/Trigger` |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `takeoff_alt` | `10.0` | Takeoff altitude (m) |

Example: `ros2 launch drone_gazebo mission1.launch.py takeoff_alt:=15.0`

---

## Mission 2 — Helipad Landing

The drone takes off, navigates via GPS to the base area, searches for the mobile base using the bottom camera (YOLO), and lands on it.

**FSM:** `Takeoff → GoingToBase → Search_Base → Scan_for_Base → Descend_and_Centralize → Landing`

If the base is lost during descent, the FSM returns to `Scan_for_Base`. A safety `ReturnBase` (RTL) state is triggered after max retries.

Open **4 terminals** inside the Docker container (use `./run.sh exec` for each new terminal):

| Terminal | Command |
|---|---|
| 1 — Gazebo | `ros2 launch drone_gazebo simple_world.launch.py` |
| 2 — Navigation | `ros2 launch drone_navigate drone_services.launch.py use_sim_time:=true` |
| 3 — Mission | `ros2 launch drone_gazebo mission2.launch.py` |
| 4 — Start | `ros2 service call /start_mission2 std_srvs/srv/Trigger` |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `base_lat` | `0.0` | Base GPS latitude |
| `base_lon` | `0.0` | Base GPS longitude |
| `takeoff_alt` | `10.0` | Takeoff altitude (m) |
| `search_alt` | `8.0` | Search altitude (m) |

Example:
```bash
ros2 launch drone_gazebo mission2.launch.py \
    base_lat:=-35.363257 base_lon:=149.165400 takeoff_alt:=10.0 search_alt:=8.0
```

---

## Useful Reference

<details>
<summary><strong>Default Simulation Coordinates</strong></summary>

| Location | Latitude | Longitude |
|---|---|---|
| Origin | -35.3632621° | 149.1652374° |
| Base H | -35.3632621° | 149.1654° |

</details>

<details>
<summary><strong>Manual Commands</strong></summary>

> **Note:** To arm and take off, the EKF must be using GPS. A confirmation message will be printed in the SITL terminal when GPS is ready.

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
├── docker/                  # Docker build & run scripts
├── docs/                    # Documentation
├── drone_description/       # URDF/models for the drone
│   └── models/
├── drone_gazebo/            # Simulation package
│   ├── config/              # Bridge configs
│   ├── launch/              # World & mission launch files
│   ├── models/              # Gazebo models (gate, helipad, etc.)
│   ├── rviz/                # RViz configs
│   ├── src/                 # Simulation mission nodes
│   └── worlds/              # Gazebo world files
└── drone_navigate/          # Navigation & control package
    ├── drone_navigate/      # Python modules
    ├── launch/              # Navigation launch files
    ├── msg/                 # Custom message definitions
    ├── src/                 # Navigation nodes
    └── srv/                 # Custom service definitions
```
