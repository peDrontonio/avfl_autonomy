# Mission 3: Mobile Base Landing with GPS Navigation (YOLO)

## Overview
Mission 3 implements autonomous landing on a mobile base using GPS navigation and **YOLO-based** computer vision detection. This is a backup/copy of the original Mission 2 before it was migrated to color filter detection.

## State Machine Flow
```
GoingToBase → Search_Base → Scan_for_Base → Landing
     ↓              ↓              ↓              ↓
  GPS Nav      Search with    Re-acquire      Land on
  to Base       Camera          Base           Base
     ↓              ↓              ↓
  (timeout)    (max retries)  (max retries)
     ↓              ↓              ↓
              ReturnBase ← ← ← ← ← (RTL safety)
```

### States Description:

1. **GoingToBase**: Navigate to base area using GPS coordinates (lat/lon)
2. **Search_Base**: Search for mobile base using downward camera and YOLO detection
3. **Scan_for_Base**: If base is lost, return to last known position and reacquire
4. **Landing**: Execute landing when base is properly aligned
5. **ReturnBase**: Safety RTL if base is not found after max retries

## Configuration

### Required Parameters
Set these parameters when launching the mission:

```bash
python3 master_mission3.py --ros-args \
  -p base_lat:=<LATITUDE> \
  -p base_lon:=<LONGITUDE> \
  -p base_alt:=<ALTITUDE_METERS>
```

**Example (simple_world helipad at pose x=15, y=0):**
```bash
python3 master_mission3.py --ros-args \
  -p base_lat:=-35.3631272 \
  -p base_lon:=149.1652374 \
  -p base_alt:=10.0
```

### Parameters:
- `base_lat`: GPS latitude of base location (degrees)
- `base_lon`: GPS longitude of base location (degrees)  
- `base_alt`: Flight altitude to reach base area (meters above ground)

### GPS Reference (simple_world.sdf)
| Base | Posição Gazebo | Latitude | Longitude |
|------|---------------|----------|-----------|
| Helipad (h_base) | `(15, 0)` | `-35.3631272` | `149.1652374` |

> Origin: Lat `-35.3632621`, Lon `149.1652374`

## Usage

### Method 1: Run nodes separately

**Terminal 1 - YOLO Detector:**
```bash
cd ~/intership_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission3/yolo_base_detector.py
```

**Terminal 2 - Mission Controller:**
```bash
cd ~/intership_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission3/master_mission3.py \
  --ros-args -p base_lat:=-35.3631272 -p base_lon:=149.1652374 -p base_alt:=10.0
```

**Terminal 3 - Start Mission:**
```bash
ros2 service call /start_mission3 std_srvs/srv/Trigger
```

### Method 2: Use launch script

```bash
cd ~/intership_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission3/run_mission3.py
```

Then trigger the mission:
```bash
ros2 service call /start_mission3 std_srvs/srv/Trigger
```

## Services

### Start Mission
```bash
ros2 service call /start_mission3 std_srvs/srv/Trigger
```

**Response:** Returns success status and completion message

## Topics

### Published by YOLO Detector:
- `/yolo/base_detection` (drone_navigate/msg/BaseDetection) - Base detection data
- `/yolo/base_detection/image` (sensor_msgs/msg/Image) - Annotated image with detections

### Subscribed:
- `/bottom_camera` (sensor_msgs/msg/Image) - Bottom camera feed for base detection

## Dependencies

- **ROS 2 Humble**
- **drone_navigate** package (Navigate, NavigateGlobal, GetTelemetry services)
- **Ultralytics YOLO** (YOLOv8)
- **OpenCV** with cv_bridge
- **PyTorch**
- **colorful** (terminal colors)

## YOLO Model

Place your trained YOLO model at:
```
mission3/yolo/modelo_fumaca.pt
```

The model should be trained to detect the mobile base (class 0).

## Key Features

1. **GPS-Based Navigation**: Uses NavigateGlobal service for lat/lon navigation
2. **Computer Vision Tracking**: YOLO-based detection for mobile base
3. **Robust State Machine**: Handles base loss and reacquisition with RTL safety
4. **Moving Target Landing**: Capable of landing on mobile platforms
5. **Configurable Parameters**: Easy mission configuration via ROS parameters

## File Structure

```
mission3/
├── FSM.py                  # Finite State Machine base class
├── states.py               # Mission states (GoingToBase, Search_Base, etc.)
├── tools_mission3.py       # Navigation tools and detection callbacks
├── master_mission3.py      # Main mission controller (MasterMission3)
├── run_mission3.py         # Launch script (starts YOLO + controller)
├── yolo_base_detector.py   # YOLO-based base detector node
├── nav_diagnostic.py       # Navigation diagnostic tool
├── README.md               # This file
└── yolo/
    └── modelo_fumaca.pt    # YOLO model weights
```

## Troubleshooting

### Base never detected
- Check bottom camera topic: `ros2 topic echo /bottom_camera`
- Verify YOLO model exists at `mission3/yolo/modelo_fumaca.pt`
- Check camera is publishing images: `ros2 topic hz /bottom_camera`
- Check detection topic: `ros2 topic echo /yolo/base_detection`

### GPS navigation fails
- Ensure navigate_global service is running
- Check GPS coordinates are valid and within simulation bounds
- Verify `base_lat`, `base_lon`, `base_alt` parameters are set
- Run diagnostic: `python3 nav_diagnostic.py`

### Landing issues  
- Adjust tolerance in `is_base_centered()` method in `tools_mission3.py`
- Tune camera focal lengths (fx, fy) for your camera
- Check consecutive detection thresholds

## Camera Calibration

If landing accuracy is poor, calibrate camera focal lengths:

Edit in `tools_mission3.py`:
```python
self.fx = 867.5579087775083  # Horizontal focal length
self.fy = 868.4321850250884  # Vertical focal length
```

## Relationship with Mission 2

Mission 3 is a **backup copy** of Mission 2 before it was refactored. Mission 2 was migrated from YOLO detection to **color filter (HSV) detection**, while Mission 3 preserves the original YOLO-based approach.

| | Mission 2 | Mission 3 |
|---|---|---|
| **Detection** | Color filter (HSV) | YOLO |
| **Service** | `/start_mission2` | `/start_mission3` |
| **Detection topic** | `/color/base_detection` | `/yolo/base_detection` |
| **Dependencies** | OpenCV only | Ultralytics + PyTorch |
