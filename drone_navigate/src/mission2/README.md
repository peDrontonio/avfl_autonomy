# Mission 2: Mobile Base Landing with GPS Navigation

## Overview
Mission 2 implements autonomous landing on a mobile base using GPS navigation and YOLO-based computer vision detection.

## State Machine Flow
```
GoingToBase → Search_Base → Scan_for_Base → Descent → Landing
     ↓              ↓              ↓            ↓         ↓
  GPS Nav      Search with    Re-acquire   Descend    Land on
  to Base         Camera         Base       to 0.8m    Moving Base
```

### States Description:

1. **GoingToBase**: Navigate to base area using GPS coordinates (lat/lon)
2. **Search_Base**: Search for mobile base using downward camera and YOLO detection
3. **Scan_for_Base**: If base is lost, return to last known position and reacquire
4. **Descent**: Descend to landing altitude while maintaining position over base
5. **Landing**: Execute landing when base is properly aligned

## Configuration

### Required Parameters
Set these parameters when launching the mission:

```bash
ros2 run python3 master_mission2.py --ros-args \
  -p base_lat:=<LATITUDE> \
  -p base_lon:=<LONGITUDE> \
  -p base_alt:=<ALTITUDE_METERS>
```

**Example:**
```bash
ros2 run python3 master_mission2.py --ros-args \
  -p base_lat:=-22.0058 \
  -p base_lon:=-47.8978 \
  -p base_alt:=10.0
```

### Parameters:
- `base_lat`: GPS latitude of base location (degrees)
- `base_lon`: GPS longitude of base location (degrees)  
- `base_alt`: Flight altitude to reach base area (meters above ground)

## Usage

### Method 1: Run nodes separately

**Terminal 1 - YOLO Detector:**
```bash
cd ~/internship_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission2/yolo_base_detector.py
```

**Terminal 2 - Mission Controller:**
```bash
cd ~/internship_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission2/master_mission2.py \
  --ros-args -p base_lat:=-22.0058 -p base_lon:=-47.8978 -p base_alt:=10.0
```

**Terminal 3 - Start Mission:**
```bash
ros2 service call /start_mission2 std_srvs/srv/Trigger
```

### Method 2: Use launch script

```bash
cd ~/internship_ws
source install/setup.bash
python3 src/avfl_autonomy/drone_gazebo/src/mission2/run_mission2.py \
  --ros-args -p base_lat:=-22.0058 -p base_lon:=-47.8978 -p base_alt:=10.0
```

## Services

### Start Mission
```bash
ros2 service call /start_mission2 std_srvs/srv/Trigger
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
mission2/yolo/modelo_fumaca.pt
```

The model should be trained to detect the mobile base (class 0).

## Key Features

1. **GPS-Based Navigation**: Uses NavigateGlobal service for lat/lon navigation
2. **Computer Vision Tracking**: YOLO-based detection for mobile base
3. **Robust State Machine**: Handles base loss and reacquisition
4. **Moving Target Landing**: Capable of landing on mobile platforms
5. **Configurable Parameters**: Easy mission configuration via ROS parameters

## Troubleshooting

### Base never detected
- Check bottom camera topic: `ros2 topic echo /bottom_camera`
- Verify YOLO model path and model file exists
- Check camera is publishing images: `ros2 topic hz /bottom_camera`

### GPS navigation fails
- Ensure navigate_global service is running
- Check GPS coordinates are valid and within simulation bounds
- Verify base_lat, base_lon, base_alt parameters are set

### Descent/Landing issues  
- Adjust tolerance in `is_base_centered()` method
- Tune camera focal lengths (fx, fy) for your camera
- Check consecutive detection thresholds

## Camera Calibration

If landing accuracy is poor, calibrate camera focal lengths:

Edit in `tools_mission2.py`:
```python
self.fx = 867.5579087775083  # Horizontal focal length
self.fy = 868.4321850250884  # Vertical focal length
```

## Migration from Mission 4 / ROS1

This mission was converted from ROS1 Noetic (Clover) to ROS2 Humble:

**Changes:**
- ❌ Removed quadrant-based navigation
- ✅ Added GPS-based NavigateGlobal
- ✅ Updated FSM: Obstacle_Avoidance → GoingToBase
- ✅ All services now use ROS2 async pattern
- ✅ Logging uses `get_logger()` instead of `rospy.log*`
