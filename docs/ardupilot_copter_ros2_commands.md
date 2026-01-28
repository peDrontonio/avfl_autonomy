# Simple Gazebo Simulation Guide

**Note:** Ensure you are inside the Docker container or connected to the real drone before proceeding.

---

## Mission 1: Simple Square Pattern

### Step 1: Launch the Simulation
Start the Gazebo environment with the following command:
```bash
ros2 launch drone_gazebo simple_world.launch.py
```

### Step 2: Set Mode, Arm, and Takeoff
Prepare the drone for flight:

1. **Switch to Mode 4 (GUIDED):**
   ```bash
   ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
   ```
2. **Arm the Motors:**
   ```bash
   ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
   ```
3. **Takeoff:**
   ```bash
   ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{altitude: 10.0}"
   ```

### Step 3: Run the Mission
Execute the mission launch file:
```bash
ros2 launch drone_gazebo simple_mission.launch.py
```

---

## Mission 2: Centralized Mission (ArUco Arch Traversal)

This mission makes the drone search for an arch with ArUco markers, center itself, and pass through.

### Step 1: Launch the Simulation
Start the Gazebo environment:
```bash
ros2 launch drone_gazebo simple_world.launch.py
```

### Step 2: Manual Takeoff
Arm and takeoff manually:

1. **Switch to Mode 4 (GUIDED):**
   ```bash
   ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
   ```
2. **Arm the Motors:**
   ```bash
   ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
   ```
3. **Takeoff to appropriate height:**
   ```bash
   ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{altitude: 10.0}"
   ```

### Step 3: Launch the Centralized Mission
After the drone has stabilized at the target altitude:
```bash
ros2 launch drone_gazebo centralized_mission.launch.py
```

### Mission Parameters
You can customize the mission behavior:
```bash
ros2 launch drone_gazebo centralized_mission.launch.py \
    search_velocity:=1.0 \
    center_velocity:=0.5 \
    forward_velocity:=1.0 \
    search_distance:=10.0 \
    center_tolerance:=0.1 \
    forward_distance:=5.0
```

### Mission Behavior
1. **Search Phase**: Drone moves right for 10m, then left for 10m, repeating until ArUcos are found
2. **Center X**: Adjusts horizontal position to align with arch center
3. **Center Y**: Adjusts vertical position to align with arch center  
4. **Forward**: Moves forward through the arch

### ArUco Configuration
The arch has 4 ArUco markers (IDs 0-3) at its corners:
- ID 0: Top-left
- ID 1: Top-right
- ID 2: Bottom-left
- ID 3: Bottom-right
