# Simple Gazebo Simulation Guide

**Note:** Ensure you are inside the Docker container or connected to the real drone before proceeding.

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
