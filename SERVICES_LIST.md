ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS1 -b 921600 -v6

ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 1.5}"

ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"

ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"

ros2 service call /ap/prearm_check std_srvs/srv/Trigger

ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

ros2 service call /avfl/navigate drone_navigate/srv/Navigate "{x: 0.0, y: 0.0, z: 2.0, speed: 0.5, frame_id: 'map', auto_arm: true}"

ros2 service call /avfl/navigate_global drone_navigate/srv/NavigateGlobal   "{lat: -35.363661, lon: 149.166230, z: 10.0, yaw: 0.0, speed: 2.0, frame_id: 'rel', auto_arm: true}"