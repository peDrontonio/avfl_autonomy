cmake_minimum_required(VERSION 3.8)
project(drone_navigate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ardupilot_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate service interfaces and messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Navigate.srv"
  "srv/GetTelemetry.srv"
  "srv/NavigateGlobal.srv"
  "srv/SetYawRate.srv"
  "msg/BaseDetection.msg"
  DEPENDENCIES geometry_msgs ardupilot_msgs std_msgs
)


install(PROGRAMS
  src/mission1/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_node
)

install(PROGRAMS
  drone_navigate/telemetry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME telemetry_node
)

install(PROGRAMS
  drone_navigate/navigate_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigate_service
)

install(PROGRAMS
  drone_navigate/navigate_global_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigate_global_service
)

install(PROGRAMS
  drone_navigate/set_yaw_rate_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_yaw_rate_service
)

# Install Python source modules (mission1 package)
install(DIRECTORY
  src/mission1
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING 
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install mission1 master node
install(PROGRAMS
  src/mission1/master_mission1.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME master_mission1
)

# Install mission2 master node
install(PROGRAMS
  src/mission2/master_mission2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME master_mission2
)

# Install mission2 yolo detector node
install(PROGRAMS
  src/mission2/yolo_base_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_base_detector
)

# Install Python source modules (mission2 package)
install(DIRECTORY
  src/mission2
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING 
  PATTERN "*.py"
  PATTERN "*.pt"
  PATTERN "__pycache__" EXCLUDE
)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
