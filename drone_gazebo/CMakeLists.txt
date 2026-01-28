cmake_minimum_required(VERSION 3.8)
project(drone_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/aruco.py
  scripts/centralized_mission.py
  scripts/generate_aruco_markers.py
  scripts/simple_mission.py
  scripts/view_camera.py
  scripts/aruco_detector_new.py
  scripts/passar_arcos_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install camera node
install(PROGRAMS
  src/camera.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_node
)

# Install mission1 wrapper
install(PROGRAMS
  src/run_mission1.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME master_mission1
)

# Install Python source modules
install(DIRECTORY
  src/passar_arcos
  src/mission1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
