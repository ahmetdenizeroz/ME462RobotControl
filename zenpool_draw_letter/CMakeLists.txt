cmake_minimum_required(VERSION 3.8)
project(zenpool_draw_letter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interactive_markers REQUIRED)

add_executable(zenpool_draw_letter src/zenpool_draw_letter.cpp)
add_executable(zenpool_draw_servo src/zenpool_draw_servo.cpp)
add_executable(zenpool_teleop_servo src/zenpool_teleop_servo.cpp)
target_include_directories(zenpool_draw_letter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zenpool_draw_letter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  zenpool_draw_letter
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

target_include_directories(zenpool_draw_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zenpool_draw_servo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  zenpool_draw_servo
  "moveit_ros_planning_interface"
  moveit_visual_tools
  tf2_ros
  controller_manager_msgs
  control_msgs
  std_srvs
  interactive_markers
)

ament_target_dependencies(zenpool_teleop_servo
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  interactive_markers
  std_srvs
  controller_manager_msgs
)

target_include_directories(zenpool_draw_letter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  zenpool_draw_letter
  zenpool_draw_servo
  zenpool_teleop_servo
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/face_tracker_servo.py
  scripts/aruco_tracker_servo.py
  scripts/custom_dashboard.py
  scripts/tool_manager.py
  scripts/take_one.py
  scripts/drop_one.py
  scripts/take_two.py
  scripts/drop_two.py
  scripts/take_three.py
  scripts/drop_three.py
  scripts/full_cleaning.py
  scripts/drawing_job_executor.py
  scripts/active_grippering_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
