cmake_minimum_required(VERSION 3.8)
project(traj_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ruckig (and <format>) requires C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)

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


# Generate interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
  "srv/GetTrajectorySetpoint.srv"
  DEPENDENCIES
    geometry_msgs
    px4_msgs
)

add_executable(online_traj_node src/online_traj_generator.cpp)
ament_target_dependencies(online_traj_node
  rclcpp
  sensor_msgs
  geometry_msgs
  px4_msgs
  tf2
)
add_dependencies(online_traj_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(online_traj_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(online_traj_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Offboard control bridge node
add_executable(offboard_control_bridge_node src/offboard_control.cpp)
ament_target_dependencies(offboard_control_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  px4_msgs
  tf2
)
target_include_directories(offboard_control_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_dependencies(offboard_control_bridge_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(offboard_control_bridge_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS online_traj_node offboard_control_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
