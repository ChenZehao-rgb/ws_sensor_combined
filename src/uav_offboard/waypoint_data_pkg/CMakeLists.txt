cmake_minimum_required(VERSION 3.8)
project(waypoint_data_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WaypointData.msg"
  "msg/WaypointDataResponse.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 编译节点
add_executable(waypoint_receiver_node src/waypoint_receiver_node.cpp)

ament_target_dependencies(waypoint_receiver_node
  rclcpp
)

# 同一包内链接自身生成的消息，必须用此方式而非 ament_target_dependencies
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_receiver_node "${cpp_typesupport_target}")

install(TARGETS waypoint_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
