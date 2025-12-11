cmake_minimum_required(VERSION 3.8)
project(rapid_trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

add_library(rapid_trajectory_generator SHARED
  src/RapidTrajectoryGenerator.cpp
  src/SingleAxisTrajectory.cpp
)
target_include_directories(rapid_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rapid_trajectory_generator PUBLIC cxx_std_14)

add_executable(rapid_trajectory_test_node
  src/rapid_trajectory_test_node.cpp
)
target_compile_features(rapid_trajectory_test_node PUBLIC cxx_std_14)
ament_target_dependencies(rapid_trajectory_test_node
  rclcpp
)
target_link_libraries(rapid_trajectory_test_node
  rapid_trajectory_generator
)

install(TARGETS rapid_trajectory_generator
  EXPORT export_rapid_trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS rapid_trajectory_test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_rapid_trajectory_generator HAS_LIBRARY_TARGET)
ament_export_dependencies()

ament_package()
