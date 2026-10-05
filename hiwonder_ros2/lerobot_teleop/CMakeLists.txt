cmake_minimum_required(VERSION 3.8)
project(lerobot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teleop_node
  src/teleop_node.cpp
)

target_compile_features(teleop_node PUBLIC cxx_std_20)

ament_target_dependencies(
  teleop_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
