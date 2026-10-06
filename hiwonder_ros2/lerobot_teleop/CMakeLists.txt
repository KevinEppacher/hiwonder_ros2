cmake_minimum_required(VERSION 3.22)
project(lerobot_teleop)

# Set CMake policy for modern find_package behavior
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################
# Find Required Packages
############################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############################################################
# Declare Executables
############################################################

# Leader Follower Teleop Node

add_executable(leader_follower_teleop
  src/leader_follower_teleop_node.cpp
  src/lerobot_teleop/leader_follower_teleop.cpp
)

target_include_directories(leader_follower_teleop
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(leader_follower_teleop
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
)

target_compile_features(leader_follower_teleop
  PUBLIC
    cxx_std_20
)

############################################################
# Installation
############################################################

install(
  TARGETS leader_follower_teleop
  DESTINATION lib/${PROJECT_NAME}
)

############################################################
# Linting and Testing
############################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

############################################################
# Finalization
############################################################

install(DIRECTORY
  include
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
