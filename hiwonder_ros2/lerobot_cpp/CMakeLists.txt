cmake_minimum_required(VERSION 3.22)
project(lerobot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

############################################################
# Find Required Packages
############################################################

find_package(ament_cmake REQUIRED)

############################################################
# Build
############################################################

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


############################################################
# Installation
############################################################

install(
  DIRECTORY include/
  DESTINATION include
)

############################################################
# Export
############################################################

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)

############################################################
# Finalization
############################################################

ament_package()
