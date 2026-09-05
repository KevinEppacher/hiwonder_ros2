cmake_minimum_required(VERSION 3.22)
project(hiwonder_servo_driver)

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
# Declare Executables
############################################################

add_executable(hiwonder-find-port
    tools/find_port.cpp
)

############################################################
# Installation
############################################################

install(
    TARGETS hiwonder-find-port
    DESTINATION bin
)

############################################################
# Headers
############################################################

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

############################################################
# Finalization
############################################################

ament_package()