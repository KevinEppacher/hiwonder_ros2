cmake_minimum_required(VERSION 3.22)
project(lerobot_bringup)

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
find_package(hiwonder_servo_driver REQUIRED)

############################################################
# Build
############################################################

############################################################
# Installation
############################################################

install(
    DIRECTORY include/
    DESTINATION include
)

############################################################
# Linting and Testing
############################################################

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

############################################################
# Export
############################################################

############################################################
# Finalization
############################################################

ament_package()
