cmake_minimum_required(VERSION 3.22)
project(lerobot_ros2_control)

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
find_package(hardware_interface REQUIRED)
find_package(lerobot_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)

############################################################
# Build
############################################################

add_library(${PROJECT_NAME} SHARED
    src/lerobot_system_hardware.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    hardware_interface
    lerobot_cpp
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_srvs
)

target_link_libraries(${PROJECT_NAME}
    yaml-cpp
)

pluginlib_export_plugin_description_file(
    hardware_interface
    lerobot_ros2_control.xml
)

############################################################
# Installation
############################################################

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

############################################################
# Linting and Testing
############################################################

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_calibration
        test/test_calibration.cpp
    )

    target_link_libraries(test_calibration
        ${PROJECT_NAME}
    )
endif()

############################################################
# Export
############################################################

ament_export_targets(
    export_${PROJECT_NAME}
    HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
    hardware_interface
    lerobot_cpp
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_srvs
    yaml-cpp
)

############################################################
# Finalization
############################################################

ament_package()
