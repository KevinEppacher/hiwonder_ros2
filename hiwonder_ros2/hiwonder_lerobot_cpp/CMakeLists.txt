cmake_minimum_required(VERSION 3.22)
project(hiwonder_lerobot_cpp)

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
find_package(yaml-cpp REQUIRED)

############################################################
# Library
############################################################

add_library(${PROJECT_NAME} SHARED
    src/robot.cpp
    src/leader.cpp
    src/follower.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    hiwonder_servo_driver
)

############################################################
# Tools
############################################################

add_executable(lerobot-test-move
    tools/test_move.cpp
)

target_link_libraries(lerobot-test-move
    PRIVATE
        ${PROJECT_NAME}
)

add_executable(lerobot-torque-mode
    tools/torque_mode.cpp
)

target_link_libraries(lerobot-torque-mode
    PRIVATE
        ${PROJECT_NAME}
)

add_executable(lerobot-calibrate
    tools/calibrate.cpp
)

target_link_libraries(lerobot-calibrate
    PRIVATE
        ${PROJECT_NAME}
        yaml-cpp
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
    TARGETS
        lerobot-test-move
        lerobot-torque-mode
        lerobot-calibrate
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
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

############################################################
# Export
############################################################

ament_export_targets(
    export_${PROJECT_NAME}
    HAS_LIBRARY_TARGET
)

ament_export_dependencies(
    hiwonder_servo_driver
)

ament_export_include_directories(include)

############################################################
# Finalization
############################################################

ament_package()