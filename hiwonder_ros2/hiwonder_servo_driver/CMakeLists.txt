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
# Driver Library
############################################################

add_library(${PROJECT_NAME} SHARED
    src/serial_port.cpp
    src/hiwonder_bus.cpp
    src/servo.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

############################################################
# Tools
############################################################

add_executable(hiwonder-find-port
    tools/find_port.cpp
)

add_executable(hiwonder-scan
    tools/scan.cpp
)

target_link_libraries(hiwonder-scan
    PRIVATE
        ${PROJECT_NAME}
)

add_executable(hiwonder-setup-motors
    tools/setup_motors.cpp
)

target_link_libraries(hiwonder-setup-motors
    PRIVATE
        ${PROJECT_NAME}
)

add_executable(hiwonder-test-move
    tools/test_move.cpp
)

target_link_libraries(hiwonder-test-move
    PRIVATE
        ${PROJECT_NAME}
)

add_executable(hiwonder-torque-mode
    tools/torque_mode.cpp
)

target_link_libraries(hiwonder-torque-mode
    PRIVATE
        ${PROJECT_NAME}
)

############################################################
# Installation
############################################################

install(
    TARGETS
        ${PROJECT_NAME}
        hiwonder-find-port
        hiwonder-scan
        hiwonder-setup-motors
        hiwonder-test-move
        hiwonder-torque-mode
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

    # ------------------------------------------------------------
    # Unit Tests for test_checksum
    # ------------------------------------------------------------

    ament_add_gtest(test_checksum
        test/test_checksum.cpp
    )

    target_link_libraries(test_checksum
        ${PROJECT_NAME}
    )

    # ------------------------------------------------------------
    # Unit Tests for test_hiwonder_bus
    # ------------------------------------------------------------

    ament_add_gtest(test_hiwonder_bus
        test/test_hiwonder_bus.cpp
    )

    target_link_libraries(test_hiwonder_bus
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

############################################################
# Finalization
############################################################

ament_package()
