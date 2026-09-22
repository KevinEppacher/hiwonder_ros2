// Copyright 2026 Kevin Eppacher
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lerobot_cpp/robot.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_ros2_control
{

/**
 * @brief Calibration data for mapping raw motor positions to joint positions.
 */
struct JointCalibration
{
    std::string name;
    uint16_t lower_position;
    uint16_t upper_position;
    double lower_limit;
    double upper_limit;
};

/**
 * @brief Converts a raw motor position to a calibrated joint position.
 *
 * @param joint Joint calibration data.
 * @param raw_position Raw motor position.
 * @return Joint position in radians.
 */
[[nodiscard]] double rawToPosition(
    const JointCalibration & joint,
    uint16_t raw_position);

/**
 * @brief Converts a joint position to a raw motor position.
 *
 * @param joint Joint calibration data.
 * @param position Joint position in radians.
 * @return Raw motor position.
 */
[[nodiscard]] uint16_t positionToRaw(
    const JointCalibration & joint,
    double position);

/**
 * @brief Provides a ros2_control system interface for LeRobot hardware.
 */
class LeRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  LeRobotSystemHardware() = default;

  /**
   * @brief Initializes the hardware interface from ros2_control parameters.
   *
   * @param params Hardware component parameters.
   * @return Lifecycle callback result.
   */
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /**
   * @brief Configures the robot backend and establishes the hardware connection.
   *
   * @param previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleans up resources used by the hardware interface.
   *
   * @param previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activates the hardware interface.
   *
   * @param previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivates the hardware interface.
   *
   * @param previous_state Previous lifecycle state.
   * @return Lifecycle callback result.
   */
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Reads motor positions and updates the joint states.
   *
   * @param time Current control loop time.
   * @param period Time elapsed since the previous update.
   * @return OK on success, otherwise ERROR.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Writes joint commands to the robot hardware.
   *
   * @param time Current control loop time.
   * @param period Time elapsed since the previous update.
   * @return OK on success, otherwise ERROR.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  /**
   * @brief Handles requests to enable or disable guiding mode.
   *
   * @param request Guiding mode request.
   * @param response Service response.
   */
  void guidingModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Enables or disables torque through the robot backend.
   *
   * @param enabled True to enable torque, false to disable it.
   * @return True if the operation succeeds, otherwise false.
   */
  bool setTorqueEnabled(
    bool enabled);

  /**
   * @brief Synchronizes joint commands with the current measured positions.
   *
   * @return True if synchronization succeeds, otherwise false.
   */
  bool synchronizeCommandsWithCurrentPosition();

  std::string robot_plugin_;
  std::string port_;
  std::string calibration_file_;
  uint32_t baud_rate_{1000000};
  std::vector<JointCalibration> joints_;

  std::unique_ptr<pluginlib::ClassLoader<lerobot::Robot>> robot_loader_;
  pluginlib::UniquePtr<lerobot::Robot> robot_;

  std::atomic_bool guiding_mode_requested_{false};
  bool guiding_mode_active_{false};

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr guiding_mode_service_;
};

}  // namespace lerobot_ros2_control
