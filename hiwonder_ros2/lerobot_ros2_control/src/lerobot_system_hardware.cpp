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

#include "lerobot_ros2_control/lerobot_system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace lerobot_ros2_control
{

double rawToPosition(const JointCalibration & joint, uint16_t raw_position)
{
  const double raw_lower = static_cast<double>(joint.lower_position);
  const double raw_upper = static_cast<double>(joint.upper_position);

  if (raw_upper == raw_lower) {
    throw std::invalid_argument("Calibration range is zero");
  }

  const double normalized =
    (static_cast<double>(raw_position) - raw_lower) /
    (raw_upper - raw_lower);

  return joint.lower_limit + normalized * (joint.upper_limit - joint.lower_limit);
}

uint16_t positionToRaw(const JointCalibration & joint, double position)
{
  const double clamped_position =
    std::clamp(position, joint.lower_limit, joint.upper_limit);

  const double normalized =
    (clamped_position - joint.lower_limit) /
    (joint.upper_limit - joint.lower_limit);

  const double raw =
    static_cast<double>(joint.lower_position) +
    normalized *
    (static_cast<double>(joint.upper_position) - static_cast<double>(joint.lower_position));

  const double raw_min =
    static_cast<double>(std::min(joint.lower_position, joint.upper_position));
  const double raw_max =
    static_cast<double>(std::max(joint.lower_position, joint.upper_position));

  return static_cast<uint16_t>(std::lround(std::clamp(raw, raw_min, raw_max)));
}

LeRobotSystemHardware::CallbackReturn
LeRobotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try {
    robot_plugin_ = info_.hardware_parameters.at("robot_plugin");
    port_ = info_.hardware_parameters.at("port");
    baud_rate_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("baud_rate")));
    calibration_file_ = info_.hardware_parameters.at("calibration_file");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Failed to read hardware parameters: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  YAML::Node calibration;
  try {
    calibration = YAML::LoadFile(calibration_file_);
  } catch (const std::exception & exception) {
    if (robot_) {
      robot_->disconnect();
    }
    robot_.reset();
    robot_loader_.reset();
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load calibration file '%s': %s",
      calibration_file_.c_str(),
      exception.what());
    return CallbackReturn::ERROR;
  }

joints_.clear();
joints_.reserve(info_.joints.size());

for (const auto & joint : info_.joints) {
    const YAML::Node calibration_joint = calibration[joint.name];
    if (!calibration_joint) {
        RCLCPP_ERROR(
            get_logger(),
            "Calibration for joint '%s' is missing",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != 1) {
        RCLCPP_ERROR(
            get_logger(),
            "Joint '%s' must have exactly one command interface",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
        RCLCPP_ERROR(
            get_logger(),
            "Joint '%s' must have exactly one state interface",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    const auto & command_interface = joint.command_interfaces.front();
    if (command_interface.name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
            get_logger(),
            "Joint '%s' command interface must be 'position'",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.front().name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
            get_logger(),
            "Joint '%s' state interface must be 'position'",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    JointCalibration config;
    config.name = joint.name;

    try {
        const auto lower_position =
            calibration_joint["lower_position"].as<unsigned int>();
        const auto upper_position =
            calibration_joint["upper_position"].as<unsigned int>();

        if (lower_position > std::numeric_limits<uint16_t>::max() ||
            upper_position > std::numeric_limits<uint16_t>::max())
        {
            throw std::out_of_range(
                "calibration position exceeds uint16_t range");
        }

        config.lower_position = static_cast<uint16_t>(lower_position);
        config.upper_position = static_cast<uint16_t>(upper_position);
        config.lower_limit = std::stod(command_interface.min);
        config.upper_limit = std::stod(command_interface.max);
    } catch (const std::exception & exception) {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid configuration for joint '%s': %s",
            joint.name.c_str(),
            exception.what());
        return CallbackReturn::ERROR;
    }

    if (config.lower_position == config.upper_position) {
        RCLCPP_ERROR(
            get_logger(),
            "Calibration range for joint '%s' is zero",
            joint.name.c_str());
        return CallbackReturn::ERROR;
    }

    if (config.lower_limit >= config.upper_limit) {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid position limits for joint '%s': %.6f >= %.6f",
            joint.name.c_str(),
            config.lower_limit,
            config.upper_limit);
        return CallbackReturn::ERROR;
    }

    joints_.push_back(config);
  }

  if (!get_node()) {
    RCLCPP_ERROR(get_logger(), "Framework-managed hardware node is not available");
    return CallbackReturn::ERROR;
  }

  guiding_mode_service_ =
    get_node()->create_service<std_srvs::srv::SetBool>(
      "~/guiding_mode",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      { guidingModeCallback(request, response); });

  return CallbackReturn::SUCCESS;
}

LeRobotSystemHardware::CallbackReturn
LeRobotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    robot_loader_ = std::make_unique<pluginlib::ClassLoader<lerobot::Robot>>(
      "lerobot_cpp",
      "lerobot::Robot");
    robot_ = robot_loader_->createUniqueInstance(robot_plugin_);

    lerobot::RobotConfig config;
    config.port = port_;
    config.baud_rate = baud_rate_;

    if (!robot_->configure(config)) {
      throw std::runtime_error("failed to configure robot plugin");
    }

    if (!robot_->connect()) {
      throw std::runtime_error("failed to connect robot plugin");
    }

    if (robot_->motorCount() != joints_.size()) {
      throw std::runtime_error("robot motor count does not match configured joints");
    }
    } catch (const std::exception & exception) {
        if (robot_) {
            robot_->disconnect();
        }
        robot_.reset();
        robot_loader_.reset();

        RCLCPP_ERROR(
            get_logger(),
            "Failed to configure robot plugin '%s': %s",
            robot_plugin_.c_str(),
            exception.what());
        return CallbackReturn::ERROR;
    }

  return CallbackReturn::SUCCESS;
}

LeRobotSystemHardware::CallbackReturn
LeRobotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (robot_) {
    robot_->disconnect();
  }
  robot_.reset();
  robot_loader_.reset();
  return CallbackReturn::SUCCESS;
}

LeRobotSystemHardware::CallbackReturn
LeRobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!robot_ || !robot_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate hardware: robot is not configured");
    return CallbackReturn::ERROR;
  }

  if (!synchronizeCommandsWithCurrentPosition()) {
      return CallbackReturn::ERROR;
  }

  if (!setTorqueEnabled(true)) {
      RCLCPP_ERROR(
          get_logger(),
          "Cannot activate hardware: failed to enable motor torque");
      return CallbackReturn::ERROR;
  }

  guiding_mode_requested_.store(false);
  guiding_mode_active_ = false;

  return CallbackReturn::SUCCESS;
}

LeRobotSystemHardware::CallbackReturn
LeRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!robot_ || !robot_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Cannot deactivate hardware: robot is not configured");
    return CallbackReturn::ERROR;
  }

  if (!setTorqueEnabled(false)) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot deactivate hardware: failed to disable motor torque");
    return CallbackReturn::ERROR;
  }

  guiding_mode_requested_.store(false);
  guiding_mode_active_ = false;

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
LeRobotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!robot_ || !robot_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Cannot read hardware: robot is not configured");
    return hardware_interface::return_type::ERROR;
  }

  std::vector<uint16_t> raw_positions;
  if (!robot_->readPositions(raw_positions)) {
    RCLCPP_ERROR(get_logger(), "Failed to read positions from robot");
    return hardware_interface::return_type::ERROR;
  }

  if (raw_positions.size() != joints_.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "Robot returned %zu positions but hardware expects %zu",
      raw_positions.size(),
      joints_.size());
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto position = rawToPosition(joints_[i], raw_positions[i]);
    set_state(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, position);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
LeRobotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!robot_ || !robot_->isConnected()) {
    RCLCPP_ERROR(get_logger(), "Cannot write hardware: robot is not configured");
    return hardware_interface::return_type::ERROR;
  }

  const bool guiding_requested = guiding_mode_requested_.load();
  if (guiding_requested != guiding_mode_active_) {
    if (guiding_requested) {
      if (!setTorqueEnabled(false)) {
        return hardware_interface::return_type::ERROR;
      }
      guiding_mode_active_ = true;
      RCLCPP_INFO(get_logger(), "Guiding mode enabled");
    } else {
      if (!synchronizeCommandsWithCurrentPosition()) {
        return hardware_interface::return_type::ERROR;
      }
      if (!setTorqueEnabled(true)) {
        return hardware_interface::return_type::ERROR;
      }
      guiding_mode_active_ = false;
      RCLCPP_INFO(get_logger(), "Guiding mode disabled");
    }
  }

  if (guiding_mode_active_) {
    return hardware_interface::return_type::OK;
  }

  std::vector<uint16_t> raw_positions;
  raw_positions.reserve(joints_.size());

  for (const auto & joint : joints_) {
    const double command =
      get_command(joint.name + "/" + hardware_interface::HW_IF_POSITION);

    if (!std::isfinite(command)) {
      RCLCPP_ERROR(get_logger(), "Invalid position command for joint '%s'", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    raw_positions.push_back(positionToRaw(joint, command));
  }

  if (!robot_->writePositions(raw_positions)) {
    RCLCPP_ERROR(get_logger(), "Failed to write synchronized joint positions");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void LeRobotSystemHardware::guidingModeCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  guiding_mode_requested_.store(request->data);
  response->success = true;
  response->message = request->data ? "Guiding mode requested" : "Guiding mode disable requested";
}

bool LeRobotSystemHardware::setTorqueEnabled(bool enabled)
{
  if (!robot_) {
    return false;
  }
  return robot_->setTorqueEnabled(enabled);
}

bool LeRobotSystemHardware::synchronizeCommandsWithCurrentPosition()
{
  if (!robot_ || !robot_->isConnected()) {
    return false;
  }

  std::vector<uint16_t> raw_positions;
  if (!robot_->readPositions(raw_positions)) {
    return false;
  }

  if (raw_positions.size() != joints_.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "Robot returned %zu positions but hardware expects %zu",
      raw_positions.size(),
      joints_.size());
    return false;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double position = rawToPosition(joints_[i], raw_positions[i]);
    set_state(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, position);
    set_command(joints_[i].name + "/" + hardware_interface::HW_IF_POSITION, position);
  }

  return true;
}

}  // namespace lerobot_ros2_control

PLUGINLIB_EXPORT_CLASS(
  lerobot_ros2_control::LeRobotSystemHardware,
  hardware_interface::SystemInterface)
