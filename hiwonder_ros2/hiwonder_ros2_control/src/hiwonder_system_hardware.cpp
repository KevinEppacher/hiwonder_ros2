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

#include "hiwonder_ros2_control/hiwonder_system_hardware.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hiwonder_servo_driver/registers.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace hiwonder_ros2_control
{

HiwonderSystemHardware::CallbackReturn
HiwonderSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try {
    port_ = info_.hardware_parameters.at("port");

    baud_rate_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("baud_rate")));

    calibration_file_ =
      info_.hardware_parameters.at("calibration_file");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to read hardware parameters: %s",
      exception.what());

    return CallbackReturn::ERROR;
  }

  YAML::Node calibration;

  try {
    calibration = YAML::LoadFile(calibration_file_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load calibration file '%s': %s",
      calibration_file_.c_str(),
      exception.what());

    return CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  std::unordered_set<uint8_t> servo_ids;

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

    const auto & command_interface =
      joint.command_interfaces.front();

    if (command_interface.name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' command interface must be 'position'",
        joint.name.c_str());

      return CallbackReturn::ERROR;
    }

    JointConfig config;
    config.name = joint.name;

    try {
      const auto servo_id =
        calibration_joint["servo_id"].as<unsigned int>();

      const auto lower_position =
        calibration_joint["lower_position"].as<unsigned int>();

      const auto upper_position =
        calibration_joint["upper_position"].as<unsigned int>();

      if (servo_id > std::numeric_limits<uint8_t>::max()) {
        throw std::out_of_range(
                "servo_id exceeds uint8_t range");
      }

      if (
        lower_position > std::numeric_limits<uint16_t>::max() ||
        upper_position > std::numeric_limits<uint16_t>::max())
      {
        throw std::out_of_range(
                "calibration position exceeds uint16_t range");
      }

      config.servo_id =
        static_cast<uint8_t>(servo_id);

      config.lower_position =
        static_cast<uint16_t>(lower_position);

      config.upper_position =
        static_cast<uint16_t>(upper_position);

      config.lower_limit =
        std::stod(command_interface.min);

      config.upper_limit =
        std::stod(command_interface.max);
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

    if (!servo_ids.insert(config.servo_id).second) {
      RCLCPP_ERROR(
        get_logger(),
        "Servo ID %u is assigned to multiple joints",
        static_cast<unsigned int>(config.servo_id));

      return CallbackReturn::ERROR;
    }

    joints_.push_back(config);

    RCLCPP_INFO(
      get_logger(),
      "Joint '%s': servo=%u, raw=[%u, %u], limits=[%.6f, %.6f]",
      config.name.c_str(),
      static_cast<unsigned int>(config.servo_id),
      static_cast<unsigned int>(config.lower_position),
      static_cast<unsigned int>(config.upper_position),
      config.lower_limit,
      config.upper_limit);
  }

  return CallbackReturn::SUCCESS;
}

HiwonderSystemHardware::CallbackReturn
HiwonderSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    serial_ =
      std::make_unique<hiwonder::SerialPort>(
      port_,
      baud_rate_);

    serial_->open();

    bus_ =
      std::make_unique<hiwonder::HiwonderBus>(
      *serial_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure HiWonder hardware: %s",
      exception.what());

    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "Connected to HiWonder bus on '%s' at %u baud",
    port_.c_str(),
    baud_rate_);

  return CallbackReturn::SUCCESS;
}

HiwonderSystemHardware::CallbackReturn
HiwonderSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  bus_.reset();

  if (serial_) {
    serial_->close();
    serial_.reset();
  }

  return CallbackReturn::SUCCESS;
}

HiwonderSystemHardware::CallbackReturn
HiwonderSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

HiwonderSystemHardware::CallbackReturn
HiwonderSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
HiwonderSystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!bus_) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot read hardware: bus is not configured");

    return hardware_interface::return_type::ERROR;
  }

  for (const auto & joint : joints_) {
    uint16_t raw_position = 0;

    if (!bus_->readWord(
        joint.servo_id,
        hiwonder::reg::kCurrentPosition,
        raw_position))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to read position from joint '%s' (servo %u)",
        joint.name.c_str(),
        static_cast<unsigned int>(joint.servo_id));

      return hardware_interface::return_type::ERROR;
    }

    const double raw_lower =
      static_cast<double>(joint.lower_position);

    const double raw_upper =
      static_cast<double>(joint.upper_position);

    const double normalized =
      (static_cast<double>(raw_position) - raw_lower) /
      (raw_upper - raw_lower);

    const double position =
      joint.lower_limit +
      normalized *
      (joint.upper_limit - joint.lower_limit);

    set_state(
      joint.name + "/" + hardware_interface::HW_IF_POSITION,
      position);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HiwonderSystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Position commands are intentionally not sent yet.
  // Hardware write support will be implemented after
  // position feedback has been validated.

  return hardware_interface::return_type::OK;
}

}  // namespace hiwonder_ros2_control

PLUGINLIB_EXPORT_CLASS(
  hiwonder_ros2_control::HiwonderSystemHardware,
  hardware_interface::SystemInterface)