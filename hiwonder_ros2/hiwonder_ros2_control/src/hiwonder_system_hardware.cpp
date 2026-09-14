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
#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

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
    baud_rate_ =
      static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("baud_rate")));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to read hardware parameters: %s",
      exception.what());

    return CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    JointConfig config;

    config.name = joint.name;

    try {
      config.servo_id =
        static_cast<uint8_t>(
        std::stoul(joint.parameters.at("servo_id")));

      config.offset =
        std::stod(joint.parameters.at("offset"));

      config.direction =
        std::stod(joint.parameters.at("direction"));
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to read parameters for joint '%s': %s",
        joint.name.c_str(),
        exception.what());

      return CallbackReturn::ERROR;
    }

    joints_.push_back(config);
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
  // TODO: Read current servo positions and convert ticks to radians.

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HiwonderSystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // TODO: Convert position commands from radians to ticks and use syncWrite().

  return hardware_interface::return_type::OK;
}

}  // namespace hiwonder_ros2_control

PLUGINLIB_EXPORT_CLASS(
  hiwonder_ros2_control::HiwonderSystemHardware,
  hardware_interface::SystemInterface)
