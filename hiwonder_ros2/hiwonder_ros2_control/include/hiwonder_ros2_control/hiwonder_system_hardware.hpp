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

#ifndef HIWONDER_ROS2_CONTROL__HIWONDER_SYSTEM_HARDWARE_HPP_
#define HIWONDER_ROS2_CONTROL__HIWONDER_SYSTEM_HARDWARE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"

namespace hiwonder_ros2_control
{

class HiwonderSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  HiwonderSystemHardware() = default;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct JointConfig
  {
    std::string name;

    uint8_t servo_id;

    uint16_t lower_position;
    uint16_t upper_position;

    double lower_limit;
    double upper_limit;
  };

  [[nodiscard]] double rawToPosition(
    const JointConfig & joint,
    uint16_t raw_position) const;

  [[nodiscard]] uint16_t positionToRaw(
    const JointConfig & joint,
    double position) const;

  void guidingModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool setTorqueEnabled(bool enabled);

  bool synchronizeCommandsWithCurrentPosition();

  std::string port_;
  std::string calibration_file_;

  uint32_t baud_rate_{1000000};

  std::vector<JointConfig> joints_;

  std::unique_ptr<hiwonder::SerialPort> serial_;
  std::unique_ptr<hiwonder::HiwonderBus> bus_;

  std::atomic_bool guiding_mode_requested_{false};
  bool guiding_mode_active_{false};

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    guiding_mode_service_;
};

}  // namespace hiwonder_ros2_control

#endif  // HIWONDER_ROS2_CONTROL__HIWONDER_SYSTEM_HARDWARE_HPP_