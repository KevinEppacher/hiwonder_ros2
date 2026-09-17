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

struct JointCalibration
{
  std::string name;
  uint8_t servo_id;
  uint16_t lower_position;
  uint16_t upper_position;
  double lower_limit;
  double upper_limit;
};

[[nodiscard]] double rawToPosition(const JointCalibration & joint, uint16_t raw_position);
[[nodiscard]] uint16_t positionToRaw(const JointCalibration & joint, double position);

class LeRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  LeRobotSystemHardware() = default;

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
  void guidingModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool setTorqueEnabled(bool enabled);
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
