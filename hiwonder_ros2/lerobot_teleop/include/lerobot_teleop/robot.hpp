#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace lerobot_teleop
{

class Robot
{
public:
  Robot(
    rclcpp_lifecycle::LifecycleNode & node,
    std::string joint_states_topic);

  virtual ~Robot() = default;

  [[nodiscard]] bool stateReceived() const noexcept;

  [[nodiscard]] const sensor_msgs::msg::JointState &
  jointState() const noexcept;

  [[nodiscard]] bool positions(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions) const;

  [[nodiscard]] double stateAge() const;

protected:
  virtual void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  std::string joint_states_topic_;

  sensor_msgs::msg::JointState joint_state_;

  bool state_received_{false};

  std::chrono::steady_clock::time_point
    last_update_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace lerobot_teleop
