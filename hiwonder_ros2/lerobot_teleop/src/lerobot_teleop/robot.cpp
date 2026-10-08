#include "lerobot_teleop/robot.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <utility>

namespace lerobot_teleop
{

Robot::Robot(
  rclcpp_lifecycle::LifecycleNode & node,
  std::string joint_states_topic)
: joint_states_topic_(std::move(joint_states_topic))
{
  subscription_ =
    node.create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &Robot::jointStateCallback,
      this,
      std::placeholders::_1));
}

bool Robot::stateReceived() const noexcept
{
  return state_received_;
}

const sensor_msgs::msg::JointState &
Robot::jointState() const noexcept
{
  return joint_state_;
}

bool Robot::positions(
  const std::vector<std::string> & joint_names,
  std::vector<double> & positions) const
{
  if (!state_received_) {
    return false;
  }

  positions.clear();
  positions.reserve(joint_names.size());

  for (const auto & joint_name : joint_names) {
    const auto it =
      std::find(
      joint_state_.name.begin(),
      joint_state_.name.end(),
      joint_name);

    if (it == joint_state_.name.end()) {
      positions.clear();
      return false;
    }

    const auto index =
      static_cast<std::size_t>(
      std::distance(
        joint_state_.name.begin(),
        it));

    if (index >= joint_state_.position.size()) {
      positions.clear();
      return false;
    }

    positions.push_back(
      joint_state_.position[index]);
  }

  return true;
}

double Robot::stateAge() const
{
  if (!state_received_) {
    return std::numeric_limits<double>::infinity();
  }

  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    last_update_).count();
}

void Robot::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (
    msg->name.empty() ||
    msg->name.size() != msg->position.size())
  {
    return;
  }

  joint_state_ = *msg;

  state_received_ = true;

  last_update_ =
    std::chrono::steady_clock::now();
}

}  // namespace lerobot_teleop
