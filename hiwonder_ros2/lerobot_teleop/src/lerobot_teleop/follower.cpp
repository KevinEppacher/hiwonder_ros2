#include "lerobot_teleop/follower.hpp"

#include <algorithm>

namespace lerobot_teleop
{

Follower::Follower(
  rclcpp_lifecycle::LifecycleNode & node,
  const FollowerParameters & parameters)
: Robot(
    node,
    parameters.joint_states_topic),
  joints_(parameters.joints),
  alignment_duration_(parameters.alignment_duration)
{
  command_publisher_ =
    node.create_publisher<
    std_msgs::msg::Float64MultiArray>(
    parameters.command_topic,
    10);
}

void Follower::startAlignment(
  const std::vector<double> & target)
{
  if (!positions(joints_, alignment_start_)) {
    aligning_ = false;
    return;
  }

  if (alignment_start_.size() != target.size()) {
    aligning_ = false;
    return;
  }

  alignment_target_ = target;

  alignment_start_time_ =
    std::chrono::steady_clock::now();

  aligning_ = true;
}

bool Follower::updateAlignment()
{
  if (!aligning_) {
    return true;
  }

  const double elapsed =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    alignment_start_time_).count();

  const double progress =
    std::clamp(
    elapsed / alignment_duration_,
    0.0,
    1.0);

  std::vector<double> command(
    alignment_target_.size());

  for (std::size_t i = 0; i < command.size(); ++i) {
    command[i] =
      alignment_start_[i] +
      progress *
      (alignment_target_[i] -
      alignment_start_[i]);
  }

  publishCommand(command);

  if (progress < 1.0) {
    return false;
  }

  aligning_ = false;
  return true;
}

void Follower::track(
  const std::vector<double> & target)
{
  publishCommand(target);
}

void Follower::publishCommand(
  const std::vector<double> & positions)
{
  std_msgs::msg::Float64MultiArray command;
  command.data = positions;

  command_publisher_->publish(command);
}

}  // namespace lerobot_teleop
