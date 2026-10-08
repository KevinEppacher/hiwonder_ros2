#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "lerobot_teleop/robot.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lerobot_teleop
{

struct FollowerParameters
{
  std::string joint_states_topic;
  std::string command_topic;
  std::vector<std::string> joints;
  double alignment_duration;
};

class Follower : public Robot
{
public:
  Follower(
    rclcpp_lifecycle::LifecycleNode & node,
    const FollowerParameters & parameters);

  void startAlignment(
    const std::vector<double> & target);

  [[nodiscard]] bool updateAlignment();

  void track(
    const std::vector<double> & target);

private:
  void publishCommand(
    const std::vector<double> & positions);

  double alignment_duration_;

  bool aligning_{false};

  std::vector<double> alignment_start_;
  std::vector<double> alignment_target_;

  std::chrono::steady_clock::time_point
    alignment_start_time_;

  std::vector<std::string> joints_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
};

}  // namespace lerobot_teleop
