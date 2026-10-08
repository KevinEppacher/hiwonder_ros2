#pragma once

#include <memory>
#include <vector>
#include <string>

#include "lerobot_teleop/filters/low_pass_filter.hpp"
#include "lerobot_teleop/follower.hpp"
#include "lerobot_teleop/leader.hpp"
#include "lerobot_teleop/state_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lerobot_teleop
{

class LeaderFollowerTeleop :
  public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LeaderFollowerTeleop(
    const rclcpp::NodeOptions & options =
    rclcpp::NodeOptions());

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

private:
  void update();

  void waitForJointStates();

  void alignFollower();

  void trackLeader();

  bool jointsMatch() const;

  double update_rate_{50.0};

  std::unique_ptr<Leader> leader_;
  std::unique_ptr<Follower> follower_;
  std::unique_ptr<LowPassFilter> low_pass_filter_;

  StateManager state_manager_;
  std::vector<std::string> joint_names_;

  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace lerobot_teleop
