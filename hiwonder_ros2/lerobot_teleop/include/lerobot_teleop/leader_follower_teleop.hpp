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

#pragma once

#include <memory>

#include "lerobot_teleop/filters/low_pass_filter.hpp"
#include "lerobot_teleop/follower.hpp"
#include "lerobot_teleop/leader.hpp"
#include "lerobot_teleop/state_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lerobot_teleop
{

class LeaderFollowerTeleop
  : public rclcpp_lifecycle::LifecycleNode
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

  [[nodiscard]] bool jointsMatch() const;

  double update_rate_{50.0};

  std::unique_ptr<Leader> leader_;
  std::unique_ptr<Follower> follower_;
  std::unique_ptr<LowPassFilter> low_pass_filter_;

  StateManager state_manager_;

  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace lerobot_teleop
