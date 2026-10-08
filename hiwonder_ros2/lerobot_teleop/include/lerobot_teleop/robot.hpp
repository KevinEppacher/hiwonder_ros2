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
    const std::string & joint_state_topic);

  virtual ~Robot() = default;

  [[nodiscard]] bool stateReceived() const noexcept;

  [[nodiscard]] const sensor_msgs::msg::JointState &
  jointState() const noexcept;

  [[nodiscard]] bool positions(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions) const;

  [[nodiscard]] double stateAge() const;

protected:
  rclcpp_lifecycle::LifecycleNode & node_;

private:
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  sensor_msgs::msg::JointState joint_state_;

  bool state_received_{false};

  std::chrono::steady_clock::time_point
    last_update_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace lerobot_teleop
