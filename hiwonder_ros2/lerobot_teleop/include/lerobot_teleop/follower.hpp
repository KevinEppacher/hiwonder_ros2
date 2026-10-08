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
#include <memory>
#include <string>
#include <vector>

#include "lerobot_teleop/robot.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lerobot_teleop
{

class Follower : public Robot
{
public:
  explicit Follower(
    rclcpp_lifecycle::LifecycleNode & node);

  [[nodiscard]] bool discoverController();

  [[nodiscard]] bool controllerDiscovered() const noexcept;

  [[nodiscard]] const std::vector<std::string> &
    commandJointNames() const noexcept;

  void startAlignment(
    const std::vector<double> & target);

  [[nodiscard]] bool updateAlignment();

  void track(
    const std::vector<double> & target);

private:
  void requestControllerJoints(
    const std::string & controller_node);

  void publishCommand(
    const std::vector<double> & positions);

  std::string command_topic_;

  double alignment_duration_;

  bool aligning_{false};
  bool discovery_in_progress_{false};

  std::vector<std::string> command_joint_names_;

  std::vector<double> alignment_start_;
  std::vector<double> alignment_target_;

  std::chrono::steady_clock::time_point
    alignment_start_time_;

  std::shared_ptr<rclcpp::AsyncParametersClient>
    parameter_client_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
};

}  // namespace lerobot_teleop
