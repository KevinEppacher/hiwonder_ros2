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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "lerobot_teleop/leader_follower_teleop.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

class LeaderFollowerTeleopTest
  : public ::testing::Test
{
protected:
  static void SetUpTestSuite();

  static void TearDownTestSuite();

  void SetUp() override;

  void TearDown() override;

  [[nodiscard]] sensor_msgs::msg::JointState createJointState(
    const std::vector<double> & positions);

  [[nodiscard]] sensor_msgs::msg::JointState createJointState(
    const std::vector<std::string> & names,
    const std::vector<double> & positions);

  void publishLeader(
    const std::vector<double> & positions);

  void publishFollower(
    const std::vector<double> & positions);

  void publishJointStates(
    const std::vector<double> & leader,
    const std::vector<double> & follower);

  void publishJointStates(
    const std::vector<std::string> & names,
    const std::vector<double> & leader,
    const std::vector<double> & follower);

  void spinFor(
    std::chrono::milliseconds duration);

  void publishFor(
    const std::vector<double> & leader,
    const std::vector<double> & follower,
    std::chrono::milliseconds duration);

  void publishFor(
    const std::vector<std::string> & names,
    const std::vector<double> & leader,
    const std::vector<double> & follower,
    std::chrono::milliseconds duration);

  void activateTeleop();

  void expectCommandWithinBounds(
    const std::vector<double> & command,
    const std::vector<double> & start,
    const std::vector<double> & target);

  void expectNear(
    const std::vector<double> & actual,
    const std::vector<double> & expected,
    double tolerance);

  const std::vector<std::string> joint_names_{
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  };

  static constexpr double kTolerance =
    1e-6;

  rclcpp::executors::SingleThreadedExecutor
    executor_;

  std::shared_ptr<rclcpp::Node>
  mock_node_;

  std::shared_ptr<rclcpp::Node>
  controller_node_;

  std::shared_ptr<LeaderFollowerTeleop>
  teleop_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    leader_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    follower_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    controller_command_subscription_;

  rclcpp::Service<
    std_srvs::srv::SetBool>::SharedPtr
    guiding_service_;

  std::vector<std::vector<double>>
  received_commands_;

  bool guiding_enabled_{false};

  std::size_t guiding_service_calls_{0};
};

}  // namespace lerobot_teleop
