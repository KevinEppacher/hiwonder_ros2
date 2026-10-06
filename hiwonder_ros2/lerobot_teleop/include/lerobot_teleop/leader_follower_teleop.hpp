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

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

class LeaderFollowerTeleop : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LeaderFollowerTeleop(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

private:
  enum class Mode
  {
    kIdle,
    kWaitingForJointStates,
    kAligning,
    kWaitingForGuiding,
    kTracking
  };

  static constexpr std::size_t kJointCount = 6;

  static constexpr std::array<const char *, kJointCount> kJointNames = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  };

  void leaderJointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  void followerJointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  bool extractJointPositions(
    const sensor_msgs::msg::JointState & msg,
    std::array<double, kJointCount> & positions) const;

  void update();

  void updateAlignment();

  void updateTracking();

  void requestGuidingMode(
    bool enabled);

  void initializeFilter(
    const std::array<double, kJointCount> & positions);

  void applyLowPassFilter(
    const std::array<double, kJointCount> & input,
    std::array<double, kJointCount> & output);

  void publishCommand(
    const std::array<double, kJointCount> & positions);

  std::string leader_joint_states_topic_;
  std::string follower_joint_states_topic_;
  std::string follower_command_topic_;
  std::string guiding_mode_service_;

  double update_rate_{50.0};
  double initial_motion_duration_{3.0};
  double cutoff_frequency_{3.0};
  double leader_timeout_{0.2};

  bool low_pass_filter_enabled_{true};

  double filter_alpha_{1.0};

  std::array<double, kJointCount> leader_positions_{};
  std::array<double, kJointCount> follower_positions_{};
  std::array<double, kJointCount> alignment_start_positions_{};
  std::array<double, kJointCount> alignment_target_positions_{};
  std::array<double, kJointCount> filtered_positions_{};

  bool leader_state_received_{false};
  bool follower_state_received_{false};
  bool filter_initialized_{false};

  Mode mode_{Mode::kIdle};

  std::chrono::steady_clock::time_point last_leader_update_;
  std::chrono::steady_clock::time_point alignment_start_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    leader_joint_state_subscription_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    follower_joint_state_subscription_;

  rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
    guiding_mode_client_;

  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace lerobot_teleop
