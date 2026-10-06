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

#include "lerobot_teleop/leader_follower_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>
#include <numbers>

namespace lerobot_teleop
{

LeaderFollowerTeleop::LeaderFollowerTeleop(
  const rclcpp::NodeOptions & options)
: LifecycleNode("leader_follower_teleop_node", options)
{
  declare_parameter("update_rate", 50.0);

  declare_parameter(
    "leader_joint_states_topic",
    "/leader/joint_states");

  declare_parameter(
    "follower_joint_states_topic",
    "/follower/joint_states");

  declare_parameter(
    "follower_command_topic",
    "/follower/position_controller/commands");

  declare_parameter(
    "guiding_mode_service",
    "/leader/leader_so101/guiding_mode");

  declare_parameter(
    "initial_motion.duration",
    3.0);

  declare_parameter(
    "low_pass_filter.enabled",
    true);

  declare_parameter(
    "low_pass_filter.cutoff_frequency",
    3.0);

  declare_parameter(
    "safety.leader_timeout",
    0.2);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_configure(
  const rclcpp_lifecycle::State &)
{
  update_rate_ =
    get_parameter("update_rate").as_double();

  leader_joint_states_topic_ =
    get_parameter("leader_joint_states_topic").as_string();

  follower_joint_states_topic_ =
    get_parameter("follower_joint_states_topic").as_string();

  follower_command_topic_ =
    get_parameter("follower_command_topic").as_string();

  guiding_mode_service_ =
    get_parameter("guiding_mode_service").as_string();

  initial_motion_duration_ =
    get_parameter("initial_motion.duration").as_double();

  low_pass_filter_enabled_ =
    get_parameter("low_pass_filter.enabled").as_bool();

  cutoff_frequency_ =
    get_parameter("low_pass_filter.cutoff_frequency").as_double();

  leader_timeout_ =
    get_parameter("safety.leader_timeout").as_double();

  if (
    update_rate_ <= 0.0 ||
    initial_motion_duration_ <= 0.0 ||
    cutoff_frequency_ <= 0.0 ||
    leader_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Teleop parameters must be greater than zero");

    return CallbackReturn::FAILURE;
  }

  const double dt = 1.0 / update_rate_;
  const double rc =
    1.0 / (2.0 * std::numbers::pi * cutoff_frequency_);

  filter_alpha_ = dt / (rc + dt);

  leader_joint_state_subscription_ =
    create_subscription<sensor_msgs::msg::JointState>(
    leader_joint_states_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &LeaderFollowerTeleop::leaderJointStateCallback,
      this,
      std::placeholders::_1));

  follower_joint_state_subscription_ =
    create_subscription<sensor_msgs::msg::JointState>(
    follower_joint_states_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &LeaderFollowerTeleop::followerJointStateCallback,
      this,
      std::placeholders::_1));

  command_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>(
    follower_command_topic_,
    10);

  guiding_mode_client_ =
    create_client<std_srvs::srv::SetBool>(
    guiding_mode_service_);

  const auto period =
    std::chrono::duration<double>(1.0 / update_rate_);

  update_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(
      &LeaderFollowerTeleop::update,
      this));

  leader_state_received_ = false;
  follower_state_received_ = false;
  filter_initialized_ = false;
  mode_ = Mode::kIdle;

  RCLCPP_INFO(
    get_logger(),
    "Configured leader-follower teleoperation");

  return CallbackReturn::SUCCESS;
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_activate(
  const rclcpp_lifecycle::State & state)
{
  command_publisher_->on_activate();

  mode_ = Mode::kWaitingForJointStates;

  RCLCPP_INFO(
    get_logger(),
    "Teleoperation activated, waiting for leader and follower joint states");

  return LifecycleNode::on_activate(state);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  mode_ = Mode::kIdle;
  filter_initialized_ = false;

  requestGuidingMode(false);

  command_publisher_->on_deactivate();

  RCLCPP_INFO(
    get_logger(),
    "Leader-follower teleoperation deactivated");

  return LifecycleNode::on_deactivate(state);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  update_timer_.reset();
  guiding_mode_client_.reset();
  command_publisher_.reset();
  follower_joint_state_subscription_.reset();
  leader_joint_state_subscription_.reset();

  leader_state_received_ = false;
  follower_state_received_ = false;
  filter_initialized_ = false;
  mode_ = Mode::kIdle;

  return CallbackReturn::SUCCESS;
}

void LeaderFollowerTeleop::leaderJointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::array<double, kJointCount> positions{};

  if (!extractJointPositions(*msg, positions)) {
    RCLCPP_WARN(
      get_logger(),
      "Leader JointState does not contain all required joints");

    return;
  }

  leader_positions_ = positions;
  leader_state_received_ = true;
  last_leader_update_ = std::chrono::steady_clock::now();
}

void LeaderFollowerTeleop::followerJointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::array<double, kJointCount> positions{};

  if (!extractJointPositions(*msg, positions)) {
    RCLCPP_WARN(
      get_logger(),
      "Follower JointState does not contain all required joints");

    return;
  }

  follower_positions_ = positions;
  follower_state_received_ = true;
}

bool LeaderFollowerTeleop::extractJointPositions(
  const sensor_msgs::msg::JointState & msg,
  std::array<double, kJointCount> & positions) const
{
  if (msg.name.size() != msg.position.size()) {
    return false;
  }

  for (std::size_t joint_index = 0;
    joint_index < kJointCount;
    ++joint_index)
  {
    const auto iterator =
      std::find(
      msg.name.begin(),
      msg.name.end(),
      kJointNames[joint_index]);

    if (iterator == msg.name.end()) {
      return false;
    }

    const auto index =
      static_cast<std::size_t>(
      std::distance(
        msg.name.begin(),
        iterator));

    positions[joint_index] =
      msg.position[index];
  }

  return true;
}

void LeaderFollowerTeleop::update()
{
  switch (mode_) {
    case Mode::kWaitingForJointStates:
      if (leader_state_received_ && follower_state_received_) {
        alignment_start_positions_ =
          follower_positions_;

        alignment_target_positions_ =
          leader_positions_;

        alignment_start_time_ =
          std::chrono::steady_clock::now();

        mode_ = Mode::kAligning;

        RCLCPP_INFO(
          get_logger(),
          "Leader and follower joint states received, starting alignment");
      }
      break;

    case Mode::kAligning:
      updateAlignment();
      break;

    case Mode::kTracking:
      updateTracking();
      break;

    case Mode::kIdle:
    case Mode::kWaitingForGuiding:
      break;
  }
}

void LeaderFollowerTeleop::updateAlignment()
{
  const auto now =
    std::chrono::steady_clock::now();

  const double elapsed =
    std::chrono::duration<double>(
    now - alignment_start_time_).count();

  const double progress =
    std::clamp(
    elapsed / initial_motion_duration_,
    0.0,
    1.0);

  std::array<double, kJointCount> command{};

  for (std::size_t i = 0; i < kJointCount; ++i) {
    command[i] =
      alignment_start_positions_[i] +
      progress *
      (alignment_target_positions_[i] -
      alignment_start_positions_[i]);
  }

  publishCommand(command);

  if (progress < 1.0) {
    return;
  }

  initializeFilter(alignment_target_positions_);

  mode_ = Mode::kWaitingForGuiding;

  requestGuidingMode(true);
}

void LeaderFollowerTeleop::updateTracking()
{
  const auto now =
    std::chrono::steady_clock::now();

  const double leader_age =
    std::chrono::duration<double>(
    now - last_leader_update_).count();

  if (leader_age > leader_timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Leader joint state timed out");

    return;
  }

  std::array<double, kJointCount> command{};

  applyLowPassFilter(
    leader_positions_,
    command);

  publishCommand(command);
}

void LeaderFollowerTeleop::requestGuidingMode(
  bool enabled)
{
  if (!guiding_mode_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(),
      "Guiding mode service is not available");

    mode_ = Mode::kIdle;
    return;
  }

  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = enabled;

  guiding_mode_client_->async_send_request(
    request,
    [this, enabled](
      rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
      const auto response = future.get();

      if (!response->success) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to set guiding mode: %s",
          response->message.c_str());

        mode_ = Mode::kIdle;
        return;
      }

      if (enabled) {
        mode_ = Mode::kTracking;

        RCLCPP_INFO(
          get_logger(),
          "Leader guiding mode enabled, teleoperation started");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Leader guiding mode disabled");
      }
    });
}

void LeaderFollowerTeleop::initializeFilter(
  const std::array<double, kJointCount> & positions)
{
  filtered_positions_ = positions;
  filter_initialized_ = true;
}

void LeaderFollowerTeleop::applyLowPassFilter(
  const std::array<double, kJointCount> & input,
  std::array<double, kJointCount> & output)
{
  if (!low_pass_filter_enabled_) {
    output = input;
    return;
  }

  if (!filter_initialized_) {
    initializeFilter(input);
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    filtered_positions_[i] +=
      filter_alpha_ *
      (input[i] - filtered_positions_[i]);

    output[i] = filtered_positions_[i];
  }
}

void LeaderFollowerTeleop::publishCommand(
  const std::array<double, kJointCount> & positions)
{
  if (!command_publisher_->is_activated()) {
    return;
  }

  std_msgs::msg::Float64MultiArray command;

  command.data.assign(
    positions.begin(),
    positions.end());

  command_publisher_->publish(command);
}

}  // namespace lerobot_teleop
