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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "lerobot_teleop/leader_follower_teleop.hpp"

namespace lerobot_teleop
{

LeaderFollowerTeleop::LeaderFollowerTeleop(
  const rclcpp::NodeOptions & options)
: LifecycleNode("leader_follower_teleop_node", options)
{
  declare_parameter(
    "update_rate",
    50.0);

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

  declare_parameter<std::vector<std::string>>(
  "follower.joints",
  {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
  });
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_configure(
  const rclcpp_lifecycle::State &)
{
  update_rate_ =
    get_parameter("update_rate").as_double();

  const LeaderParameters leader_parameters{
    .joint_states_topic =
      get_parameter(
      "leader_joint_states_topic").as_string(),

    .guiding_mode_service =
      get_parameter(
      "guiding_mode_service").as_string(),

    .timeout =
      get_parameter(
      "safety.leader_timeout").as_double()
  };

  const FollowerParameters follower_parameters{
    .joint_states_topic =
      get_parameter(
      "follower_joint_states_topic").as_string(),

    .command_topic =
      get_parameter(
      "follower_command_topic").as_string(),

    .joints =
      get_parameter(
      "follower.joints").as_string_array(),

    .alignment_duration =
      get_parameter(
      "initial_motion.duration").as_double()
  };

  const bool low_pass_filter_enabled =
    get_parameter(
    "low_pass_filter.enabled").as_bool();

  const double cutoff_frequency =
    get_parameter(
    "low_pass_filter.cutoff_frequency").as_double();

  if (
    update_rate_ <= 0.0 ||
    leader_parameters.timeout <= 0.0 ||
    follower_parameters.alignment_duration <= 0.0 ||
    cutoff_frequency <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Teleop parameters must be greater than zero");

    return CallbackReturn::FAILURE;
  }

  joint_names_ =
    get_parameter(
    "follower.joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Follower joint list must not be empty");

    return CallbackReturn::FAILURE;
  }

  leader_ =
    std::make_unique<Leader>(
    *this,
    leader_parameters);

  follower_ =
    std::make_unique<Follower>(
    *this,
    follower_parameters);

  low_pass_filter_ =
    std::make_unique<LowPassFilter>(
    update_rate_,
    cutoff_frequency,
    low_pass_filter_enabled);

  const auto period =
    std::chrono::duration<double>(
    1.0 / update_rate_);

  update_timer_ =
    create_wall_timer(
    std::chrono::duration_cast<
      std::chrono::nanoseconds>(period),
    std::bind(
      &LeaderFollowerTeleop::update,
      this));

  state_manager_.transitionTo(
    StateManager::State::kIdle);

  RCLCPP_INFO(
    get_logger(),
    "Configured leader-follower teleoperation");

  return CallbackReturn::SUCCESS;
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_activate(
  const rclcpp_lifecycle::State & state)
{
  state_manager_.transitionTo(
    StateManager::State::kWaitingForJointStates);

  RCLCPP_INFO(
    get_logger(),
    "Teleoperation activated, waiting for leader and follower joint states");

  return LifecycleNode::on_activate(state);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  state_manager_.transitionTo(
    StateManager::State::kIdle);

  if (leader_) {
    leader_->setGuidingMode(
      false,
      [this](bool success)
      {
        if (!success) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to disable leader guiding mode");
        }
      });
  }

  RCLCPP_INFO(
    get_logger(),
    "Leader-follower teleoperation deactivated");

  return LifecycleNode::on_deactivate(state);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  state_manager_.transitionTo(
    StateManager::State::kIdle);

  update_timer_.reset();

  low_pass_filter_.reset();
  follower_.reset();
  leader_.reset();

  RCLCPP_INFO(
    get_logger(),
    "Cleaned up leader-follower teleoperation");

  return CallbackReturn::SUCCESS;
}

void LeaderFollowerTeleop::update()
{
  switch (state_manager_.state()) {
    case StateManager::State::kWaitingForJointStates:
      waitForJointStates();
      break;

    case StateManager::State::kAligning:
      alignFollower();
      break;

    case StateManager::State::kTracking:
      trackLeader();
      break;

    case StateManager::State::kIdle:
    case StateManager::State::kWaitingForGuiding:
      break;
  }
}

void LeaderFollowerTeleop::waitForJointStates()
{
  if (
    !leader_->stateReceived() ||
    !follower_->stateReceived())
  {
    return;
  }

  if (!jointsMatch()) {
    RCLCPP_ERROR(
      get_logger(),
      "Leader and follower joint names do not match");

    state_manager_.transitionTo(
      StateManager::State::kIdle);

    return;
  }

  std::vector<double> leader_positions;

  if (!leader_->positions(
      joint_names_,
      leader_positions))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to map leader joint positions");

    state_manager_.transitionTo(
      StateManager::State::kIdle);

    return;
  }

  follower_->startAlignment(
    leader_positions);

  state_manager_.transitionTo(
    StateManager::State::kAligning);

  RCLCPP_INFO(
    get_logger(),
    "Leader and follower joint states received, starting alignment");
}

void LeaderFollowerTeleop::alignFollower()
{
  if (!follower_->updateAlignment()) {
    return;
  }

  std::vector<double> leader_positions;

  if (!leader_->positions(
      joint_names_,
      leader_positions))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to map leader joint positions");

    state_manager_.transitionTo(
      StateManager::State::kIdle);

    return;
  }

  low_pass_filter_->reset(
    leader_positions);

  state_manager_.transitionTo(
    StateManager::State::kWaitingForGuiding);

  const bool request_sent =
    leader_->setGuidingMode(
    true,
    [this](bool success)
    {
      if (!success) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to enable leader guiding mode");

        state_manager_.transitionTo(
          StateManager::State::kIdle);

        return;
      }

      state_manager_.transitionTo(
        StateManager::State::kTracking);

      RCLCPP_INFO(
        get_logger(),
        "Leader guiding mode enabled, teleoperation started");
    });

  if (!request_sent) {
    state_manager_.transitionTo(
      StateManager::State::kIdle);
  }
}

void LeaderFollowerTeleop::trackLeader()
{
  if (leader_->timedOut()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Leader joint state timed out");

    return;
  }

  std::vector<double> leader_positions;

  if (!leader_->positions(
      joint_names_,
      leader_positions))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Failed to map leader joint positions");

    return;
  }

  const auto target =
    low_pass_filter_->filter(
    leader_positions);

  follower_->track(target);
}

bool LeaderFollowerTeleop::jointsMatch() const
{
  std::vector<double> leader_positions;
  std::vector<double> follower_positions;

  return
    leader_->positions(
      joint_names_,
      leader_positions) &&
    follower_->positions(
      joint_names_,
      follower_positions);
}
}  // namespace lerobot_teleop
