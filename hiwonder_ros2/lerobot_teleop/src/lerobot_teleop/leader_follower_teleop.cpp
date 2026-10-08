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

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace lerobot_teleop
{

LeaderFollowerTeleop::LeaderFollowerTeleop(
  const rclcpp::NodeOptions & options)
: LifecycleNode(
    "leader_follower_teleop_node",
    options)
{
  declare_parameter<double>(
    "update_rate",
    50.0);
}

LeaderFollowerTeleop::CallbackReturn
LeaderFollowerTeleop::on_configure(
  const rclcpp_lifecycle::State &)
{
  update_rate_ =
    get_parameter(
    "update_rate").as_double();

  if (update_rate_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "Update rate must be greater than zero");

    return CallbackReturn::FAILURE;
  }

  try {
    leader_ =
      std::make_unique<Leader>(
      *this);

    follower_ =
      std::make_unique<Follower>(
      *this);

    low_pass_filter_ =
      std::make_unique<LowPassFilter>(
      *this,
      update_rate_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure teleoperation: %s",
      exception.what());

    leader_.reset();
    follower_.reset();
    low_pass_filter_.reset();

    return CallbackReturn::FAILURE;
  }

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

  if (!follower_->controllerDiscovered()) {
    if (!follower_->discoverController()) {
      return;
    }
  }

  if (!jointsMatch()) {
    RCLCPP_ERROR(
      get_logger(),
      "Leader and follower do not provide all controller joints");

    state_manager_.transitionTo(
      StateManager::State::kIdle);

    return;
  }

  std::vector<double> leader_positions;

  if (!leader_->positions(
      follower_->commandJointNames(),
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
      follower_->commandJointNames(),
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
      follower_->commandJointNames(),
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

  follower_->track(
    target);
}

bool LeaderFollowerTeleop::jointsMatch() const
{
  const auto & command_joint_names =
    follower_->commandJointNames();

  if (command_joint_names.empty()) {
    return false;
  }

  std::vector<double> leader_positions;
  std::vector<double> follower_positions;

  return
    leader_->positions(
      command_joint_names,
      leader_positions) &&
    follower_->positions(
      command_joint_names,
      follower_positions);
}

}  // namespace lerobot_teleop
