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

#include "lerobot_teleop/follower.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lerobot_teleop
{

Follower::Follower(
  rclcpp_lifecycle::LifecycleNode & node)
: Robot(
    node,
    node.declare_parameter<std::string>(
      "follower.joint_state_topic",
      "/follower/joint_states")),
  command_topic_(
    node.declare_parameter<std::string>(
      "follower.command_topic",
      "/follower/position_controller/commands")),
  alignment_duration_(
    node.declare_parameter<double>(
      "follower.alignment_duration",
      3.0))
{
  if (alignment_duration_ <= 0.0) {
    throw std::invalid_argument(
            "Follower alignment duration must be greater than zero");
  }

  command_publisher_ =
    node_.create_publisher<
    std_msgs::msg::Float64MultiArray>(
    command_topic_,
    10);
}

bool Follower::discoverController()
{
  if (controllerDiscovered()) {
    return true;
  }

  if (discovery_in_progress_) {
    return false;
  }

  const auto subscriptions =
    node_.get_subscriptions_info_by_topic(
    command_topic_);

  if (subscriptions.empty()) {
    return false;
  }

  std::unordered_set<std::string>
    subscriber_nodes;

  for (const auto & subscription : subscriptions) {
    std::string node_name =
      subscription.node_namespace();

    if (
      node_name.empty() ||
      node_name == "/")
    {
      node_name = "/";
    } else if (node_name.back() != '/') {
      node_name += '/';
    }

    node_name +=
      subscription.node_name();

    subscriber_nodes.insert(
      std::move(node_name));
  }

  if (subscriber_nodes.empty()) {
    return false;
  }

  if (subscriber_nodes.size() > 1) {
    RCLCPP_DEBUG(
      node_.get_logger(),
      "Found %zu subscribers on command topic '%s', searching for controller",
      subscriber_nodes.size(),
      command_topic_.c_str());
  }

  /*
   * A monitoring node may also subscribe to the command topic.
   *
   * Prefer subscribers exposing a parameter service. We query candidates
   * one at a time. A controller is accepted only after its "joints"
   * parameter has been received successfully.
   */
  for (const auto & subscriber_node : subscriber_nodes) {
    auto client =
      std::make_shared<rclcpp::AsyncParametersClient>(
      &node_,
      subscriber_node);

    if (!client->service_is_ready()) {
      continue;
    }

    parameter_client_ =
      std::move(client);

    discovery_in_progress_ = true;

    requestControllerJoints(
      subscriber_node);

    return false;
  }

  return false;
}

void Follower::requestControllerJoints(
  const std::string & controller_node)
{
  parameter_client_->get_parameters(
    {"joints"},
    [this, controller_node](
      std::shared_future<
        std::vector<rclcpp::Parameter>> future)
    {
      discovery_in_progress_ = false;

      const auto parameters =
        future.get();

      if (parameters.size() != 1) {
        parameter_client_.reset();
        return;
      }

      const auto & parameter =
        parameters.front();

      if (
        parameter.get_type() !=
        rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        parameter_client_.reset();
        return;
      }

      const auto joints =
        parameter.as_string_array();

      if (joints.empty()) {
        parameter_client_.reset();
        return;
      }

      std::unordered_set<std::string>
        unique_joints;

      for (const auto & joint : joints) {
        if (
          joint.empty() ||
          !unique_joints.insert(joint).second)
        {
          RCLCPP_ERROR(
            node_.get_logger(),
            "Controller '%s' contains invalid or duplicate joint names",
            controller_node.c_str());

          parameter_client_.reset();
          return;
        }
      }

      command_joint_names_ =
        joints;

      RCLCPP_INFO(
        node_.get_logger(),
        "Discovered controller '%s' with %zu command joints",
        controller_node.c_str(),
        command_joint_names_.size());

      parameter_client_.reset();
    });
}

bool Follower::controllerDiscovered() const noexcept
{
  return !command_joint_names_.empty();
}

const std::vector<std::string> &
Follower::commandJointNames() const noexcept
{
  return command_joint_names_;
}

void Follower::startAlignment(
  const std::vector<double> & target)
{
  if (!controllerDiscovered()) {
    aligning_ = false;

    RCLCPP_ERROR(
      node_.get_logger(),
      "Cannot start alignment before controller discovery");

    return;
  }

  if (!positions(
      command_joint_names_,
      alignment_start_))
  {
    aligning_ = false;

    RCLCPP_ERROR(
      node_.get_logger(),
      "Failed to map follower joint positions");

    return;
  }

  if (alignment_start_.size() != target.size()) {
    aligning_ = false;

    RCLCPP_ERROR(
      node_.get_logger(),
      "Follower alignment target size does not match controller joint count");

    return;
  }

  alignment_target_ =
    target;

  alignment_start_time_ =
    std::chrono::steady_clock::now();

  aligning_ = true;
}

bool Follower::updateAlignment()
{
  if (!aligning_) {
    return true;
  }

  const double elapsed =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    alignment_start_time_).count();

  const double progress =
    std::clamp(
    elapsed / alignment_duration_,
    0.0,
    1.0);

  std::vector<double> command(
    alignment_target_.size());

  for (std::size_t i = 0; i < command.size(); ++i) {
    command[i] =
      alignment_start_[i] +
      progress *
      (alignment_target_[i] -
      alignment_start_[i]);
  }

  publishCommand(
    command);

  if (progress < 1.0) {
    return false;
  }

  aligning_ = false;

  return true;
}

void Follower::track(
  const std::vector<double> & target)
{
  if (target.size() != command_joint_names_.size()) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "Tracking target size does not match controller joint count");

    return;
  }

  publishCommand(
    target);
}

void Follower::publishCommand(
  const std::vector<double> & positions)
{
  std_msgs::msg::Float64MultiArray command;

  command.data =
    positions;

  command_publisher_->publish(
    command);
}

}  // namespace lerobot_teleop
