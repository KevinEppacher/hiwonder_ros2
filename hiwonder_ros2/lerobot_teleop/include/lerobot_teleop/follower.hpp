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
#include <optional>
#include <string>
#include <vector>

#include "lerobot_teleop/robot.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lerobot_teleop
{

/**
 * @brief Represents the follower robot in leader-follower teleoperation.
 *
 * The follower discovers its command controller, determines the configured
 * joint order, aligns itself with the leader, and tracks leader joint
 * positions during teleoperation.
 */
class Follower : public Robot
{
public:
  /**
   * @brief Construct a follower interface.
   *
   * @param node Lifecycle node used for ROS 2 communication and parameters.
   *
   * @throws std::invalid_argument If the configured alignment duration is
   * not greater than zero.
   */
  explicit Follower(
    rclcpp_lifecycle::LifecycleNode & node);

  /**
   * @brief Discover the controller subscribed to the command topic.
   *
   * Controller discovery is asynchronous. Repeated calls may be required
   * until the controller and its configured joints have been discovered.
   *
   * @return true if the controller has been discovered, otherwise false.
   */
  [[nodiscard]] bool discoverController();

  /**
   * @brief Check whether the command controller has been discovered.
   *
   * @return true if controller discovery is complete, otherwise false.
   */
  [[nodiscard]] bool controllerDiscovered() const noexcept;

  /**
   * @brief Get the joint names in the command controller order.
   *
   * @return Reference to the configured command joint names.
   */
  [[nodiscard]] const std::vector<std::string> &
  commandJointNames() const noexcept;

  /**
   * @brief Start alignment of the follower with a target joint configuration.
   *
   * @param target Target joint positions in controller joint order.
   */
  void startAlignment(
    const std::vector<double> & target);

  /**
   * @brief Update the follower alignment.
   *
   * Computes and publishes the next interpolated joint command.
   *
   * @return true when alignment is complete, otherwise false.
   */
  [[nodiscard]] bool updateAlignment();

  /**
   * @brief Command the follower to track target joint positions.
   *
   * @param target Target joint positions in controller joint order.
   */
  void track(
    const std::vector<double> & target);

private:
  /**
   * @brief Get the nodes subscribed to the follower command topic.
   *
   * @return Fully qualified names of subscribing nodes.
   */
  [[nodiscard]] std::vector<std::string>
  commandSubscribers() const;

  /**
   * @brief Find a command-topic subscriber with an available parameter service.
   *
   * @return Node name of a candidate controller, or std::nullopt if none
   * is available.
   */
  [[nodiscard]] std::optional<std::string>
  findCommandController();

  /**
   * @brief Request the configured joints from a controller candidate.
   *
   * @param controller_node Fully qualified controller node name.
   */
  void requestControllerJoints(
    const std::string & controller_node);

  /**
   * @brief Process the joints parameter returned by a controller candidate.
   *
   * @param controller_node Fully qualified controller node name.
   * @param parameters Parameters returned by the parameter request.
   */
  void handleControllerJoints(
    const std::string & controller_node,
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Validate a list of controller joint names.
   *
   * @param joints Joint names to validate.
   * @return true if the list is non-empty and contains only unique,
   * non-empty joint names.
   */
  [[nodiscard]] bool validJointNames(
    const std::vector<std::string> & joints) const;

  /**
   * @brief Prepare the initial and target states for follower alignment.
   *
   * @param target Target joint positions in controller joint order.
   * @return true if alignment can be started, otherwise false.
   */
  [[nodiscard]] bool prepareAlignment(
    const std::vector<double> & target);

  /**
   * @brief Calculate the current normalized alignment progress.
   *
   * @return Alignment progress in the range [0.0, 1.0].
   */
  [[nodiscard]] double alignmentProgress() const;

  /**
   * @brief Calculate an interpolated alignment command.
   *
   * @param progress Normalized alignment progress.
   * @return Interpolated joint positions.
   */
  [[nodiscard]] std::vector<double>
  alignmentCommand(
    double progress) const;

  /**
   * @brief Publish joint position commands to the follower controller.
   *
   * @param positions Joint positions in controller joint order.
   */
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
