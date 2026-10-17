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

#include <functional>

#include "lerobot_teleop/robot.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace lerobot_teleop
{

/**
 * @brief Represents the leader robot in leader-follower teleoperation.
 *
 * Extends Robot with joint state timeout detection and control of the
 * leader guiding mode.
 */
class Leader : public Robot
{
public:
  /**
   * @brief Callback invoked after a guiding mode request completes.
   *
   * The callback receives true if the guiding mode request succeeded,
   * otherwise false.
   */
  using GuidingCallback =
    std::function<void(bool)>;

  /**
   * @brief Construct a leader interface.
   *
   * @param node Lifecycle node used for ROS 2 communication and parameters.
   */
  explicit Leader(
    rclcpp_lifecycle::LifecycleNode & node);

  /**
   * @brief Check whether the leader joint state has timed out.
   *
   * @return true if the age of the latest joint state exceeds the configured
   * timeout, otherwise false.
   */
  [[nodiscard]] bool timedOut() const;

  /**
   * @brief Enable or disable guiding mode on the leader.
   *
   * Sends an asynchronous request to the configured guiding mode service.
   * The callback is invoked when the service response is received.
   *
   * @param enabled true to enable guiding mode, false to disable it.
   * @param callback Callback invoked with the result of the service request.
   * @return true if the request was sent, false if the service is not
   * available.
   */
  bool setGuidingMode(
    bool enabled,
    GuidingCallback callback);

private:
  double timeout_;

  rclcpp::Client<
    std_srvs::srv::SetBool>::SharedPtr guiding_mode_client_;
};

}  // namespace lerobot_teleop
