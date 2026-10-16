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

#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lerobot_teleop
{

/**
 * @brief First-order low-pass filter for joint position commands.
 *
 * The filter operates independently on each element of the input vector.
 * Its behavior is configured through ROS 2 parameters.
 */
class LowPassFilter
{
public:
  /**
   * @brief Construct a low-pass filter.
   *
   * Reads the filter configuration from the lifecycle node and calculates
   * the filter coefficient based on the configured cutoff frequency and
   * update rate.
   *
   * @param node Lifecycle node providing the filter parameters.
   * @param update_rate Filter update rate in Hz.
   *
   * @throws std::invalid_argument If the update rate or cutoff frequency
   * is not valid.
   */
  LowPassFilter(
    rclcpp_lifecycle::LifecycleNode & node,
    double update_rate);

  /**
   * @brief Reset the internal filter state.
   *
   * @param values Values used to initialize the filter state.
   */
  void reset(
    const std::vector<double> & values);

  /**
   * @brief Apply the low-pass filter to the input values.
   *
   * If filtering is disabled, the input values are returned unchanged.
   *
   * @param input Values to filter.
   * @return Filtered values.
   */
  [[nodiscard]] std::vector<double> filter(
    const std::vector<double> & input);

private:
  bool enabled_;
  bool initialized_{false};

  double alpha_;

  std::vector<double> state_;
};

}  // namespace lerobot_teleop
