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

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace lerobot_teleop::utils
{

/**
 * @brief Declare a ROS 2 parameter if it has not already been declared.
 *
 * If the parameter already exists, its current value is returned instead.
 * This allows components to be reconstructed during lifecycle
 * reconfiguration without redeclaring existing parameters.
 *
 * @tparam T Parameter value type.
 * @param node Lifecycle node containing the parameter.
 * @param name Parameter name.
 * @param default_value Default value used when declaring the parameter.
 * @return The current parameter value.
 */
template<typename T>
T declareIfNotDeclared(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(
      name,
      default_value);
  }

  return node.get_parameter(name).get_value<T>();
}

}  // namespace lerobot_teleop::utils
