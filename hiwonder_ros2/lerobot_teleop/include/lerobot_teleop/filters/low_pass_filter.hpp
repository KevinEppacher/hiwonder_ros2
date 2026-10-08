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

class LowPassFilter
{
public:
  LowPassFilter(
    rclcpp_lifecycle::LifecycleNode & node,
    double update_rate);

  void reset(
    const std::vector<double> & values);

  [[nodiscard]] std::vector<double> filter(
    const std::vector<double> & input);

private:
  bool enabled_;
  bool initialized_{false};

  double alpha_;

  std::vector<double> state_;
};

}  // namespace lerobot_teleop
