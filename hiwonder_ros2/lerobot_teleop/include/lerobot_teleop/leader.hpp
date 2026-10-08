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

class Leader : public Robot
{
public:
  using GuidingCallback =
    std::function<void(bool)>;

  explicit Leader(
    rclcpp_lifecycle::LifecycleNode & node);

  [[nodiscard]] bool timedOut() const;

  bool setGuidingMode(
    bool enabled,
    GuidingCallback callback);

private:
  double timeout_;

  rclcpp::Client<
    std_srvs::srv::SetBool>::SharedPtr guiding_mode_client_;
};

}  // namespace lerobot_teleop
