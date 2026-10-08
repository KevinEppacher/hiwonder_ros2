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

namespace lerobot_teleop
{

class StateManager
{
public:
  enum class State
  {
    kIdle,
    kWaitingForJointStates,
    kAligning,
    kWaitingForGuiding,
    kTracking
  };

  [[nodiscard]] State state() const noexcept
  {
    return state_;
  }

  void transitionTo(
    State state) noexcept
  {
    state_ = state;
  }

  [[nodiscard]] bool is(
    State state) const noexcept
  {
    return state_ == state;
  }

private:
  State state_{State::kIdle};
};

}  // namespace lerobot_teleop
