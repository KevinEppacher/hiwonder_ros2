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

#include "hiwonder_lerobot_cpp/robot.hpp"

namespace hiwonder
{

class Follower : public LeRobot
{
public:
  Follower()
  {
    addMotor(1);
    addMotor(2);
    addMotor(3);
    addMotor(4);
    addMotor(5);
    addMotor(6);
  }

  explicit Follower(
    const std::string & port);

  ~Follower() override = default;
};

}  // namespace hiwonder