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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lerobot
{

struct RobotConfig
{
  std::string port;
  uint32_t baud_rate{1000000};
};

class Robot
{
public:
  virtual ~Robot() = default;

  virtual bool configure(
    const RobotConfig & config) = 0;

  virtual bool connect() = 0;

  virtual void disconnect() = 0;

  [[nodiscard]] virtual bool isConnected() const noexcept = 0;

  [[nodiscard]] virtual std::size_t motorCount() const noexcept = 0;

  virtual bool setTorqueEnabled(
    bool enabled) = 0;

  virtual bool readPositions(
    std::vector<uint16_t> & positions) = 0;

  virtual bool writePositions(
    const std::vector<uint16_t> & positions) = 0;
};

}  // namespace lerobot
