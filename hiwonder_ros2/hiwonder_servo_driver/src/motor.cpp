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

#include "hiwonder_servo_driver/motor.hpp"

#include <array>
#include <cstdint>

#include "hiwonder_servo_driver/registers.hpp"

namespace hiwonder
{

Motor::Motor(
  uint8_t id,
  HiwonderBus & bus)
: id_(id),
  bus_(bus)
{
}

uint8_t Motor::id() const noexcept
{
  return id_;
}

bool Motor::ping()
{
  return bus_.ping(id_);
}

bool Motor::readPosition(
  uint16_t & position)
{
  return bus_.readWord(
    id_,
    reg::kCurrentPosition,
    position);
}

bool Motor::writePosition(
  uint16_t position)
{
  return bus_.writeWord(
    id_,
    reg::kTargetPosition,
    position);
}

bool Motor::setTorqueEnabled(
  bool enabled)
{
  const std::array<uint8_t, 1> data{
    static_cast<uint8_t>(enabled ? 1 : 0)
  };

  return bus_.write(
    id_,
    reg::kTorqueEnable,
    data);
}

}  // namespace hiwonder
