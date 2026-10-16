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
  int16_t & position)
{
  uint16_t raw_position{};

  if (!bus_.readWord(
      id_,
      reg::kCurrentPosition,
      raw_position))
  {
    return false;
  }

  position = decodePosition(raw_position);

  return true;
}

bool Motor::writePosition(
  int16_t position)
{
  return bus_.writeWord(
    id_,
    reg::kTargetPosition,
    encodePosition(position));
}

uint16_t Motor::encodePosition(
  int16_t position)
{
  const uint16_t magnitude =
    static_cast<uint16_t>(
    position < 0 ?
    -static_cast<int32_t>(position) :
    static_cast<int32_t>(position));

  return position < 0 ?
         static_cast<uint16_t>(0x8000U | magnitude) :
         magnitude;
}

int16_t Motor::decodeVelocity(uint16_t raw_velocity)
{
  constexpr uint16_t kDirectionBit = 0x8000;
  constexpr uint16_t kSpeedMask = 0x7FFF;

  const int16_t speed =
    static_cast<int16_t>(raw_velocity & kSpeedMask);

  if ((raw_velocity & kDirectionBit) != 0) {
    return -speed;
  }

  return speed;
}

int16_t Motor::decodePosition(
  uint16_t raw_position)
{
  const auto magnitude =
    static_cast<int16_t>(
    raw_position & 0x7FFFU);

  return (raw_position & 0x8000U) != 0U ?
         -magnitude :
         magnitude;
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

bool Motor::readState(
  MotorState & state)
{
  std::array<uint8_t, 4> motion_data{};

  if (!bus_.read(
      id_,
      reg::kCurrentPosition,
      motion_data))
  {
    return false;
  }

  const uint16_t raw_position =
    static_cast<uint16_t>(motion_data[0]) |
    (static_cast<uint16_t>(motion_data[1]) << 8);

  const uint16_t raw_velocity =
    static_cast<uint16_t>(motion_data[2]) |
    (static_cast<uint16_t>(motion_data[3]) << 8);

  uint16_t raw_current{};

  if (!bus_.readWord(
      id_,
      reg::kCurrentCurrent,
      raw_current))
  {
    return false;
  }

  state.position = decodePosition(raw_position);
  state.velocity = decodeVelocity(raw_velocity);
  state.current = raw_current;

  return true;
}

}  // namespace hiwonder
