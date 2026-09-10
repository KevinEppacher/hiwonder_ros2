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

#include "hiwonder_servo_driver/servo.hpp"

#include "hiwonder_servo_driver/registers.hpp"

namespace hiwonder
{

Servo::Servo(HiwonderBus & bus, uint8_t id)
: bus_(bus),
  id_(id)
{
}

uint8_t Servo::id() const noexcept
{
  return id_;
}

bool Servo::ping()
{
  return bus_.ping(id_);
}

bool Servo::writePersistent(
  uint8_t address,
  const uint8_t * data,
  uint8_t length)
{
  constexpr uint8_t unlock = 0;
  constexpr uint8_t lock = 1;

  if (!bus_.write(
                id_,
                reg::kNvsLock,
                std::span<const uint8_t>(&unlock, 1)))
  {
    return false;
  }

  if (!bus_.write(
                id_,
                address,
                std::span<const uint8_t>(data, length)))
  {
    return false;
  }

  if (!bus_.write(
                id_,
                reg::kNvsLock,
                std::span<const uint8_t>(&lock, 1)))
  {
    return false;
  }

  return true;
}

bool Servo::setId(uint8_t new_id)
{
  if (new_id == id_) {
    return true;
  }

  constexpr uint8_t unlock = 0;
  constexpr uint8_t lock = 1;

  if (!bus_.write(
                id_,
                reg::kNvsLock,
                std::span<const uint8_t>(&unlock, 1)))
  {
    return false;
  }

        /*
        * The servo starts using the new ID after this write.
        * Subsequent packets must therefore address new_id.
        */
  if (!bus_.write(
                id_,
                reg::kId,
                std::span<const uint8_t>(&new_id, 1)))
  {
    return false;
  }

  id_ = new_id;

  if (!bus_.write(
                id_,
                reg::kNvsLock,
                std::span<const uint8_t>(&lock, 1)))
  {
    return false;
  }

  return bus_.ping(id_);
}

bool Servo::setMaxTorque(uint16_t permille)
{
  if (permille > 1000) {
    return false;
  }

  const uint8_t data[2]{
    static_cast<uint8_t>(permille & 0xFF),
    static_cast<uint8_t>((permille >> 8) & 0xFF)
  };

  return writePersistent(
            reg::kMaxTorque,
            data,
            2);
}

bool Servo::setTorqueEnabled(bool enabled)
{
  const uint8_t value = enabled ? 1 : 0;
  return bus_.write(id_, reg::kTorqueEnable, std::span<const uint8_t>(&value, 1));
}

}  // namespace hiwonder
