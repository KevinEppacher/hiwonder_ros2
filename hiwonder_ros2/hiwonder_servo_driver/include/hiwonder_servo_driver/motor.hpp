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

#include <cstdint>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"

namespace hiwonder
{

/**
 * @brief Provides access to a single HiWonder bus servo.
 */
class Motor
{
public:
  /**
   * @brief Constructs a motor interface.
   *
   * @param id Servo ID.
   * @param bus HiWonder bus used for communication with the servo.
   */
  Motor(
    uint8_t id,
    HiwonderBus & bus);

  ~Motor() = default;

  /**
   * @brief Returns the servo ID.
   *
   * @return Servo ID.
   */
  [[nodiscard]] uint8_t id() const noexcept;

  /**
   * @brief Checks whether the servo responds on the bus.
   *
   * @return True if the servo responds, otherwise false.
   */
  bool ping();

  /**
   * @brief Reads the current raw servo position.
   *
   * @param position Reference receiving the raw position.
   * @return True if the position is read successfully, otherwise false.
   */
  bool readPosition(
    uint16_t & position);

  /**
   * @brief Writes a raw target position to the servo.
   *
   * @param position Raw target position.
   * @return True if the position is written successfully, otherwise false.
   */
  bool writePosition(
    uint16_t position);

  /**
   * @brief Enables or disables servo torque.
   *
   * @param enabled True to enable torque, false to disable it.
   * @return True if the operation succeeds, otherwise false.
   */
  bool setTorqueEnabled(
    bool enabled);

private:
  uint8_t id_;
  HiwonderBus & bus_;
};

}  // namespace hiwonder
