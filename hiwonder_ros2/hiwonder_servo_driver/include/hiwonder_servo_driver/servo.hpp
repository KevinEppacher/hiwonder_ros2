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
 * @brief Provides a high-level interface for a single HiWonder servo.
 */
class Servo
{
public:
  /**
   * @brief Constructs a servo interface.
   *
   * @param bus HiWonder bus used for communication.
   * @param id Servo ID.
   */
  Servo(HiwonderBus & bus, uint8_t id);

  /**
   * @brief Returns the current servo ID.
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
   * @brief Changes the persistent servo ID.
   *
   * @param new_id New servo ID.
   * @return True if the ID is changed successfully, otherwise false.
   */
  bool setId(uint8_t new_id);

  /**
   * @brief Sets the persistent maximum servo torque.
   *
   * @param permille Maximum torque in permille.
   * @return True if the configuration is written successfully, otherwise false.
   */
  bool setMaxTorque(uint16_t permille);

  /**
   * @brief Enables or disables servo torque.
   *
   * @param enabled True to enable torque, false to disable torque.
   * @return True if the command succeeds, otherwise false.
   */
  bool setTorqueEnabled(bool enabled);

private:
  HiwonderBus & bus_;
  uint8_t id_;

  /**
   * @brief Writes data to a persistent servo register.
   *
   * The non-volatile register area is unlocked before writing and locked again
   * after the operation.
   *
   * @param address Start address of the persistent register.
   * @param data Pointer to the data to write.
   * @param length Number of bytes to write.
   * @return True if the persistent write succeeds, otherwise false.
   */
  bool writePersistent(
    uint8_t address,
    const uint8_t * data,
    uint8_t length);
};

}  // namespace hiwonder
