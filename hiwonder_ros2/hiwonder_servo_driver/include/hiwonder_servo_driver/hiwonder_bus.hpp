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

#include <span>

#include <cstddef>
#include <cstdint>

#include "hiwonder_servo_driver/serial_port.hpp"

namespace hiwonder
{

/**
 * @brief Provides low-level communication with HiWonder bus servos.
 */
class HiwonderBus
{
public:
  /**
   * @brief Constructs a HiWonder servo bus.
   *
   * @param serial Serial port used for communication with the servos.
   */
  explicit HiwonderBus(SerialPort & serial);

  /**
   * @brief Checks whether a servo responds on the bus.
   *
   * @param id Servo ID.
   * @return True if a valid response is received, otherwise false.
   */
  bool ping(uint8_t id);

  /**
   * @brief Reads bytes from a servo register.
   *
   * @param id Servo ID.
   * @param address Start address of the register to read.
   * @param data Buffer receiving the register data.
   * @return True if the read operation succeeds, otherwise false.
   */
  bool read(
    uint8_t id,
    uint8_t address,
    std::span<uint8_t> data);

  /**
   * @brief Reads a 16-bit value from a servo register.
   *
   * @param id Servo ID.
   * @param address Start address of the register to read.
   * @param value Reference receiving the decoded 16-bit value.
   * @return True if the read operation succeeds, otherwise false.
   */
  bool readWord(
    uint8_t id,
    uint8_t address,
    uint16_t & value);

  /**
   * @brief Writes bytes to a servo register.
   *
   * @param id Servo ID.
   * @param address Start address of the register to write.
   * @param data Data to write to the register.
   * @return True if the write operation succeeds, otherwise false.
   */
  bool write(
    uint8_t id,
    uint8_t address,
    std::span<const uint8_t> data);

  /**
   * @brief Writes a 16-bit value to a servo register.
   *
   * @param id Servo ID.
   * @param address Start address of the register to write.
   * @param value 16-bit value to write.
   * @return True if the write operation succeeds, otherwise false.
   */
  bool writeWord(
    uint8_t id,
    uint8_t address,
    uint16_t value);

  /**
   * @brief Writes data to multiple servos using a broadcast SYNC_WRITE command.
   *
   * @param address Start address of the register to write.
   * @param data_length Number of data bytes written per servo.
   * @param ids Servo IDs receiving the data.
   * @param data Contiguous data for all specified servos.
   * @return True if the packet is valid and transmitted, otherwise false.
   */
  bool syncWrite(
    uint8_t address,
    uint8_t data_length,
    std::span<const uint8_t> ids,
    std::span<const uint8_t> data);

  /**
   * @brief Calculates the HiWonder protocol checksum.
   *
   * @param data Bytes included in the checksum calculation.
   * @return Calculated 8-bit checksum.
   */
  static uint8_t checksum(
    std::span<const uint8_t> data);

private:
  static constexpr uint8_t kHeader = 0xFF;
  static constexpr uint8_t kPingInstruction = 0x01;
  static constexpr uint8_t kReadInstruction = 0x02;
  static constexpr uint8_t kWriteInstruction = 0x03;
  static constexpr uint8_t kSyncWriteInstruction = 0x83;
  static constexpr uint8_t kBroadcastId = 0xFE;

  static constexpr std::size_t kPacketBufferSize = 256;
  static constexpr std::size_t kPacketOverhead = 6;
  static constexpr std::size_t kMaxParameterCount =
    kPacketBufferSize - kPacketOverhead;

  SerialPort & serial_;

  /**
   * @brief Builds a HiWonder protocol packet.
   *
   * @param output Buffer receiving the encoded packet.
   * @param id Target servo ID or broadcast ID.
   * @param instruction Protocol instruction.
   * @param parameters Instruction parameters.
   * @return Number of encoded packet bytes, or zero if the packet is invalid.
   */
  static std::size_t buildPacket(
    std::span<uint8_t> output,
    uint8_t id,
    uint8_t instruction,
    std::span<const uint8_t> parameters);

  /**
   * @brief Receives and validates a status packet from a servo.
   *
   * @param expected_id Expected servo ID.
   * @param parameters Buffer receiving the status packet parameters.
   * @param timeout_ms Maximum time to wait for the response in milliseconds.
   * @return True if a valid status packet is received, otherwise false.
   */
  bool receiveStatusPacket(
    uint8_t expected_id,
    std::span<uint8_t> parameters,
    uint32_t timeout_ms);
};

}  // namespace hiwonder
