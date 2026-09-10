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

namespace hiwonder
{

/**
 * @brief Provides serial communication through a Linux serial device.
 */
class SerialPort
{
public:
  /**
   * @brief Constructs a serial port interface.
   *
   * @param device Path to the serial device.
   * @param baud_rate Communication baud rate.
   */
  explicit SerialPort(
    std::string device,
    uint32_t baud_rate = 1000000);

  /**
   * @brief Destroys the serial port and closes the device if necessary.
   */
  virtual ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief Opens and configures the serial device.
   */
  virtual void open();

  /**
   * @brief Closes the serial device.
   */
  virtual void close();

  /**
   * @brief Checks whether the serial device is currently open.
   *
   * @return True if the serial device is open, otherwise false.
   */
  [[nodiscard]] virtual bool isOpen() const noexcept;

  /**
   * @brief Discards unread data from the serial input buffer.
   */
  virtual void flushInput();

  /**
   * @brief Writes bytes to the serial device.
   *
   * @param data Pointer to the data to transmit.
   * @param size Number of bytes to transmit.
   */
  virtual void write(
    const uint8_t * data,
    std::size_t size);

  /**
   * @brief Reads bytes from the serial device.
   *
   * @param data Pointer to the destination buffer.
   * @param size Maximum number of bytes to read.
   * @param timeout_ms Maximum time to wait for data in milliseconds.
   * @return Number of bytes read.
   */
  virtual std::size_t read(
    uint8_t * data,
    std::size_t size,
    uint32_t timeout_ms);

private:
  std::string device_;
  uint32_t baud_rate_;
  int fd_{-1};
};

}  // namespace hiwonder
