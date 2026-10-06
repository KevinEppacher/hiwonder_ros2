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

#include <array>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/registers.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"

namespace
{

void printBytes(
  const std::array<uint8_t, 2> & data)
{
  std::cout
    << "Raw bytes: 0x"
    << std::hex
    << std::setw(2)
    << std::setfill('0')
    << static_cast<int>(data[0])
    << " 0x"
    << std::setw(2)
    << static_cast<int>(data[1])
    << std::dec
    << '\n';
}

uint16_t decodeWord(
  const std::array<uint8_t, 2> & data)
{
  return
    static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8);
}

int16_t decodeSignMagnitude(
  uint16_t raw_value)
{
  const auto magnitude =
    static_cast<int16_t>(
    raw_value & 0x7FFFU);

  return (raw_value & 0x8000U) != 0U ?
         -magnitude :
         magnitude;
}

void printPositionRegister(
  const std::string & name,
  uint8_t address,
  const std::array<uint8_t, 2> & data)
{
  const uint16_t raw_value =
    decodeWord(data);

  std::cout
    << name
    << " register 0x"
    << std::hex
    << std::setw(2)
    << std::setfill('0')
    << static_cast<int>(address)
    << std::dec
    << '\n';

  printBytes(data);

  std::cout
    << "Raw uint16: "
    << raw_value
    << '\n';

  std::cout
    << "Sign-magnitude decode: "
    << decodeSignMagnitude(raw_value)
    << "\n\n";
}

}  // namespace

int main(
  int argc,
  char ** argv)
{
  if (argc != 3) {
    std::cerr
      << "Usage: hiwonder-position-debug <port> <servo_id>\n";

    return 1;
  }

  const std::string port = argv[1];

  int servo_id{};

  try {
    servo_id = std::stoi(argv[2]);
  } catch (const std::exception &) {
    std::cerr << "Invalid servo ID\n";
    return 1;
  }

  if (servo_id < 0 || servo_id > 253) {
    std::cerr << "Servo ID must be between 0 and 253\n";
    return 1;
  }

  try {
    hiwonder::SerialPort serial(port);
    serial.open();

    hiwonder::HiwonderBus bus(serial);

    const auto id =
      static_cast<uint8_t>(servo_id);

    std::cout
      << "Position register debug\n"
      << "Port: " << port << '\n'
      << "Servo ID: " << servo_id
      << "\n\n";

    if (!bus.ping(id)) {
      std::cerr << "Servo did not respond\n";
      return 1;
    }

    std::array<uint8_t, 1> operating_mode{};

    if (!bus.read(
        id,
        0x21,
        operating_mode))
    {
      std::cerr << "Failed to read operating mode\n";
      return 1;
    }

    std::cout
      << "Operating mode register 0x21: "
      << static_cast<int>(operating_mode[0])
      << "\n\n";

    std::array<uint8_t, 2> current_position{};

    if (!bus.read(
        id,
        hiwonder::reg::kCurrentPosition,
        current_position))
    {
      std::cerr << "Failed to read current position\n";
      return 1;
    }

    std::array<uint8_t, 2> target_position{};

    if (!bus.read(
        id,
        hiwonder::reg::kTargetPosition,
        target_position))
    {
      std::cerr << "Failed to read target position\n";
      return 1;
    }

    printPositionRegister(
      "Current position",
      hiwonder::reg::kCurrentPosition,
      current_position);

    printPositionRegister(
      "Target position",
      hiwonder::reg::kTargetPosition,
      target_position);

    std::cout
      << "Difference (decoded target - current): "
      << static_cast<int32_t>(
        decodeSignMagnitude(
          decodeWord(target_position))) -
      static_cast<int32_t>(
        decodeSignMagnitude(
          decodeWord(current_position)))
      << '\n';

  } catch (const std::exception & e) {
    std::cerr
      << "Error: "
      << e.what()
      << '\n';

    return 1;
  }

  return 0;
}
