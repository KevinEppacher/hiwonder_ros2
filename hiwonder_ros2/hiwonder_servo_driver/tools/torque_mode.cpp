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
#include <iostream>
#include <string>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"
#include "hiwonder_servo_driver/servo.hpp"

namespace
{

constexpr uint32_t kBaudRate = 1000000;

constexpr std::array<uint8_t, 6> kServoIds{
  1,
  2,
  3,
  4,
  5,
  6
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr
            << "Usage: hiwonder-torque-off <port> <on|off>\n"
            << "\n"
            << "  off  Disable torque output for manual guiding\n"
            << "  on   Enable torque output for position control\n"
            << "\n"
            << "Example:\n"
            << "  hiwonder-torque-off /dev/ttyACM0 off\n";

    return 1;
  }

  const std::string port = argv[1];
  const std::string mode = argv[2];

  bool enable_torque;

  if (mode == "on") {
    enable_torque = true;
  } else if (mode == "off") {
    enable_torque = false;
  } else {
    std::cerr
            << "Invalid mode: "
            << mode
            << ". Use 'on' or 'off'.\n";

    return 1;
  }

  try {
    hiwonder::SerialPort serial(port, kBaudRate);
    serial.open();

    hiwonder::HiwonderBus bus(serial);

    for (const uint8_t id : kServoIds) {
      hiwonder::Servo servo(bus, id);

      std::cout
                << "Motor "
                << static_cast<int>(id)
                << ": ";

      if (!servo.ping()) {
        std::cout << "not found\n";
        continue;
      }

      if (!servo.setTorqueEnabled(enable_torque)) {
        std::cout << "failed\n";
        return 1;
      }

      std::cout
                << (enable_torque ?
      "torque enabled" :
      "torque disabled")
                << '\n';
    }

    std::cout << '\n';

    if (enable_torque) {
      std::cout << "Position control enabled.\n";
    } else {
      std::cout << "Guiding mode enabled.\n";
    }
  } catch (const std::exception & exception) {
    std::cerr
            << "Error: "
            << exception.what()
            << '\n';

    return 1;
  }

  return 0;
}
