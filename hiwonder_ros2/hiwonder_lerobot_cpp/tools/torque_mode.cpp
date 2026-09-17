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

#include <iostream>
#include <string>
#include "lerobot_cpp/robot.hpp"

#include "hiwonder_lerobot_cpp/follower.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr
      << "Usage: lerobot-torque-mode <port> <on|off>\n"
      << "\n"
      << "  off  Disable torque output for manual guiding\n"
      << "  on   Enable torque output for position control\n"
      << "\n"
      << "Example:\n"
      << "  lerobot-torque-mode /dev/ttyACM0 off\n";

    return 1;
  }

  const std::string port = argv[1];
  const std::string mode = argv[2];

  if (mode != "on" && mode != "off") {
    std::cerr
      << "Invalid mode: "
      << mode
      << ". Use 'on' or 'off'.\n";

    return 1;
  }

  const bool enable_torque = mode == "on";

  hiwonder::Follower robot;
  lerobot::RobotConfig config;
  config.port = port;
  config.baud_rate = 1000000;

  if(!robot.configure(config)){
    std::cerr<<"Failed to configure robot\n";
    return 1;
  }

  if (!robot.connect()) {
    std::cerr << "Failed to connect to robot.\n";
    return 1;
  }

  if (!robot.setTorqueEnabled(enable_torque)) {
    std::cerr << "Failed to set torque mode.\n";
    return 1;
  }

  std::cout
    << (enable_torque ?
    "Position control enabled.\n" :
    "Guiding mode enabled.\n");

  return 0;
}