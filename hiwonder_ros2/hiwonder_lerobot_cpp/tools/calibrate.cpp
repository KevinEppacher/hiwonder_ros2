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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "hiwonder_lerobot_cpp/follower.hpp"

namespace
{

constexpr auto kUpdatePeriod =
  std::chrono::milliseconds(100);

struct Joint
{
  const char * name;
  uint8_t servo_id;
};

struct JointCalibration
{
  uint16_t homing_offset;
  uint16_t range_min;
  uint16_t position;
  uint16_t range_max;
};

constexpr std::array<Joint, 6> kJoints{{
  {"shoulder_pan", 1},
  {"shoulder_lift", 2},
  {"elbow_flex", 3},
  {"wrist_flex", 4},
  {"wrist_roll", 5},
  {"gripper", 6}
}};

void clearTerminal()
{
  std::cout << "\033[2J\033[H";
}

void waitForEnter()
{
  std::string input;
  std::getline(std::cin, input);
}

bool enterPressed()
{
  pollfd descriptor{};
  descriptor.fd = STDIN_FILENO;
  descriptor.events = POLLIN;

  const int result = poll(
    &descriptor,
    1,
    0);

  if (result <= 0) {
    return false;
  }

  if ((descriptor.revents & POLLIN) == 0) {
    return false;
  }

  std::string input;
  std::getline(std::cin, input);

  return true;
}

void printZeroPositions(
  const std::vector<uint16_t> & positions)
{
  clearTerminal();

  std::cout
    << "LeRobot Calibration\n"
    << "====================\n\n"
    << "Zero positions recorded.\n\n";

  std::cout
    << std::left
    << std::setw(18) << "NAME"
    << std::right
    << std::setw(10) << "OFFSET"
    << '\n';

  std::cout
    << "----------------------------\n";

  for (std::size_t i = 0; i < positions.size(); ++i) {
    std::cout
      << std::left
      << std::setw(18) << kJoints[i].name
      << std::right
      << std::setw(10) << positions[i]
      << '\n';
  }
}

void printStates(
  const std::vector<JointCalibration> & calibration)
{
  clearTerminal();

  std::cout
    << "LeRobot Calibration\n"
    << "====================\n\n"
    << "Recording joint ranges.\n"
    << "Move every joint through its full range of motion.\n"
    << "Press ENTER when finished.\n\n";

  std::cout
    << std::left
    << std::setw(18) << "NAME"
    << std::right
    << std::setw(10) << "OFFSET"
    << std::setw(8) << "MIN"
    << std::setw(8) << "POS"
    << std::setw(8) << "MAX"
    << '\n';

  std::cout
    << "----------------------------------------------------\n";

  for (std::size_t i = 0; i < calibration.size(); ++i) {
    std::cout
      << std::left
      << std::setw(18) << kJoints[i].name
      << std::right
      << std::setw(10) << calibration[i].homing_offset
      << std::setw(8) << calibration[i].range_min
      << std::setw(8) << calibration[i].position
      << std::setw(8) << calibration[i].range_max
      << '\n';
  }

  std::cout.flush();
}

void printCalibration(
  const std::vector<JointCalibration> & calibration)
{
  clearTerminal();

  std::cout
    << "LeRobot Calibration\n"
    << "====================\n\n"
    << "Calibration complete.\n\n";

  std::cout
    << std::left
    << std::setw(18) << "NAME"
    << std::right
    << std::setw(10) << "OFFSET"
    << std::setw(8) << "MIN"
    << std::setw(8) << "MAX"
    << std::setw(10) << "RANGE"
    << '\n';

  std::cout
    << "------------------------------------------------------\n";

  for (std::size_t i = 0; i < calibration.size(); ++i) {
    const uint32_t range =
      static_cast<uint32_t>(calibration[i].range_max) -
      static_cast<uint32_t>(calibration[i].range_min);

    std::cout
      << std::left
      << std::setw(18) << kJoints[i].name
      << std::right
      << std::setw(10) << calibration[i].homing_offset
      << std::setw(8) << calibration[i].range_min
      << std::setw(8) << calibration[i].range_max
      << std::setw(10) << range
      << '\n';
  }
}

bool recordZeroPositions(
  hiwonder::Follower & robot,
  std::vector<uint16_t> & positions)
{
  if (!robot.readPositions(positions)) {
    std::cerr << "Failed to read zero positions.\n";
    return false;
  }

  if (positions.size() != kJoints.size()) {
    std::cerr
      << "Unexpected motor count: "
      << positions.size()
      << '\n';

    return false;
  }

  return true;
}

bool recordRanges(
  hiwonder::Follower & robot,
  const std::vector<uint16_t> & homing_offsets,
  std::vector<JointCalibration> & calibration)
{
  std::vector<uint16_t> positions;

  if (!robot.readPositions(positions)) {
    std::cerr << "Failed to read initial motor positions.\n";
    return false;
  }

  if (positions.size() != homing_offsets.size()) {
    std::cerr << "Motor count does not match calibration data.\n";
    return false;
  }

  calibration.clear();
  calibration.reserve(positions.size());

  for (std::size_t i = 0; i < positions.size(); ++i) {
    calibration.push_back({
      homing_offsets[i],
      positions[i],
      positions[i],
      positions[i]
    });
  }

  while (true) {
    if (enterPressed()) {
      break;
    }

    if (!robot.readPositions(positions)) {
      std::cerr << "Failed to read motor positions.\n";
      return false;
    }

    for (std::size_t i = 0; i < positions.size(); ++i) {
      calibration[i].position =
        positions[i];

      calibration[i].range_min =
        std::min(
        calibration[i].range_min,
        positions[i]);

      calibration[i].range_max =
        std::max(
        calibration[i].range_max,
        positions[i]);
    }

    printStates(calibration);

    std::this_thread::sleep_for(
      kUpdatePeriod);
  }

  return true;
}

bool saveCalibration(
  const std::string & path,
  const std::vector<JointCalibration> & calibration)
{
  if (calibration.size() != kJoints.size()) {
    std::cerr
      << "Calibration data does not match joint count.\n";

    return false;
  }

  YAML::Node root;

  for (std::size_t i = 0; i < calibration.size(); ++i) {
    YAML::Node joint;

    joint["servo_id"] =
      static_cast<unsigned int>(kJoints[i].servo_id);

    joint["homing_offset"] =
      calibration[i].homing_offset;

    joint["range_min"] =
      calibration[i].range_min;

    joint["range_max"] =
      calibration[i].range_max;

    root[kJoints[i].name] = joint;
  }

  std::ofstream output(path);

  if (!output.is_open()) {
    std::cerr
      << "Failed to open calibration file: "
      << path
      << '\n';

    return false;
  }

  output << root;

  if (!output.good()) {
    std::cerr
      << "Failed to write calibration file: "
      << path
      << '\n';

    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr
      << "Usage:\n"
      << "  lerobot-calibrate <port> <calibration.yaml>\n"
      << "\n"
      << "Example:\n"
      << "  lerobot-calibrate /dev/ttyACM0 "
      << "/app/config/calibration.yaml\n";

    return 1;
  }

  const std::string port = argv[1];
  const std::string calibration_path = argv[2];

  hiwonder::Follower robot(port);

  std::cout
    << "LeRobot Calibration\n"
    << "====================\n\n"
    << "Connecting to robot...\n";

  if (!robot.connect()) {
    std::cerr << "Failed to connect to robot.\n";
    return 1;
  }

  std::cout
    << "Connected successfully.\n"
    << "Disabling motor torque...\n";

  if (!robot.setTorqueEnabled(false)) {
    std::cerr << "Failed to disable motor torque.\n";
    return 1;
  }

  std::cout
    << "Torque disabled.\n\n"
    << "Step 1: Zero Position\n"
    << "---------------------\n"
    << "Move the robot into its defined zero pose.\n"
    << "Press ENTER to record the zero positions.";

  waitForEnter();

  std::vector<uint16_t> homing_offsets;

  if (!recordZeroPositions(
      robot,
      homing_offsets))
  {
    return 1;
  }

  printZeroPositions(homing_offsets);

  std::cout
    << "\nStep 2: Range Calibration\n"
    << "-------------------------\n"
    << "Press ENTER to start recording.";

  waitForEnter();

  std::vector<JointCalibration> calibration;

  if (!recordRanges(
      robot,
      homing_offsets,
      calibration))
  {
    return 1;
  }

  printCalibration(calibration);

  std::cout
    << "\nCalibration file:\n"
    << calibration_path
    << "\n\n"
    << "Press ENTER to save calibration.";

  waitForEnter();

  if (!saveCalibration(
      calibration_path,
      calibration))
  {
    return 1;
  }

  std::cout
    << "\nCalibration saved successfully.\n";

  return 0;
}