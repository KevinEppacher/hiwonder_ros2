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

#include "hiwonder_lerobot_cpp/robot.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "hiwonder_servo_driver/registers.hpp"

namespace hiwonder
{

bool LeRobot::configure(
  const lerobot::RobotConfig & config)
{
  if (isConnected()) {
    return false;
  }

  port_ = config.port;
  baud_rate_ = config.baud_rate;
  return true;
}

bool LeRobot::connect()
{
  if (isConnected()) {
    return true;
  }

  try {
    serial_ =
      std::make_unique<SerialPort>(
      port_,
      baud_rate_);

    serial_->open();

    bus_ =
      std::make_unique<HiwonderBus>(
      *serial_);

    motors_.clear();

    for (const uint8_t id : motor_ids_) {
      motors_.push_back(
        std::make_unique<Motor>(
          id,
          *bus_));
    }

    for (auto & motor : motors_) {
      if (!motor->ping()) {
        disconnect();
        return false;
      }
    }

    return true;
  } catch (...) {
    disconnect();
    return false;
  }
}

void LeRobot::disconnect()
{
  motors_.clear();

  bus_.reset();

  if (serial_) {
    serial_->close();
    serial_.reset();
  }
}

bool LeRobot::isConnected() const noexcept
{
  return
    serial_ &&
    serial_->isOpen() &&
    bus_;
}

Motor & LeRobot::motor(
  std::size_t index)
{
  if (index >= motors_.size()) {
    throw std::out_of_range(
      "Motor index out of range");
  }

  return *motors_[index];
}

const Motor & LeRobot::motor(
  std::size_t index) const
{
  if (index >= motors_.size()) {
    throw std::out_of_range(
      "Motor index out of range");
  }

  return *motors_[index];
}

std::size_t LeRobot::motorCount() const noexcept
{
  return motor_ids_.size();
}

bool LeRobot::setTorqueEnabled(
  bool enabled)
{
  if (!isConnected()) {
    return false;
  }

  for (auto & motor : motors_) {
    if (!motor->setTorqueEnabled(enabled)) {
      return false;
    }
  }

  return true;
}

void LeRobot::addMotor(
  uint8_t id)
{
  if (isConnected()) {
    throw std::logic_error(
      "Cannot add motors while robot is connected");
  }

  if (std::find(motor_ids_.begin(), motor_ids_.end(), id) != motor_ids_.end()) {
    throw std::logic_error("Cannot add duplicate motor ID");
  }

  motor_ids_.push_back(id);
}

bool LeRobot::writePositions(
  const std::vector<uint16_t> & positions)
{
  return writePositions(positions, 0, 0);
}

bool LeRobot::writePositions(
  const std::vector<uint16_t> & positions,
  uint16_t move_time_ms,
  uint16_t move_speed)
{
  if (!isConnected()) {
    return false;
  }

  if (positions.size() != motors_.size()) {
    return false;
  }

  constexpr std::size_t kBytesPerMotor = 6;

  std::vector<uint8_t> ids;
  std::vector<uint8_t> data(
    motors_.size() * kBytesPerMotor);

  ids.reserve(motors_.size());

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    ids.push_back(motors_[i]->id());

    uint8_t * motor_data =
      data.data() + i * kBytesPerMotor;

    motor_data[0] =
      static_cast<uint8_t>(positions[i] & 0xFF);
    motor_data[1] =
      static_cast<uint8_t>((positions[i] >> 8) & 0xFF);

    motor_data[2] =
      static_cast<uint8_t>(move_time_ms & 0xFF);
    motor_data[3] =
      static_cast<uint8_t>((move_time_ms >> 8) & 0xFF);

    motor_data[4] =
      static_cast<uint8_t>(move_speed & 0xFF);
    motor_data[5] =
      static_cast<uint8_t>((move_speed >> 8) & 0xFF);
  }

  return bus_->syncWrite(
    reg::kTargetPosition,
    static_cast<uint8_t>(kBytesPerMotor),
    ids,
    data);
}

bool LeRobot::readPositions(
  std::vector<uint16_t> & positions)
{
  if (!isConnected()) {
    return false;
  }

  positions.resize(motors_.size());

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (!motors_[i]->readPosition(positions[i])) {
      return false;
    }
  }

  return true;
}

}  // namespace hiwonder
