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
#include <memory>
#include <string>
#include <vector>

#include "hiwonder_servo_driver/motor.hpp"
#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"
#include "lerobot_cpp/robot.hpp"

namespace hiwonder
{

class LeRobot : public lerobot::Robot
{
public:
  LeRobot() = default;

  explicit LeRobot(
    std::string port,
    uint32_t baud_rate = 1000000);

  ~LeRobot() override = default;

  bool configure(
    const lerobot::RobotConfig & config) override;

  bool connect() override;

  void disconnect() override;

  [[nodiscard]] bool isConnected() const noexcept override;

  Motor & motor(
    std::size_t index);

  const Motor & motor(
    std::size_t index) const;

  [[nodiscard]] std::size_t motorCount() const noexcept override;

  bool setTorqueEnabled(
    bool enabled) override;

  bool readPositions(
    std::vector<uint16_t> & positions) override;

  bool writePositions(
    const std::vector<uint16_t> & positions) override;

  bool writePositions(
    const std::vector<uint16_t> & positions,
    uint16_t move_time_ms,
    uint16_t move_speed);

protected:
  void addMotor(
    uint8_t id);

private:
  std::string port_;
  uint32_t baud_rate_{1000000};

  std::vector<uint8_t> motor_ids_;

  std::unique_ptr<SerialPort> serial_;
  std::unique_ptr<HiwonderBus> bus_;
  std::vector<std::unique_ptr<Motor>> motors_;
};

}  // namespace hiwonder