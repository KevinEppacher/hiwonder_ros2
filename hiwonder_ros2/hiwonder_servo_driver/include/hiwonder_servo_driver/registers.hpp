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

// Register entries for the HiWonder servo driver from:
// https://github.com/Hiwonder-official/hiwonder-servo-sdk/blob/main/HX-30HM%E5%AF%84%E5%AD%98%E5%99%A8%E5%88%97%E8%A1%A8%E5%8F%8A%E5%8F%82%E6%95%B0.xls
// Not every register is implemented in this driver, only the ones that are used.

namespace hiwonder::reg
{

// ROM / NVS configuration registers

constexpr uint8_t kFirmwareMain = 0x00;
constexpr uint8_t kFirmwareSub = 0x01;
constexpr uint8_t kModelLow = 0x03;
constexpr uint8_t kModelHigh = 0x04;

constexpr uint8_t kId = 0x05;
constexpr uint8_t kBaudRate = 0x06;
constexpr uint8_t kResponseLevel = 0x08;

constexpr uint8_t kTemperatureLimit = 0x0D;
constexpr uint8_t kOvervoltageLimit = 0x0E;
constexpr uint8_t kUndervoltageLimit = 0x0F;
constexpr uint8_t kMaxTorque = 0x10;

constexpr uint8_t kProtectionControl = 0x13;
constexpr uint8_t kLedAlarm = 0x14;

constexpr uint8_t kPositionP = 0x15;
constexpr uint8_t kPositionD = 0x16;
constexpr uint8_t kPositionI = 0x17;

constexpr uint8_t kMinimumStartupTorque = 0x18;

constexpr uint8_t kClockwiseDeadzone = 0x1A;
constexpr uint8_t kCounterclockwiseDeadzone = 0x1B;

constexpr uint8_t kOvercurrentLimit = 0x1C;
constexpr uint8_t kPositionCorrection = 0x1F;

constexpr uint8_t kOperatingMode = 0x21;

constexpr uint8_t kOverloadTorqueLimit = 0x22;
constexpr uint8_t kOverloadTriggerTime = 0x23;
constexpr uint8_t kOverloadTorqueThreshold = 0x24;

constexpr uint8_t kSpeedP = 0x25;
constexpr uint8_t kOvercurrentProtectionTime = 0x26;
constexpr uint8_t kSpeedI = 0x27;


// SRAM control registers

constexpr uint8_t kTorqueEnable = 0x28;
constexpr uint8_t kAcceleration = 0x29;

constexpr uint8_t kTargetPosition = 0x2A;
constexpr uint8_t kPwmSpeed = 0x2C;
constexpr uint8_t kMoveSpeed = 0x2E;

constexpr uint8_t kNvsLock = 0x37;


// SRAM feedback registers

constexpr uint8_t kCurrentPosition = 0x38;
constexpr uint8_t kCurrentSpeed = 0x3A;
constexpr uint8_t kCurrentLoad = 0x3C;
constexpr uint8_t kCurrentVoltage = 0x3E;
constexpr uint8_t kCurrentTemperature = 0x3F;

constexpr uint8_t kAsyncWriteFlag = 0x40;
constexpr uint8_t kStatus = 0x41;
constexpr uint8_t kMoving = 0x42;

constexpr uint8_t kCurrentCurrent = 0x45;

}  // namespace hiwonder::reg
