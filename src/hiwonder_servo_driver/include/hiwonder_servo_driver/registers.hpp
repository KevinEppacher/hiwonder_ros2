#pragma once

#include <cstdint>

namespace hiwonder::reg
{

constexpr uint8_t kFirmwareMain = 0x00;
constexpr uint8_t kFirmwareSub = 0x01;
constexpr uint8_t kModelLow = 0x03;
constexpr uint8_t kModelHigh = 0x04;
constexpr uint8_t kId = 0x05;
constexpr uint8_t kBaudRate = 0x06;

constexpr uint8_t kCurrentPosition = 0x38;
constexpr uint8_t kCurrentSpeed = 0x3A;
constexpr uint8_t kCurrentLoad = 0x3C;
constexpr uint8_t kCurrentVoltage = 0x3E;
constexpr uint8_t kCurrentTemperature = 0x3F;
constexpr uint8_t kMoving = 0x42;
constexpr uint8_t kCurrentCurrent = 0x45;
constexpr uint8_t kNvsLock = 0x37;
constexpr uint8_t kTargetPosition = 0x2A;
constexpr uint8_t kMoveTime = 0x2C;
constexpr uint8_t kMoveSpeed = 0x2E;

}