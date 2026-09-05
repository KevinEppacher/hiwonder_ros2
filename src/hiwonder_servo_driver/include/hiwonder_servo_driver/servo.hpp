#pragma once

#include <cstdint>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"

namespace hiwonder
{

class Servo
{
public:
    Servo(HiwonderBus& bus, uint8_t id);

    [[nodiscard]] uint8_t id() const noexcept;

    bool ping();

    bool setId(uint8_t new_id);

private:
    HiwonderBus& bus_;
    uint8_t id_;

    bool writePersistent(
        uint8_t address,
        const uint8_t* data,
        uint8_t length);
};

}