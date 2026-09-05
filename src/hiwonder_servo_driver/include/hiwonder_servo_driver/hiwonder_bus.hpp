#pragma once

#include <cstddef>
#include <cstdint>

#include "hiwonder_servo_driver/serial_port.hpp"

namespace hiwonder
{

class HiwonderBus
{
public:
    explicit HiwonderBus(SerialPort& serial);

    bool ping(uint8_t id);

    bool read(
        uint8_t id,
        uint8_t address,
        uint8_t length,
        uint8_t* data);

    bool readWord(
        uint8_t id,
        uint8_t address,
        uint16_t& value);

    bool write(
        uint8_t id,
        uint8_t address,
        const uint8_t* data,
        uint8_t length);

    bool writeWord(
        uint8_t id,
        uint8_t address,
        uint16_t value);

    bool syncWrite(
        uint8_t address,
        uint8_t data_length,
        const uint8_t* ids,
        const uint8_t* data,
        uint8_t count);

private:
    static constexpr uint8_t kHeader = 0xFF;
    static constexpr uint8_t kPingInstruction = 0x01;
    static constexpr uint8_t kReadInstruction = 0x02;
    static constexpr uint8_t kWriteInstruction = 0x03;
    static constexpr uint8_t kSyncWriteInstruction = 0x83;
    static constexpr uint8_t kBroadcastId = 0xFE;

    SerialPort& serial_;

    static uint8_t checksum(
        const uint8_t* data,
        std::size_t size);

    static std::size_t buildPacket(
        uint8_t* output,
        uint8_t id,
        uint8_t instruction,
        const uint8_t* parameters,
        std::size_t parameter_count);

    bool receiveStatusPacket(
        uint8_t expected_id,
        uint8_t* parameters,
        std::size_t expected_parameter_count,
        uint32_t timeout_ms);
};

}