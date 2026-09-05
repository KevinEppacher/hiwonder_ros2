#include "hiwonder_servo_driver/hiwonder_bus.hpp"

#include <array>
#include <chrono>

namespace hiwonder
{

HiwonderBus::HiwonderBus(SerialPort& serial)
    : serial_(serial)
{
}

uint8_t HiwonderBus::checksum(
    const uint8_t* data,
    std::size_t size)
{
    uint16_t sum = 0;

    for (std::size_t i = 0; i < size; ++i) {
        sum += data[i];
    }

    return static_cast<uint8_t>(~sum & 0xFF);
}

std::size_t HiwonderBus::buildPacket(
    uint8_t* output,
    uint8_t id,
    uint8_t instruction,
    const uint8_t* parameters,
    std::size_t parameter_count)
{
    const uint8_t length =
        static_cast<uint8_t>(parameter_count + 2);

    output[0] = kHeader;
    output[1] = kHeader;
    output[2] = id;
    output[3] = length;
    output[4] = instruction;

    for (std::size_t i = 0; i < parameter_count; ++i) {
        output[5 + i] = parameters[i];
    }

    std::array<uint8_t, 256> checksum_data{};

    checksum_data[0] = id;
    checksum_data[1] = length;
    checksum_data[2] = instruction;

    for (std::size_t i = 0; i < parameter_count; ++i) {
        checksum_data[3 + i] = parameters[i];
    }

    output[5 + parameter_count] = checksum(
        checksum_data.data(),
        3 + parameter_count);

    return 6 + parameter_count;
}

bool HiwonderBus::receiveStatusPacket(
    uint8_t expected_id,
    uint8_t* parameters,
    std::size_t expected_parameter_count,
    uint32_t timeout_ms)
{
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    bool first_header_received = false;
    bool header_received = false;

    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t byte = 0;

        if (serial_.read(&byte, 1, 5) == 0) {
            continue;
        }

        if (!first_header_received) {
            first_header_received = (byte == kHeader);
            continue;
        }

        if (byte == kHeader) {
            header_received = true;
            break;
        }

        first_header_received = (byte == kHeader);
    }

    if (!header_received) {
        return false;
    }

    // Read ID and LENGTH first.

    std::array<uint8_t, 2> header{};

    std::size_t received = 0;

    while (
        received < header.size() &&
        std::chrono::steady_clock::now() < deadline)
    {
        received += serial_.read(
            header.data() + received,
            header.size() - received,
            5);
    }

    if (received != header.size()) {
        return false;
    }

    const uint8_t id = header[0];
    const uint8_t length = header[1];

    if (id != expected_id) {
        return false;
    }

    // LENGTH contains:
    //
    // ERROR + PARAMETERS + CHECKSUM
    //
    // Therefore:
    //
    // parameter_count = LENGTH - 2

    if (length < 2) {
        return false;
    }

    const std::size_t parameter_count = length - 2;

    if (parameter_count != expected_parameter_count) {
        return false;
    }

    // Read ERROR + PARAMETERS + CHECKSUM.

    std::array<uint8_t, 256> payload{};

    received = 0;

    while (
        received < length &&
        std::chrono::steady_clock::now() < deadline)
    {
        received += serial_.read(
            payload.data() + received,
            length - received,
            5);
    }

    if (received != length) {
        return false;
    }

    const uint8_t error = payload[0];
    const uint8_t received_checksum = payload[length - 1];

    std::array<uint8_t, 256> checksum_data{};

    checksum_data[0] = id;
    checksum_data[1] = length;
    checksum_data[2] = error;

    for (std::size_t i = 0; i < parameter_count; ++i) {
        checksum_data[3 + i] = payload[1 + i];

        if (parameters != nullptr) {
            parameters[i] = payload[1 + i];
        }
    }

    const uint8_t expected_checksum = checksum(
        checksum_data.data(),
        3 + parameter_count);

    if (received_checksum != expected_checksum) {
        return false;
    }

    return error == 0;
}

bool HiwonderBus::ping(uint8_t id)
{
    std::array<uint8_t, 8> packet{};

    const std::size_t packet_size = buildPacket(
        packet.data(),
        id,
        kPingInstruction,
        nullptr,
        0);

    serial_.flushInput();
    serial_.write(packet.data(), packet_size);

    return receiveStatusPacket(
        id,
        nullptr,
        0,
        50);
}

bool HiwonderBus::read(
    uint8_t id,
    uint8_t address,
    uint8_t length,
    uint8_t* data)
{
    if (length == 0 || data == nullptr) {
        return false;
    }

    const std::array<uint8_t, 2> parameters{
        address,
        length
    };

    std::array<uint8_t, 16> packet{};

    const std::size_t packet_size = buildPacket(
        packet.data(),
        id,
        kReadInstruction,
        parameters.data(),
        parameters.size());

    serial_.flushInput();
    serial_.write(packet.data(), packet_size);

    return receiveStatusPacket(
        id,
        data,
        length,
        50);
}

bool HiwonderBus::readWord(
    uint8_t id,
    uint8_t address,
    uint16_t& value)
{
    std::array<uint8_t, 2> data{};

    if (!read(
            id,
            address,
            static_cast<uint8_t>(data.size()),
            data.data()))
    {
        return false;
    }

    value =
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);

    return true;
}

bool HiwonderBus::write(
    uint8_t id,
    uint8_t address,
    const uint8_t* data,
    uint8_t length)
{
    if (length == 0 || data == nullptr) {
        return false;
    }

    std::array<uint8_t, 256> parameters{};

    parameters[0] = address;

    for (std::size_t i = 0; i < length; ++i) {
        parameters[1 + i] = data[i];
    }

    std::array<uint8_t, 256> packet{};

    const std::size_t packet_size = buildPacket(
        packet.data(),
        id,
        kWriteInstruction,
        parameters.data(),
        static_cast<std::size_t>(length) + 1);

    serial_.flushInput();
    serial_.write(packet.data(), packet_size);

    return receiveStatusPacket(
        id,
        nullptr,
        0,
        50);
}

bool HiwonderBus::writeWord(
    uint8_t id,
    uint8_t address,
    uint16_t value)
{
    const std::array<uint8_t, 2> data{
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };

    return write(
        id,
        address,
        data.data(),
        static_cast<uint8_t>(data.size()));
}

bool HiwonderBus::syncWrite(
    uint8_t address,
    uint8_t data_length,
    const uint8_t* ids,
    const uint8_t* data,
    uint8_t count)
{
    if (
        data_length == 0 ||
        ids == nullptr ||
        data == nullptr ||
        count == 0)
    {
        return false;
    }

    std::array<uint8_t, 256> parameters{};

    std::size_t index = 0;

    parameters[index++] = address;
    parameters[index++] = data_length;

    for (uint8_t i = 0; i < count; ++i) {
        parameters[index++] = ids[i];

        for (uint8_t j = 0; j < data_length; ++j) {
            parameters[index++] =
                data[static_cast<std::size_t>(i) * data_length + j];
        }
    }

    std::array<uint8_t, 256> packet{};

    const std::size_t packet_size = buildPacket(
        packet.data(),
        kBroadcastId,
        kSyncWriteInstruction,
        parameters.data(),
        index);

    serial_.flushInput();
    serial_.write(packet.data(), packet_size);

    // Broadcast packets do not return a status packet.
    return true;
}

}