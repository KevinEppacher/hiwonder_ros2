#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/registers.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"

namespace
{

constexpr uint32_t kBaudRate = 1000000;

constexpr double kTicksPerRevolution = 4096.0;
constexpr double kDegreesPerRevolution = 360.0;

constexpr double kMovementDegrees = -10.0;

constexpr uint16_t kMinPosition = 0;
constexpr uint16_t kMaxPosition = 4095;

constexpr uint16_t kMoveTimeMs = 1000;
constexpr uint16_t kMoveSpeed = 0;

constexpr std::array<uint8_t, 6> kServoIds{
    1,
    2,
    3,
    4,
    5,
    6
};

uint16_t addDegrees(
    uint16_t position,
    double degrees)
{
    const auto delta = static_cast<int>(
        std::lround(
            degrees *
            kTicksPerRevolution /
            kDegreesPerRevolution));

    const int target =
        static_cast<int>(position) + delta;

    return static_cast<uint16_t>(
        std::clamp(
            target,
            static_cast<int>(kMinPosition),
            static_cast<int>(kMaxPosition)));
}

void writeWord(
    uint8_t* output,
    uint16_t value)
{
    output[0] = static_cast<uint8_t>(value & 0xFF);
    output[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr
            << "Usage: hiwonder-test-move <port>\n"
            << "Example: hiwonder-test-move /dev/ttyACM0\n";

        return 1;
    }

    const std::string port = argv[1];

    try {
        hiwonder::SerialPort serial(port, kBaudRate);
        serial.open();

        hiwonder::HiwonderBus bus(serial);

        std::array<uint16_t, kServoIds.size()> current_positions{};

        for (std::size_t i = 0; i < kServoIds.size(); ++i) {
            if (!bus.readWord(
                    kServoIds[i],
                    hiwonder::reg::kCurrentPosition,
                    current_positions[i]))
            {
                std::cerr
                    << "Failed to read motor "
                    << static_cast<int>(kServoIds[i])
                    << ".\n";

                return 1;
            }
        }

        std::array<uint16_t, kServoIds.size()> target_positions{};

        for (std::size_t i = 0; i < kServoIds.size(); ++i) {
            target_positions[i] = addDegrees(
                current_positions[i],
                kMovementDegrees);

            std::cout
                << "Motor "
                << static_cast<int>(kServoIds[i])
                << ": "
                << current_positions[i]
                << " -> "
                << target_positions[i]
                << " ticks ("
                << kMovementDegrees
                << " deg)\n";
        }

        /*
         * Each servo receives:
         *
         * target position: 2 bytes
         * move time:       2 bytes
         * move speed:      2 bytes
         */
        constexpr std::size_t kBytesPerServo = 6;

        std::array<
            uint8_t,
            kServoIds.size() * kBytesPerServo
        > data{};

        for (std::size_t i = 0; i < kServoIds.size(); ++i) {
            uint8_t* servo_data =
                data.data() + i * kBytesPerServo;

            writeWord(
                servo_data + 0,
                target_positions[i]);

            writeWord(
                servo_data + 2,
                kMoveTimeMs);

            writeWord(
                servo_data + 4,
                kMoveSpeed);
        }

        std::cout
            << "\nMoving all motors by "
            << kMovementDegrees
            << " degrees...\n";

        if (!bus.syncWrite(
                hiwonder::reg::kTargetPosition,
                kBytesPerServo,
                kServoIds.data(),
                data.data(),
                static_cast<uint8_t>(kServoIds.size())))
        {
            std::cerr << "Failed to send movement command.\n";
            return 1;
        }

        std::cout << "Command sent successfully.\n";

    } catch (const std::exception& exception) {
        std::cerr
            << "Error: "
            << exception.what()
            << '\n';

        return 1;
    }

    return 0;
}