#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/registers.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"

namespace
{

constexpr uint8_t kMinServoId = 1;
constexpr uint8_t kMaxServoId = 20;
constexpr uint32_t kBaudRate = 1000000;

void printServoInfo(
    hiwonder::HiwonderBus& bus,
    uint8_t id)
{
    uint8_t firmware_main = 0;
    uint8_t firmware_sub = 0;
    uint8_t configured_id = 0;
    uint8_t temperature = 0;

    uint16_t model = 0;
    uint16_t position = 0;
    uint8_t voltage_raw = 0;

    const bool firmware_main_ok = bus.read(
        id,
        hiwonder::reg::kFirmwareMain,
        1,
        &firmware_main);

    const bool firmware_sub_ok = bus.read(
        id,
        hiwonder::reg::kFirmwareSub,
        1,
        &firmware_sub);

    const bool model_ok = bus.readWord(
        id,
        hiwonder::reg::kModelLow,
        model);

    const bool id_ok = bus.read(
        id,
        hiwonder::reg::kId,
        1,
        &configured_id);

    const bool position_ok = bus.readWord(
        id,
        hiwonder::reg::kCurrentPosition,
        position);

    const bool voltage_ok = bus.read(
        id,
        hiwonder::reg::kCurrentVoltage,
        1,
        &voltage_raw);

    const bool temperature_ok = bus.read(
        id,
        hiwonder::reg::kCurrentTemperature,
        1,
        &temperature);

    std::cout << "Servo found at ID " << static_cast<int>(id) << '\n';

    if (firmware_main_ok && firmware_sub_ok) {
        std::cout
            << "  Firmware: "
            << static_cast<int>(firmware_main)
            << '.'
            << static_cast<int>(firmware_sub)
            << '\n';
    }

    if (model_ok) {
        std::cout
            << "  Model ID: "
            << model
            << '\n';
    }

    if (id_ok) {
        std::cout
            << "  Configured ID: "
            << static_cast<int>(configured_id)
            << '\n';
    }

    if (position_ok) {
        std::cout
            << "  Position: "
            << position
            << '\n';
    }

    if (voltage_ok) {
        std::cout
            << "  Voltage: "
            << static_cast<double>(voltage_raw) * 0.1
            << " V\n";
    }

    if (temperature_ok) {
        std::cout
            << "  Temperature: "
            << static_cast<int>(temperature)
            << " C\n";
    }
}

}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr
            << "Usage: hiwonder-scan <port>\n"
            << "Example: hiwonder-scan /dev/ttyACM0\n";

        return 1;
    }

    const std::string port = argv[1];

    try {
        hiwonder::SerialPort serial(port, kBaudRate);
        serial.open();

        hiwonder::HiwonderBus bus(serial);

        std::cout << "Scanning Hiwonder servo bus\n";
        std::cout << "Port: " << port << '\n';
        std::cout << "Baud rate: " << kBaudRate << "\n\n";

        std::size_t servo_count = 0;

        for (
            uint16_t id = kMinServoId;
            id <= kMaxServoId;
            ++id)
        {
            if (!bus.ping(static_cast<uint8_t>(id))) {
                continue;
            }

            printServoInfo(
                bus,
                static_cast<uint8_t>(id));

            std::cout << '\n';

            ++servo_count;
        }

        std::cout
            << "Found "
            << servo_count
            << " servo(s).\n";
    } catch (const std::exception& exception) {
        std::cerr
            << "Error: "
            << exception.what()
            << '\n';

        return 1;
    }

    return 0;
}