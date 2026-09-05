#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace hiwonder
{

class SerialPort
{
public:
    explicit SerialPort(
        std::string device,
        uint32_t baud_rate = 1000000);

    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void open();
    void close();

    [[nodiscard]] bool isOpen() const noexcept;

    void flushInput();

    void write(const uint8_t* data, std::size_t size);

    std::size_t read(
        uint8_t* data,
        std::size_t size,
        uint32_t timeout_ms);

private:
    std::string device_;
    uint32_t baud_rate_;
    int fd_{-1};
};

}