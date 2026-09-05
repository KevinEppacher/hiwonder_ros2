#include "hiwonder_servo_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace hiwonder
{

namespace
{

speed_t toTermiosBaudRate(uint32_t baud_rate)
{
    switch (baud_rate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 1000000:
            return B1000000;
        default:
            throw std::invalid_argument(
                "Unsupported baud rate: " + std::to_string(baud_rate));
    }
}

}

SerialPort::SerialPort(std::string device, uint32_t baud_rate)
    : device_(std::move(device)),
      baud_rate_(baud_rate)
{
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::open()
{
    if (isOpen()) {
        return;
    }

    fd_ = ::open(
        device_.c_str(),
        O_RDWR | O_NOCTTY | O_CLOEXEC);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open " + device_ + ": " + std::strerror(errno));
    }

    termios tty{};

    if (tcgetattr(fd_, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close();
        throw std::runtime_error("tcgetattr failed: " + error);
    }

    cfmakeraw(&tty);

    const speed_t speed = toTermiosBaudRate(baud_rate_);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close();
        throw std::runtime_error("tcsetattr failed: " + error);
    }

    tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close()
{
    if (!isOpen()) {
        return;
    }

    ::close(fd_);
    fd_ = -1;
}

bool SerialPort::isOpen() const noexcept
{
    return fd_ >= 0;
}

void SerialPort::flushInput()
{
    if (!isOpen()) {
        throw std::runtime_error("Serial port is not open");
    }

    if (tcflush(fd_, TCIFLUSH) != 0) {
        throw std::runtime_error(
            "Failed to flush serial input: " +
            std::string(std::strerror(errno)));
    }
}

void SerialPort::write(const uint8_t* data, std::size_t size)
{
    if (!isOpen()) {
        throw std::runtime_error("Serial port is not open");
    }

    std::size_t written = 0;

    while (written < size) {
        const ssize_t result = ::write(
            fd_,
            data + written,
            size - written);

        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }

            throw std::runtime_error(
                "Serial write failed: " +
                std::string(std::strerror(errno)));
        }

        written += static_cast<std::size_t>(result);
    }

    if (tcdrain(fd_) != 0) {
        throw std::runtime_error(
            "tcdrain failed: " +
            std::string(std::strerror(errno)));
    }
}

std::size_t SerialPort::read(
    uint8_t* data,
    std::size_t size,
    uint32_t timeout_ms)
{
    if (!isOpen()) {
        throw std::runtime_error("Serial port is not open");
    }

    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLIN;

    const int result = ::poll(
        &descriptor,
        1,
        static_cast<int>(timeout_ms));

    if (result < 0) {
        if (errno == EINTR) {
            return 0;
        }

        throw std::runtime_error(
            "Serial poll failed: " +
            std::string(std::strerror(errno)));
    }

    if (result == 0) {
        return 0;
    }

    const ssize_t bytes_read = ::read(fd_, data, size);

    if (bytes_read < 0) {
        if (errno == EINTR || errno == EAGAIN) {
            return 0;
        }

        throw std::runtime_error(
            "Serial read failed: " +
            std::string(std::strerror(errno)));
    }

    return static_cast<std::size_t>(bytes_read);
}

}