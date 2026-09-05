#include "hiwonder_servo_driver/servo.hpp"

#include "hiwonder_servo_driver/registers.hpp"

namespace hiwonder
{

Servo::Servo(HiwonderBus& bus, uint8_t id)
    : bus_(bus),
      id_(id)
{
}

uint8_t Servo::id() const noexcept
{
    return id_;
}

bool Servo::ping()
{
    return bus_.ping(id_);
}

bool Servo::writePersistent(
    uint8_t address,
    const uint8_t* data,
    uint8_t length)
{
    constexpr uint8_t unlock = 0;
    constexpr uint8_t lock = 1;

    if (!bus_.write(
            id_,
            reg::kNvsLock,
            &unlock,
            1))
    {
        return false;
    }

    if (!bus_.write(
            id_,
            address,
            data,
            length))
    {
        return false;
    }

    if (!bus_.write(
            id_,
            reg::kNvsLock,
            &lock,
            1))
    {
        return false;
    }

    return true;
}

bool Servo::setId(uint8_t new_id)
{
    if (new_id == id_) {
        return true;
    }

    constexpr uint8_t unlock = 0;
    constexpr uint8_t lock = 1;

    if (!bus_.write(
            id_,
            reg::kNvsLock,
            &unlock,
            1))
    {
        return false;
    }

    /*
     * The servo starts using the new ID after this write.
     * Subsequent packets must therefore address new_id.
     */
    if (!bus_.write(
            id_,
            reg::kId,
            &new_id,
            1))
    {
        return false;
    }

    id_ = new_id;

    if (!bus_.write(
            id_,
            reg::kNvsLock,
            &lock,
            1))
    {
        return false;
    }

    return bus_.ping(id_);
}

}