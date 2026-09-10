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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"
#include "hiwonder_servo_driver/serial_port.hpp"

namespace
{

class FakeSerialPort : public hiwonder::SerialPort
{
public:
  FakeSerialPort()
  : SerialPort("/dev/null")
  {
  }

  void flushInput() override
  {
    ++flush_count_;
  }

  void write(
    const uint8_t * data,
    std::size_t size) override
  {
    written_data_.assign(
            data,
            data + size);
  }

  std::size_t read(
    uint8_t * data,
    std::size_t size,
    uint32_t) override
  {
    const std::size_t remaining =
      read_data_.size() - read_index_;

    const std::size_t bytes_to_read =
      std::min(size, remaining);

    std::copy_n(
            read_data_.begin() +
            static_cast<std::ptrdiff_t>(read_index_),
            bytes_to_read,
            data);

    read_index_ += bytes_to_read;

    return bytes_to_read;
  }

  void setReadData(
    const std::vector<uint8_t> & data)
  {
    read_data_ = data;
    read_index_ = 0;
  }

  [[nodiscard]] const std::vector<uint8_t> & writtenData() const
  {
    return written_data_;
  }

  [[nodiscard]] std::size_t flushCount() const
  {
    return flush_count_;
  }

private:
  std::vector<uint8_t> written_data_;
  std::vector<uint8_t> read_data_;

  std::size_t read_index_{0};
  std::size_t flush_count_{0};
};

TEST(HiwonderBusTest, PingWritesCorrectPacket)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    // Valid status packet for servo ID 1:
    // FF FF ID LENGTH ERROR CHECKSUM
    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x02,
      0x00,
      0xFC
    });

    ASSERT_TRUE(bus.ping(0x01));

    const std::vector<uint8_t> expected{
    0xFF,
    0xFF,
    0x01,
    0x02,
    0x01,
    0xFB
    };

    EXPECT_EQ(
        serial.writtenData(),
        expected);

    EXPECT_EQ(
        serial.flushCount(),
        1U);
}

TEST(HiwonderBusTest, PingFailsForWrongServoId)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    // Valid status packet, but returned by servo ID 2.
    serial.setReadData({
      0xFF,
      0xFF,
      0x02,
      0x02,
      0x00,
      0xFB
    });

    EXPECT_FALSE(bus.ping(0x01));
}

TEST(HiwonderBusTest, PingFailsForInvalidChecksum)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x02,
      0x00,
      0x00
    });

    EXPECT_FALSE(bus.ping(0x01));
}

TEST(HiwonderBusTest, PingFailsForServoError)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    // ID + LENGTH + ERROR = 0x01 + 0x02 + 0x01
    // Checksum = 0xFB
    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x02,
      0x01,
      0xFB
    });

    EXPECT_FALSE(bus.ping(0x01));
}

TEST(HiwonderBusTest, ReadReturnsServoData)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    // Status packet:
    // ID         = 1
    // LENGTH     = 4
    // ERROR      = 0
    // PARAMETERS = 0xE9 0x07
    //
    // 0x07E9 = 2025
    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x04,
      0x00,
      0xE9,
      0x07,
      0x0A
    });

    std::array<uint8_t, 2> data{};

    ASSERT_TRUE(
        bus.read(
            0x01,
            0x38,
            data));

    EXPECT_EQ(data[0], 0xE9);
    EXPECT_EQ(data[1], 0x07);

    const std::vector<uint8_t> expected{
    0xFF,
    0xFF,
    0x01,
    0x04,
    0x02,
    0x38,
    0x02,
    0xBE
    };

    EXPECT_EQ(
        serial.writtenData(),
        expected);
}

TEST(HiwonderBusTest, ReadWordDecodesLittleEndian)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x04,
      0x00,
      0xE9,
      0x07,
      0x0A
    });

    uint16_t value = 0;

    ASSERT_TRUE(
        bus.readWord(
            0x01,
            0x38,
            value));

    EXPECT_EQ(value, 2025);
}

TEST(HiwonderBusTest, ReadRejectsEmptyBuffer)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    std::span<uint8_t> data{};

    EXPECT_FALSE(
        bus.read(
            0x01,
            0x38,
            data));

    EXPECT_TRUE(
        serial.writtenData().empty());
}

TEST(HiwonderBusTest, WriteCreatesCorrectPacket)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    // Empty successful status packet from servo ID 1.
    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x02,
      0x00,
      0xFC
    });

    const std::array<uint8_t, 1> data{
    0x01
    };

    ASSERT_TRUE(
        bus.write(
            0x01,
            0x28,
            data));

    // WRITE:
    // ID          = 0x01
    // LENGTH      = 0x04
    // INSTRUCTION = 0x03
    // ADDRESS     = 0x28
    // DATA        = 0x01
    //
    // Checksum = ~(01 + 04 + 03 + 28 + 01) = 0xCE
    const std::vector<uint8_t> expected{
    0xFF,
    0xFF,
    0x01,
    0x04,
    0x03,
    0x28,
    0x01,
    0xCE
    };

    EXPECT_EQ(
        serial.writtenData(),
        expected);
}

TEST(HiwonderBusTest, WriteWordEncodesLittleEndian)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    serial.setReadData({
      0xFF,
      0xFF,
      0x01,
      0x02,
      0x00,
      0xFC
    });

    ASSERT_TRUE(
        bus.writeWord(
            0x01,
            0x2A,
            0x1234));

    const std::vector<uint8_t> expected{
    0xFF,
    0xFF,
    0x01,
    0x05,
    0x03,
    0x2A,
    0x34,
    0x12,
    0x86
    };

    EXPECT_EQ(
        serial.writtenData(),
        expected);
}

TEST(HiwonderBusTest, SyncWriteCreatesCorrectBroadcastPacket)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    const std::array<uint8_t, 2> ids{
    0x01,
    0x02
    };

    const std::array<uint8_t, 4> data{
    0xE8,
    0x03,
    0xD0,
    0x07
    };

    ASSERT_TRUE(
        bus.syncWrite(
            0x2A,
            2,
            ids,
            data));

    // SYNC_WRITE packet:
    //
    // FF FF FE 0A 83 2A 02 01 E8 03 02 D0 07 83
    //
    // FE       Broadcast ID
    // 0A       Protocol length
    // 83       SYNC_WRITE instruction
    // 2A       Start address
    // 02       Two data bytes per servo
    // 01       First servo ID
    // E8 03    First servo data
    // 02       Second servo ID
    // D0 07    Second servo data
    // 83       Checksum
    const std::vector<uint8_t> expected{
    0xFF,
    0xFF,
    0xFE,
    0x0A,
    0x83,
    0x2A,
    0x02,
    0x01,
    0xE8,
    0x03,
    0x02,
    0xD0,
    0x07,
    0x83
    };

    EXPECT_EQ(
        serial.writtenData(),
        expected);
}

TEST(HiwonderBusTest, SyncWriteRejectsMismatchedDataSize)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    const std::array<uint8_t, 2> ids{
    0x01,
    0x02
    };

    // Two servos with two bytes each would require four bytes.
    const std::array<uint8_t, 3> data{
    0x01,
    0x02,
    0x03
    };

    EXPECT_FALSE(
        bus.syncWrite(
            0x2A,
            2,
            ids,
            data));

    EXPECT_TRUE(
        serial.writtenData().empty());
}

TEST(HiwonderBusTest, SyncWriteRejectsZeroDataLength)
{
    FakeSerialPort serial;
    hiwonder::HiwonderBus bus(serial);

    const std::array<uint8_t, 1> ids{
    0x01
    };

    const std::array<uint8_t, 1> data{
    0x00
    };

    EXPECT_FALSE(
        bus.syncWrite(
            0x2A,
            0,
            ids,
            data));

    EXPECT_TRUE(
        serial.writtenData().empty());
}

}  // namespace
