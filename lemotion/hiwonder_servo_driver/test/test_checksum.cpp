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

#include <array>
#include <cstdint>

#include "hiwonder_servo_driver/hiwonder_bus.hpp"

TEST(ChecksumTest, EmptyData)
{
    const std::array<uint8_t, 0> data{};

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0xFF);
}

TEST(ChecksumTest, SingleByte)
{
    const std::array<uint8_t, 1> data{
    0x01
    };

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0xFE);
}

TEST(ChecksumTest, PingPacket)
{
    // Checksum input:
    // ID          = 0x01
    // LENGTH      = 0x02
    // INSTRUCTION = 0x01
    //
    // Sum      = 0x04
    // Checksum = ~0x04 & 0xFF = 0xFB
    const std::array<uint8_t, 3> data{
    0x01,
    0x02,
    0x01
    };

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0xFB);
}

TEST(ChecksumTest, ReadPacket)
{
    // Example READ request:
    //
    // ID          = 0x01
    // LENGTH      = 0x04
    // INSTRUCTION = 0x02
    // ADDRESS     = 0x38
    // READ LENGTH = 0x02
    //
    // Sum      = 0x41
    // Checksum = ~0x41 & 0xFF = 0xBE
    const std::array<uint8_t, 5> data{
    0x01,
    0x04,
    0x02,
    0x38,
    0x02
    };

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0xBE);
}

TEST(ChecksumTest, HandlesOverflow)
{
    // The checksum uses only the lower 8 bits of the accumulated sum.
    //
    // 0xFF + 0xFF = 0x01FE
    // ~0x01FE & 0xFF = 0x01
    const std::array<uint8_t, 2> data{
    0xFF,
    0xFF
    };

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0x01);
}

TEST(ChecksumTest, AllZeroBytes)
{
    const std::array<uint8_t, 4> data{
    0x00,
    0x00,
    0x00,
    0x00
    };

    EXPECT_EQ(
        hiwonder::HiwonderBus::checksum(data),
        0xFF);
}
