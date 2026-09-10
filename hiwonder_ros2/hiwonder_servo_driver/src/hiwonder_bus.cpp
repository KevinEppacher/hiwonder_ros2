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

#include "hiwonder_servo_driver/hiwonder_bus.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>

namespace hiwonder
{

HiwonderBus::HiwonderBus(SerialPort & serial)
: serial_(serial)
{
}

uint8_t HiwonderBus::checksum(
  std::span<const uint8_t> data)
{
  uint16_t sum = 0;

  for (const uint8_t byte : data) {
    sum += byte;
  }

  return static_cast<uint8_t>(~sum & 0xFF);
}

std::size_t HiwonderBus::buildPacket(
  std::span<uint8_t> output,
  uint8_t id,
  uint8_t instruction,
  std::span<const uint8_t> parameters)
{
  if (parameters.size() > kMaxParameterCount) {
    return 0;
  }

  const std::size_t packet_size =
    kPacketOverhead + parameters.size();

  if (output.size() < packet_size) {
    return 0;
  }

  const std::size_t protocol_length = parameters.size() + 2;

  if (protocol_length > std::numeric_limits<uint8_t>::max()) {
    return 0;
  }

  const uint8_t length =
    static_cast<uint8_t>(protocol_length);

  output[0] = kHeader;
  output[1] = kHeader;
  output[2] = id;
  output[3] = length;
  output[4] = instruction;

  std::copy(
        parameters.begin(),
        parameters.end(),
        output.begin() + 5);

  output[5 + parameters.size()] = checksum(
        output.subspan(
            2,
            3 + parameters.size()));

  return packet_size;
}

bool HiwonderBus::receiveStatusPacket(
  uint8_t expected_id,
  std::span<uint8_t> parameters,
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

  const std::size_t parameter_count =
    static_cast<std::size_t>(length) - 2;

  if (parameter_count != parameters.size()) {
    return false;
  }

    // Read ERROR + PARAMETERS + CHECKSUM.

  std::array<uint8_t, kPacketBufferSize> payload{};

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

  std::array<uint8_t, kPacketBufferSize> checksum_data{};

  checksum_data[0] = id;
  checksum_data[1] = length;
  checksum_data[2] = error;

  std::copy(
        payload.begin() + 1,
        payload.begin() + 1 + parameter_count,
        checksum_data.begin() + 3);

  const uint8_t expected_checksum = checksum(
        std::span<const uint8_t>(
            checksum_data.data(),
            3 + parameter_count));

  if (received_checksum != expected_checksum) {
    return false;
  }

  if (error != 0) {
    return false;
  }

  std::copy(
        payload.begin() + 1,
        payload.begin() + 1 + parameter_count,
        parameters.begin());

  return true;
}

bool HiwonderBus::ping(uint8_t id)
{
  std::array<uint8_t, 8> packet{};

  const std::size_t packet_size = buildPacket(
        packet,
        id,
        kPingInstruction,
    {});

  if (packet_size == 0) {
    return false;
  }

  serial_.flushInput();
  serial_.write(packet.data(), packet_size);

  return receiveStatusPacket(
        id,
    {},
        50);
}

bool HiwonderBus::read(
  uint8_t id,
  uint8_t address,
  std::span<uint8_t> data)
{
  if (
    data.empty() ||
    data.size() > std::numeric_limits<uint8_t>::max())
  {
    return false;
  }

  const std::array<uint8_t, 2> parameters{
    address,
    static_cast<uint8_t>(data.size())
  };

  std::array<uint8_t, 16> packet{};

  const std::size_t packet_size = buildPacket(
        packet,
        id,
        kReadInstruction,
        parameters);

  if (packet_size == 0) {
    return false;
  }

  serial_.flushInput();
  serial_.write(packet.data(), packet_size);

  return receiveStatusPacket(
        id,
        data,
        50);
}

bool HiwonderBus::readWord(
  uint8_t id,
  uint8_t address,
  uint16_t & value)
{
  std::array<uint8_t, 2> data{};

  if (!read(
            id,
            address,
            data))
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
  std::span<const uint8_t> data)
{
  if (data.empty()) {
    return false;
  }

  if (data.size() + 1 > kMaxParameterCount) {
    return false;
  }

  std::array<uint8_t, kMaxParameterCount> parameters{};

  parameters[0] = address;

  std::copy(
        data.begin(),
        data.end(),
        parameters.begin() + 1);

  const std::size_t parameter_count = data.size() + 1;

  std::array<uint8_t, kPacketBufferSize> packet{};

  const std::size_t packet_size = buildPacket(
        packet,
        id,
        kWriteInstruction,
        std::span<const uint8_t>(
            parameters.data(),
            parameter_count));

  if (packet_size == 0) {
    return false;
  }

  serial_.flushInput();
  serial_.write(packet.data(), packet_size);

  return receiveStatusPacket(
        id,
    {},
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
        data);
}

bool HiwonderBus::syncWrite(
  uint8_t address,
  uint8_t data_length,
  std::span<const uint8_t> ids,
  std::span<const uint8_t> data)
{
  if (
    data_length == 0 ||
    ids.empty() ||
    data.empty())
  {
    return false;
  }

    // Each servo must have exactly data_length bytes of data.
    //
    // Example:
    //   6 servos * 6 bytes per servo = 36 data bytes.
  const std::size_t expected_data_size =
    ids.size() * static_cast<std::size_t>(data_length);

  if (data.size() != expected_data_size) {
    return false;
  }

    // SYNC_WRITE parameters are encoded as:
    //
    //   [address] [data_length]
    //   [id_1] [data_1 ...]
    //   [id_2] [data_2 ...]
    //   ...
    //
    // The first 2 bytes contain address and data_length.
    // Each servo then requires 1 byte for its ID plus
    // data_length bytes of payload.
    //
    // Therefore:
    //   parameter_count = 2 + servo_count * (1 + data_length)
  const std::size_t parameter_count =
    2 +
    ids.size() *
    (1 + static_cast<std::size_t>(data_length));

    // The complete packet must fit into kPacketBufferSize.
    // kMaxParameterCount accounts for the packet overhead:
    //
    //   FF FF ID LENGTH INSTRUCTION PARAMETERS... CHECKSUM
    //
    // For a 256-byte packet buffer, 6 bytes are reserved for
    // the header, ID, length, instruction, and checksum, leaving
    // at most 250 bytes for parameters.
  if (parameter_count > kMaxParameterCount) {
    return false;
  }

  std::array<uint8_t, kMaxParameterCount> parameters{};

  std::size_t index = 0;

    // SYNC_WRITE parameter header.
  parameters[index++] = address;
  parameters[index++] = data_length;

    // Append one ID followed by data_length bytes for each servo.
  for (std::size_t i = 0; i < ids.size(); ++i) {
    parameters[index++] = ids[i];

    for (std::size_t j = 0; j < data_length; ++j) {
      parameters[index++] =
        data[i * data_length + j];
    }
  }

  std::array<uint8_t, kPacketBufferSize> packet{};

  const std::size_t packet_size = buildPacket(
        packet,
        kBroadcastId,
        kSyncWriteInstruction,
        std::span<const uint8_t>(
            parameters.data(),
            index));

  if (packet_size == 0) {
    return false;
  }

  serial_.flushInput();
  serial_.write(packet.data(), packet_size);

    // SYNC_WRITE uses the broadcast ID, so no status packet is returned.
  return true;
}

}  // namespace hiwonder
