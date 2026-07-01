// Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

#ifndef ALOHAMINI_BASE_CONTROL__FEETECH_PROTOCOL_HPP_
#define ALOHAMINI_BASE_CONTROL__FEETECH_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace alohamini_base_control
{

// Feetech STS/SMS control-table addresses (protocol 0).
// Mirrors lerobot's src/lerobot/motors/feetech/tables.py.
namespace reg
{
constexpr std::uint8_t kReturnDelayTime = 7;
constexpr std::uint8_t kOperatingMode = 33;
constexpr std::uint8_t kMaximumAcceleration = 85;
constexpr std::uint8_t kAcceleration = 41;
constexpr std::uint8_t kTorqueEnable = 40;
constexpr std::uint8_t kLock = 55;
constexpr std::uint8_t kGoalVelocity = 46;     // 2 bytes, sign-magnitude (bit 15)
constexpr std::uint8_t kPresentVelocity = 58;  // 2 bytes, sign-magnitude (bit 15)
}  // namespace reg

// Operating modes (Operating_Mode register).
namespace operating_mode
{
constexpr std::uint8_t kPosition = 0;
constexpr std::uint8_t kVelocity = 1;
}  // namespace operating_mode

// Steps-per-degree for a 4096-count encoder, used by deg/s <-> raw conversions.
// Mirrors lekiwi.py `_degps_to_raw` / `_raw_to_degps`.
constexpr double kStepsPerDeg = 4096.0 / 360.0;

// Convert wheel angular speed (deg/s) to a signed raw velocity tick, clamped to int16.
std::int16_t degpsToRaw(double degps);

// Convert a signed raw velocity tick back into deg/s.
double rawToDegps(std::int16_t raw);

// Encode a signed value into Feetech sign-magnitude form (sign bit = bit `sign_bit_index`).
std::uint16_t encodeSignMagnitude(std::int32_t value, int sign_bit_index = 15);

// Decode a Feetech sign-magnitude value back into a signed integer.
std::int32_t decodeSignMagnitude(std::uint16_t encoded, int sign_bit_index = 15);

// Feetech protocol-0 checksum: ~(sum of bytes from ID onward) & 0xFF.
std::uint8_t checksum(const std::vector<std::uint8_t> & packet_from_id);

// Build a complete SYNC WRITE (0x83) packet for a 2-byte register across several motors.
// `id_value` pairs are (motor_id, raw_register_value); values are written verbatim
// (caller is responsible for sign-magnitude encoding via encodeSignMagnitude()).
std::vector<std::uint8_t> buildSyncWritePacket(
  std::uint8_t address, std::uint8_t data_len,
  const std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> & id_data);

// Build a WRITE (0x03) packet for a single motor register (1 or 2 bytes).
std::vector<std::uint8_t> buildWritePacket(
  std::uint8_t motor_id, std::uint8_t address, const std::vector<std::uint8_t> & data);

// Build a SYNC READ (0x82) packet requesting `data_len` bytes at `address` from each id.
std::vector<std::uint8_t> buildSyncReadPacket(
  std::uint8_t address, std::uint8_t data_len, const std::vector<std::uint8_t> & ids);

// Thin termios-based serial transport speaking Feetech protocol 0.
// Intentionally free of any ROS dependency so it can be unit-tested standalone.
class FeetechBus
{
public:
  FeetechBus() = default;
  ~FeetechBus();

  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  // Open `port` at `baud` (default 1 Mbps) in raw mode. Returns false on failure.
  bool open(const std::string & port, int baud = 1000000);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  const std::string & lastError() const { return last_error_; }

  // Write a single 1- or 2-byte register on one motor (with status read-back drained).
  bool writeRegister(std::uint8_t motor_id, std::uint8_t address, std::uint32_t value, int data_len);

  // SYNC WRITE Goal_Velocity (addr 46) for the given (id, raw deg/s tick) pairs.
  // Raw ticks are sign-magnitude encoded internally.
  bool syncWriteGoalVelocity(const std::vector<std::pair<std::uint8_t, std::int16_t>> & id_raw);

  // SYNC READ Present_Velocity (addr 58). On success `out` is filled in `ids` order
  // with sign-magnitude decoded ticks. Missing replies leave the previous value and
  // mark the call as failed.
  bool syncReadPresentVelocity(
    const std::vector<std::uint8_t> & ids, std::vector<std::int16_t> & out);

private:
  // Flush, then send raw bytes.
  bool writeAll(const std::vector<std::uint8_t> & data);
  // Read exactly `n` bytes or time out. Appends to `out`.
  bool readExact(std::size_t n, std::vector<std::uint8_t> & out, int timeout_ms);
  // Drain whatever status bytes the bus echoes back (best-effort, ignores content).
  void drainInput(int timeout_ms);

  int fd_ = -1;
  std::string last_error_;
  int timeout_ms_ = 50;
};

}  // namespace alohamini_base_control

#endif  // ALOHAMINI_BASE_CONTROL__FEETECH_PROTOCOL_HPP_
