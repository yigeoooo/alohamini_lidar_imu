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

#include "alohamini_base_control/feetech_protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace alohamini_base_control
{

namespace
{
constexpr std::uint8_t kHeader = 0xFF;
constexpr std::uint8_t kInstWrite = 0x03;
constexpr std::uint8_t kInstSyncRead = 0x82;
constexpr std::uint8_t kInstSyncWrite = 0x83;

std::uint8_t loByte(std::uint16_t v) { return static_cast<std::uint8_t>(v & 0xFF); }
std::uint8_t hiByte(std::uint16_t v) { return static_cast<std::uint8_t>((v >> 8) & 0xFF); }
}  // namespace

std::int16_t degpsToRaw(double degps)
{
  double speed_in_steps = degps * kStepsPerDeg;
  long speed_int = std::lround(speed_in_steps);
  if (speed_int > 0x7FFF) {
    speed_int = 0x7FFF;
  } else if (speed_int < -0x8000) {
    speed_int = -0x8000;
  }
  return static_cast<std::int16_t>(speed_int);
}

double rawToDegps(std::int16_t raw)
{
  return static_cast<double>(raw) / kStepsPerDeg;
}

std::uint16_t encodeSignMagnitude(std::int32_t value, int sign_bit_index)
{
  const std::int32_t max_magnitude = (1 << sign_bit_index) - 1;
  std::int32_t magnitude = std::abs(value);
  if (magnitude > max_magnitude) {
    magnitude = max_magnitude;
  }
  const std::uint16_t direction_bit = (value < 0) ? 1u : 0u;
  return static_cast<std::uint16_t>((direction_bit << sign_bit_index) | magnitude);
}

std::int32_t decodeSignMagnitude(std::uint16_t encoded, int sign_bit_index)
{
  const std::uint16_t direction_bit = (encoded >> sign_bit_index) & 1u;
  const std::uint16_t magnitude_mask = static_cast<std::uint16_t>((1 << sign_bit_index) - 1);
  const std::int32_t magnitude = encoded & magnitude_mask;
  return direction_bit ? -magnitude : magnitude;
}

std::uint8_t checksum(const std::vector<std::uint8_t> & packet_from_id)
{
  std::uint32_t sum = 0;
  for (std::uint8_t b : packet_from_id) {
    sum += b;
  }
  return static_cast<std::uint8_t>(~sum & 0xFF);
}

std::vector<std::uint8_t> buildWritePacket(
  std::uint8_t motor_id, std::uint8_t address, const std::vector<std::uint8_t> & data)
{
  // Length = #params + 2 (instruction + checksum); params = address + data.
  const std::uint8_t length = static_cast<std::uint8_t>(data.size() + 3);
  std::vector<std::uint8_t> body;  // from ID onward, used for checksum
  body.push_back(motor_id);
  body.push_back(length);
  body.push_back(kInstWrite);
  body.push_back(address);
  body.insert(body.end(), data.begin(), data.end());

  std::vector<std::uint8_t> packet{kHeader, kHeader};
  packet.insert(packet.end(), body.begin(), body.end());
  packet.push_back(checksum(body));
  return packet;
}

std::vector<std::uint8_t> buildSyncWritePacket(
  std::uint8_t address, std::uint8_t data_len,
  const std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> & id_data)
{
  // Length = (L+1) * N + 4, where L = data_len, N = #motors.
  const std::size_t n = id_data.size();
  const std::uint8_t length = static_cast<std::uint8_t>((data_len + 1) * n + 4);

  std::vector<std::uint8_t> body;  // from ID (=0xFE broadcast) onward
  body.push_back(0xFE);            // broadcast ID for sync write
  body.push_back(length);
  body.push_back(kInstSyncWrite);
  body.push_back(address);
  body.push_back(data_len);
  for (const auto & [motor_id, data] : id_data) {
    body.push_back(motor_id);
    body.insert(body.end(), data.begin(), data.end());
  }

  std::vector<std::uint8_t> packet{kHeader, kHeader};
  packet.insert(packet.end(), body.begin(), body.end());
  packet.push_back(checksum(body));
  return packet;
}

std::vector<std::uint8_t> buildSyncReadPacket(
  std::uint8_t address, std::uint8_t data_len, const std::vector<std::uint8_t> & ids)
{
  // Length = #ids + 4 (instruction + address + data_len + checksum).
  const std::uint8_t length = static_cast<std::uint8_t>(ids.size() + 4);
  std::vector<std::uint8_t> body;
  body.push_back(0xFE);  // broadcast ID for sync read
  body.push_back(length);
  body.push_back(kInstSyncRead);
  body.push_back(address);
  body.push_back(data_len);
  body.insert(body.end(), ids.begin(), ids.end());

  std::vector<std::uint8_t> packet{kHeader, kHeader};
  packet.insert(packet.end(), body.begin(), body.end());
  packet.push_back(checksum(body));
  return packet;
}

// ---------------------------------------------------------------------------
// FeetechBus
// ---------------------------------------------------------------------------

FeetechBus::~FeetechBus() { close(); }

bool FeetechBus::open(const std::string & port, int baud)
{
  close();
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = "open(" + port + ") failed: " + std::strerror(errno);
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::string("tcgetattr failed: ") + std::strerror(errno);
    close();
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  speed_t speed;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 500000: speed = B500000; break;
    case 1000000: speed = B1000000; break;
    default: speed = B1000000; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::string("tcsetattr failed: ") + std::strerror(errno);
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  last_error_.clear();
  return true;
}

void FeetechBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool FeetechBus::writeAll(const std::vector<std::uint8_t> & data)
{
  if (fd_ < 0) {
    last_error_ = "port not open";
    return false;
  }
  std::size_t written = 0;
  while (written < data.size()) {
    ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }
      last_error_ = std::string("write failed: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

bool FeetechBus::readExact(std::size_t n, std::vector<std::uint8_t> & out, int timeout_ms)
{
  if (fd_ < 0) {
    return false;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t got = 0;
  std::uint8_t buf[64];
  while (got < n) {
    ssize_t r = ::read(fd_, buf, std::min(sizeof(buf), n - got));
    if (r > 0) {
      out.insert(out.end(), buf, buf + r);
      got += static_cast<std::size_t>(r);
      continue;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return true;
}

void FeetechBus::drainInput(int timeout_ms)
{
  if (fd_ < 0) {
    return;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::uint8_t buf[64];
  while (std::chrono::steady_clock::now() < deadline) {
    ssize_t r = ::read(fd_, buf, sizeof(buf));
    if (r <= 0) {
      break;
    }
  }
}

bool FeetechBus::writeRegister(
  std::uint8_t motor_id, std::uint8_t address, std::uint32_t value, int data_len)
{
  std::vector<std::uint8_t> data;
  if (data_len == 1) {
    data.push_back(static_cast<std::uint8_t>(value & 0xFF));
  } else {
    data.push_back(loByte(static_cast<std::uint16_t>(value)));
    data.push_back(hiByte(static_cast<std::uint16_t>(value)));
  }
  tcflush(fd_, TCIFLUSH);
  if (!writeAll(buildWritePacket(motor_id, address, data))) {
    return false;
  }
  // A status packet (6 bytes) is echoed back; drain it so it does not corrupt
  // the next sync-read. Best-effort only.
  drainInput(timeout_ms_);
  return true;
}

bool FeetechBus::syncWriteGoalVelocity(
  const std::vector<std::pair<std::uint8_t, std::int16_t>> & id_raw)
{
  std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> id_data;
  id_data.reserve(id_raw.size());
  for (const auto & [motor_id, raw] : id_raw) {
    const std::uint16_t enc = encodeSignMagnitude(raw, 15);
    id_data.emplace_back(motor_id, std::vector<std::uint8_t>{loByte(enc), hiByte(enc)});
  }
  tcflush(fd_, TCIFLUSH);
  // Sync write is a broadcast: no status packet is returned.
  return writeAll(buildSyncWritePacket(reg::kGoalVelocity, 2, id_data));
}

bool FeetechBus::syncReadPresentVelocity(
  const std::vector<std::uint8_t> & ids, std::vector<std::int16_t> & out)
{
  out.assign(ids.size(), 0);
  if (fd_ < 0) {
    last_error_ = "port not open";
    return false;
  }

  tcflush(fd_, TCIFLUSH);
  if (!writeAll(buildSyncReadPacket(reg::kPresentVelocity, 2, ids))) {
    return false;
  }

  // Each motor replies with a status packet:
  //   FF FF ID LEN ERR D0 D1 CHK   (LEN = data_len + 2 = 4)
  // Replies arrive in the request order, but we match by ID to be robust.
  std::vector<std::uint8_t> buf;
  const std::size_t expected = ids.size() * 8;
  readExact(expected, buf, timeout_ms_);  // best-effort; parse whatever arrived

  bool all_ok = true;
  std::size_t i = 0;
  std::vector<bool> filled(ids.size(), false);
  while (i + 7 < buf.size() + 1 && i + 8 <= buf.size()) {
    if (buf[i] != kHeader || buf[i + 1] != kHeader) {
      ++i;
      continue;
    }
    const std::uint8_t id = buf[i + 2];
    const std::uint8_t len = buf[i + 3];
    if (len != 4) {
      ++i;
      continue;
    }
    // Verify checksum over [id .. last data byte].
    std::vector<std::uint8_t> body(buf.begin() + i + 2, buf.begin() + i + 7);
    const std::uint8_t expected_chk = checksum(body);
    if (buf[i + 7] != expected_chk) {
      ++i;
      continue;
    }
    const std::uint16_t raw_enc =
      static_cast<std::uint16_t>(buf[i + 5] | (buf[i + 6] << 8));
    const std::int16_t value =
      static_cast<std::int16_t>(decodeSignMagnitude(raw_enc, 15));
    for (std::size_t k = 0; k < ids.size(); ++k) {
      if (ids[k] == id) {
        out[k] = value;
        filled[k] = true;
      }
    }
    i += 8;
  }

  for (std::size_t k = 0; k < ids.size(); ++k) {
    if (!filled[k]) {
      all_ok = false;
    }
  }
  if (!all_ok) {
    last_error_ = "sync read: missing or corrupt replies";
  }
  return all_ok;
}

}  // namespace alohamini_base_control
