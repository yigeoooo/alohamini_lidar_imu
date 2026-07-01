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

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "alohamini_base_control/feetech_protocol.hpp"

using namespace alohamini_base_control;

// --- deg/s <-> raw, mirroring lekiwi.py _degps_to_raw / _raw_to_degps. ---

TEST(FeetechProtocol, DegpsToRawZero)
{
  EXPECT_EQ(degpsToRaw(0.0), 0);
}

TEST(FeetechProtocol, DegpsToRawRoundTrip)
{
  // 90 deg/s * (4096/360) = 1024 ticks.
  EXPECT_EQ(degpsToRaw(90.0), 1024);
  EXPECT_NEAR(rawToDegps(1024), 90.0, 1e-9);
  // Negative direction preserved.
  EXPECT_EQ(degpsToRaw(-90.0), -1024);
  EXPECT_NEAR(rawToDegps(-1024), -90.0, 1e-9);
}

TEST(FeetechProtocol, DegpsToRawClampsInt16)
{
  EXPECT_EQ(degpsToRaw(1e9), 0x7FFF);
  EXPECT_EQ(degpsToRaw(-1e9), -0x8000);
}

// --- sign-magnitude encoding, mirroring encoding_utils.py. ---

TEST(FeetechProtocol, SignMagnitudeRoundTrip)
{
  EXPECT_EQ(encodeSignMagnitude(0, 15), 0u);
  EXPECT_EQ(encodeSignMagnitude(1000, 15), 1000u);
  // Negative sets bit 15.
  EXPECT_EQ(encodeSignMagnitude(-1000, 15), static_cast<std::uint16_t>((1u << 15) | 1000u));

  EXPECT_EQ(decodeSignMagnitude(1000, 15), 1000);
  EXPECT_EQ(decodeSignMagnitude((1u << 15) | 1000u, 15), -1000);
}

TEST(FeetechProtocol, SignMagnitudeEncodeDecodeIdentity)
{
  for (int v : {0, 1, -1, 255, -255, 16383, -16383}) {
    const std::uint16_t enc = encodeSignMagnitude(v, 15);
    EXPECT_EQ(decodeSignMagnitude(enc, 15), v) << "value=" << v;
  }
}

// --- Checksum: ~(sum of bytes) & 0xFF (Feetech protocol 0). ---

TEST(FeetechProtocol, Checksum)
{
  // Example: ID=1, LEN=2, INST=1 (PING) -> ~(1+2+1) & 0xFF = ~4 & 0xFF = 0xFB.
  std::vector<std::uint8_t> body{0x01, 0x02, 0x01};
  EXPECT_EQ(checksum(body), 0xFB);
}

// --- WRITE packet structure (FF FF ID LEN INST ADDR DATA... CHK). ---

TEST(FeetechProtocol, WritePacketSingleByte)
{
  // Write Operating_Mode (addr 33) = 1 on motor 8.
  auto pkt = buildWritePacket(8, reg::kOperatingMode, {0x01});
  // FF FF 08 04 03 21 01 CHK
  ASSERT_EQ(pkt.size(), 8u);
  EXPECT_EQ(pkt[0], 0xFF);
  EXPECT_EQ(pkt[1], 0xFF);
  EXPECT_EQ(pkt[2], 8);          // ID
  EXPECT_EQ(pkt[3], 4);          // LEN = data(1) + addr(1) + inst(1) + chk(1)... = 3 + 1
  EXPECT_EQ(pkt[4], 0x03);       // INST_WRITE
  EXPECT_EQ(pkt[5], reg::kOperatingMode);
  EXPECT_EQ(pkt[6], 0x01);
  std::vector<std::uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
  EXPECT_EQ(pkt.back(), checksum(body));
}

// --- SYNC WRITE Goal_Velocity packet for 3 motors. ---

TEST(FeetechProtocol, SyncWriteGoalVelocityPacket)
{
  // 3 motors, 2-byte data each.
  std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> id_data = {
    {8, {0x00, 0x04}},   // 1024
    {10, {0x00, 0x00}},  // 0
    {9, {0x00, 0x84}},   // 1024 with sign bit (negative)
  };
  auto pkt = buildSyncWritePacket(reg::kGoalVelocity, 2, id_data);
  // Header(2) + broadcastID(1) + LEN(1) + INST(1) + ADDR(1) + DATALEN(1)
  //   + N*(id + 2 data)(3*3) + CHK(1) = 17
  ASSERT_EQ(pkt.size(), 17u);
  EXPECT_EQ(pkt[0], 0xFF);
  EXPECT_EQ(pkt[1], 0xFF);
  EXPECT_EQ(pkt[2], 0xFE);   // broadcast
  EXPECT_EQ(pkt[3], (2 + 1) * 3 + 4);  // LEN = 13
  EXPECT_EQ(pkt[4], 0x83);   // INST_SYNC_WRITE
  EXPECT_EQ(pkt[5], reg::kGoalVelocity);
  EXPECT_EQ(pkt[6], 2);      // data length
  EXPECT_EQ(pkt[7], 8);      // first motor id
  std::vector<std::uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
  EXPECT_EQ(pkt.back(), checksum(body));
}

// --- SYNC READ Present_Velocity request packet. ---

TEST(FeetechProtocol, SyncReadPacket)
{
  auto pkt = buildSyncReadPacket(reg::kPresentVelocity, 2, {8, 9, 10});
  // Header(2) + 0xFE + LEN + INST + ADDR + DATALEN + 3 ids + CHK
  ASSERT_EQ(pkt.size(), 11u);
  EXPECT_EQ(pkt[2], 0xFE);
  EXPECT_EQ(pkt[3], 3 + 4);  // LEN
  EXPECT_EQ(pkt[4], 0x82);   // INST_SYNC_READ
  EXPECT_EQ(pkt[5], reg::kPresentVelocity);
  EXPECT_EQ(pkt[6], 2);
  EXPECT_EQ(pkt[7], 8);
  EXPECT_EQ(pkt[8], 9);
  EXPECT_EQ(pkt[9], 10);
  std::vector<std::uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
  EXPECT_EQ(pkt.back(), checksum(body));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
