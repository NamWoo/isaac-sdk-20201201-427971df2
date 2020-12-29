/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <queue>

#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/math/types.hpp"
#include "packages/coms/gems/serial.hpp"
#include "gtest/gtest.h"

namespace {
constexpr int kPacketSize = 11;
}  // namespace

TEST(HelloTest, GetGreet) {

  isaac::Serial s = isaac::Serial("/dev/ttyUSB1", 9600);
  unsigned char command[5] = {0xFF, 0xAA, 0x63, 0x5c, 0x30};

  s.writeChars(command, 5);
  s = isaac::Serial("/dev/ttyUSB1", 115200);
  unsigned char buffer[1024] = {0};
  int size = 0;

  isaac::Vector3d acceleration, velocity, angle;

  while (true) {
    usleep(10);

    size+= s.readChars(&buffer[size], sizeof(buffer) - size);
    if (size < kPacketSize) {
      continue;
    }
    int start = 0;
    for (; start<1024; start++ ) {
      if (buffer[start] == 0x55) {
        break;
      }
    }
    if (buffer[start] != 0x55) {
      continue;
    }

    ASSERT(start+12 < 1024, "Buffer overflow");

    // Verify checksum
    unsigned char sum = 0;
    for (int i = start; i<kPacketSize-1; i++) {
      sum+= buffer[i];
    }
    ASSERT(sum == buffer[start+kPacketSize-1], "Invalid checksum %02x %02x", sum, buffer[start+kPacketSize-1]);

    switch (buffer[1]) {
      case 0x51:
        // Acceleration in x,y,z
        acceleration[0] = ((short)(buffer[start+3]<<8)|buffer[start+2])/32768.0*16.0;
        acceleration[1] = ((short)(buffer[start+5]<<8)|buffer[start+4])/32768.0*16.0;
        acceleration[2] = ((short)(buffer[start+6]<<8)|buffer[start+7])/32768.0*16.0;
        LOG_INFO("Acceleration %f %f %f", acceleration[0], acceleration[1], acceleration[2]);
        break;
      case 0x52:
        // Angular velocity
        velocity[0] = isaac::DegToRad(((short)(buffer[start+3]<<8)|buffer[start+2])/32768.0*2000.0);
        velocity[1] = isaac::DegToRad(((short)(buffer[start+5]<<8)|buffer[start+4])/32768.0*2000.0);
        velocity[2] = isaac::DegToRad(((short)(buffer[start+7]<<8)|buffer[start+6])/32768.0*2000.0);
        LOG_INFO("velocity %f %f %f", velocity[0], velocity[1], velocity[2]);
        break;
      case 0x53:
        // Measured angles
        angle[0] = isaac::DegToRad(((short)(buffer[start+3]<<8)|buffer[start+2])/32768.0*180.0);
        angle[1] = isaac::DegToRad(((short)(buffer[start+5]<<8)|buffer[start+4])/32768.0*180.0);
        angle[2] = isaac::DegToRad(((short)(buffer[start+7]<<8)|buffer[start+6])/32768.0*180.0);
        LOG_INFO("angle %f %f %f", angle[0], angle[1], angle[2]);
        break;
    }
    memmove(buffer, buffer+start+kPacketSize, size-start);
    size-= start+kPacketSize;
  }
}