/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "i2c_stub.hpp"

#include <algorithm>
#include <cstring>

#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"

namespace {
const size_t MAX_I2C_TRANSFER_SIZE = 32;  // max size of an I2C transfer, in bytes
}  // namespace

namespace isaac {

I2cStub::I2cStub(device_memory_map& memory_map): memory_map_(memory_map) {
  // no action needed
}

bool I2cStub::open(int device_id, int i2c_address) {
  // we're just a stub, return true
  return true;
}

void I2cStub::close() {
  // we're just a stub, do nothing
}

int I2cStub::read(int i2c_register, size_t num_of_bytes, byte* buffer) {
  // copy the bytes from the memory map to the buffer

  ASSERT(i2c_register >= 0 && static_cast<size_t>(i2c_register) < memory_map_.size(),
      "i2c_register is out of range");

  ASSERT(static_cast<size_t>(i2c_register + num_of_bytes) <= memory_map_.size(),
      "requested read contains buffer overflow");

  // cap the transfer size to MAX_I2C_TRANSFER_SIZE
  num_of_bytes = std::min(num_of_bytes, MAX_I2C_TRANSFER_SIZE);

  std::memcpy(buffer, memory_map_.data() + i2c_register, num_of_bytes);
  return num_of_bytes;
}

int I2cStub::write(int i2c_register, size_t num_of_bytes, const byte* buffer) {
  ASSERT(i2c_register >= 0 && static_cast<size_t>(i2c_register) < memory_map_.size(),
      "i2c_register is out of range");

  ASSERT(static_cast<size_t>(i2c_register + num_of_bytes) <= memory_map_.size(),
      "requested write contains buffer overflow");

  // cap the transfer size to MAX_I2C_TRANSFER_SIZE
  num_of_bytes = std::min(num_of_bytes, MAX_I2C_TRANSFER_SIZE);

  std::memcpy(memory_map_.data() + i2c_register, buffer, num_of_bytes);

  return num_of_bytes;
}

}  // namespace isaac
