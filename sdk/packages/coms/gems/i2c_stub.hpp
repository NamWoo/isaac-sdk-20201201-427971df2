/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <cstddef>

#include "engine/core/byte.hpp"
#include "packages/coms/gems/i2c.hpp"

namespace isaac {

/*
This class is used to stub out an I2C device for testing. Instead of opening
an actual I2C device it accesses a user supplied memory map.
*/

// Class for stubbing I2C Devices
class I2cStub : public I2c {
 public:
  // represents the internal memory map of the I2C stub device
  using device_memory_map = std::array<byte, 256>;

  // Construct an I2cStub device
  // memory_map: internal memory map of the I2C stub device
  I2cStub(device_memory_map& memory_map);

  // closes I2C device
  virtual ~I2cStub() = default;

  // "open" a stub I2C device
  // NOTE: no actual device is opened, this method is kept for compatibility
  //       with the I2c class
  // device_id: matches ID of /dev/i2c-X
  // i2c_address: address of I2C device on bus
  // return value: always returns true
  virtual bool open(int device_id, int i2c_address);

  // close I2C device
  virtual void close();

  // read from the memory map, simulating an I2C device
  // i2c_register: register at which to start the read
  // num_of_bytes: number of bytes to read
  // buffer: buffer to store the bytes
  // return value: number of bytes read
  virtual int read(int i2c_register, size_t num_of_bytes, byte* buffer);

  // write to the memory map, simulating an I2C device
  // i2c_register: register at which to start the write
  // num_of_bytes: number of bytes to write
  // buffer: buffer containing the bytes to write
  // return value: number of bytes written
  virtual int write(int i2c_register, size_t num_of_bytes, const byte* buffer);

 private:
  device_memory_map& memory_map_;
};

};  // namespace isaac
