/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstddef>

#include "engine/core/byte.hpp"

namespace isaac {

// Class for using I2C Devices
class I2c {
 public:
  I2c() = default;

  I2c(I2c&& rhs) = delete;
  I2c& operator=(I2c&& rhs) = delete;

  I2c(const I2c& rhs) = delete;
  I2c& operator=(const I2c& rhs) = delete;

  // closes I2C device
  virtual ~I2c();

  // open I2C device
  // if the device is already open, it will close the old device
  // device_id: matches ID of /dev/i2c-X
  // i2c_address: address of I2C device on bus
  // return value: true on success, otherwise false
  virtual bool open(int device_id, int i2c_address);

  // close I2C device
  virtual void close();

  // read from I2C device
  // i2c_register: register at which to start the read
  // num_of_bytes: number of bytes to read
  // buffer: buffer to store the bytes
  // return value: number of bytes read
  virtual int read(int i2c_register, size_t num_of_bytes, byte* buffer);

  // write to I2C device
  // i2c_register: register at which to start the write
  // num_of_bytes: number of bytes to write
  // buffer: buffer containing the bytes to write
  // return value: number of bytes written
  virtual int write(int i2c_register, size_t num_of_bytes, const byte* buffer);

 private:
  // log the errno as a human readable message
  void logErrno(int err_number);

  // file descripter of device
  int file_descriptor_ = -1;

  // last error logged
  int last_errno_logged_ = 0;  // errno of 0 is considered no error via spec,
                               // so we choose that as our starting value
};

};  // namespace isaac
