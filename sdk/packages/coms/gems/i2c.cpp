/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "i2c.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

#include <cmath>

#include "engine/core/logger.hpp"

namespace isaac {

I2c::~I2c() {
  // make sure the device is closed
  close();
}

bool I2c::open(int device_id, int i2c_address) {
  // make sure we don't already have an open device
  close();

  char filename[32] = {'\0'};
  snprintf(filename, sizeof(filename) - 1, "/dev/i2c-%d", device_id);
  file_descriptor_ = ::open(filename, O_RDWR);

  if (file_descriptor_ < 0) {
    logErrno(errno);
    return false;
  }

  // let the system know that this file descriptor represents and I2C device
  if (ioctl(file_descriptor_, I2C_SLAVE, i2c_address) < 0) {
    logErrno(errno);
    close();
    return false;
  }

  return true;
}

void I2c::close() {
  if (file_descriptor_ < 0) return;

  const int rv = ::close(file_descriptor_);
  if (rv < 0) {
    logErrno(errno);
  }

  // the file desciptor is now invalid, mark it as such
  file_descriptor_ = -1;
}

int I2c::read(int i2c_register, size_t num_of_bytes, byte* buffer) {
  int const bytes_read =
      i2c_smbus_read_i2c_block_data(file_descriptor_, i2c_register, num_of_bytes, buffer);

  if (bytes_read < 0) {
    logErrno(errno);
    return 0;  // there was an error, we read 0 bytes
  }

  return bytes_read;
}

int I2c::write(int i2c_register, size_t num_of_bytes, const byte* buffer) {
  const int write_status =
      i2c_smbus_write_i2c_block_data(file_descriptor_, i2c_register, num_of_bytes, buffer);

  if (write_status < 0) {
    logErrno(errno);
    return 0;  // there was an error, we wrote 0 bytes
  }

  return num_of_bytes;
}

// log an errno
void I2c::logErrno(int err_number) {
  // we accept errno as a parameter because it is a global and can change, this
  // way we cache it.

  // don't spam the console with repeating messages
  if (err_number == last_errno_logged_) return;

  last_errno_logged_ = err_number;

  char buffer[128] = {'\0'};
  char* err_msg = strerror_r(err_number, buffer, sizeof(buffer) - 1);
  LOG_ERROR("I2C Error: %s (errno %d)", err_msg, err_number);
}

}  // namespace isaac
