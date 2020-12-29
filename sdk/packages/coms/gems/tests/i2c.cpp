/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/coms/gems/i2c.hpp"

#include "gtest/gtest.h"

namespace {
constexpr int TEST_DEVICE_I2C_ADDR = 0x69;
constexpr int TEST_DEVICE_CHIP_ID = 0xd1;
constexpr int TEST_DEVICE_TEST_REG = 0x18;
constexpr int TEST_DEVICE_TEST_READ_SIZE = 3;
}  // namespace

TEST(I2c, IsConstructible) {
  // if this test compiles, it passes
  isaac::I2c i2c;
}

TEST(I2c, open) {
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);
}

TEST(I2c, open_and_close) {
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);

  i2c.close();
}

TEST(I2c, open_and_reopen) {
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);

  isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);
}

TEST(I2c, read) {
  std::array<isaac::byte, 8> buffer = {'\0'};
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);

  // read the chip ID and see if we get the expected result
  int bytes_read = i2c.read(0x00, 1, buffer.data());
  EXPECT_EQ(bytes_read, 1);
  EXPECT_EQ(buffer[0], TEST_DEVICE_CHIP_ID);
}

TEST(I2c, read_large) {
  // in this test we are going to try to read an amount bigger than the max
  // I2C read size (32 bytes), and make sure the read succeeds.

  const int PASS_THRESHOLD = 20;

  std::array<isaac::byte, 0x7e> buffer = {'\0'};
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);

  // read device and populate the buffer
  int total_bytes_read = 0;
  while (total_bytes_read < buffer.size()) {
    const int bytes_read = i2c.read(
        0x00, buffer.size() - total_bytes_read, buffer.data() + total_bytes_read);

    ASSERT_GT(bytes_read, 0); // fail this test if we get a zero byte read
    total_bytes_read += bytes_read;
  }
  EXPECT_EQ(total_bytes_read, buffer.size());

  // now that we have read a large number of bytes, count how many are non-zero
  int non_zero = 0;
  for (const auto& b : buffer) {
    if (b != 0) non_zero += 1;
  }

  // we don't know the state of the device we are reading, but assume that if
  // at least PASS_THRESHOLD number of bytes are non zero, we got a valid read.
  EXPECT_GE(non_zero, PASS_THRESHOLD);
}

TEST(I2c, write) {
  std::array<isaac::byte, 8> buffer = {'\0'};
  isaac::I2c i2c;

  bool isOpen = i2c.open(1, TEST_DEVICE_I2C_ADDR);
  EXPECT_TRUE(isOpen);

  // write to the chip ID and see if we get the expected result
  buffer[0] = 0x42;  // some arbitrary value
  const int bytes_written = i2c.write(0x00, 1, buffer.data());
  EXPECT_EQ(bytes_written, 1);
}
