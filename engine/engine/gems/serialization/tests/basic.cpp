/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/serialization/header.hpp"
#include "gtest/gtest.h"

#include <numeric>

namespace isaac {
namespace serialization {

TEST(Serialization, Empty) {
  serialization::Header header;
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(header, buffer);
  ASSERT_TRUE(ok);
  ASSERT_EQ(buffer.size(), 4);
  EXPECT_EQ(buffer[0], 0);
  EXPECT_EQ(buffer[1], 0);
  EXPECT_EQ(buffer[2], 0);
  EXPECT_EQ(buffer[3], 0);
}

TEST(Serialization, WriteReadTimestamp) {
  serialization::Header expected;
  expected.timestamp = 123456789;
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(expected, buffer);
  ASSERT_TRUE(ok);
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_TRUE(actual.timestamp);
  EXPECT_EQ(*actual.timestamp, *expected.timestamp);
}

TEST(Serialization, WriteRead) {
  serialization::Header expected;
  expected.timestamp = 123456789;
  expected.uuid = Uuid::Generate();
  expected.tag = "so long and thanks for all the fish";
  expected.acqtime = 23486765552;
  expected.minipayload.resize(12);
  std::iota(expected.minipayload.begin(), expected.minipayload.end(), 0);
  expected.segments = {15664, 123, 17, 933};
  expected.buffers = {3422, 1288493, 229434};
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(expected, buffer);
  EXPECT_EQ(buffer.size(), 4 + 8 + 16 + (1 + 35) + 8 + (1 + 12) + (1 + 4 * 4) + (1 + 3 * 4));
  ASSERT_TRUE(ok);
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_TRUE(actual.timestamp);
  EXPECT_EQ(*actual.timestamp, *expected.timestamp);
  EXPECT_TRUE(actual.uuid);
  EXPECT_EQ(*actual.uuid, *expected.uuid);
  EXPECT_EQ(actual.tag, expected.tag);
  EXPECT_TRUE(actual.acqtime);
  EXPECT_EQ(*actual.acqtime, *expected.acqtime);
  EXPECT_EQ(actual.minipayload, expected.minipayload);
  EXPECT_EQ(actual.segments, expected.segments);
  EXPECT_EQ(actual.buffers, expected.buffers);
}

TEST(Serialization, MaxMessage) {
  serialization::Header expected;
  expected.timestamp = 123456789;
  expected.uuid = Uuid::Generate();
  expected.tag = std::string(kHeaderMaxVectorSize, 'x');
  expected.acqtime = 23486765552;
  expected.format = 1337;
  expected.minipayload.resize(kHeaderMaxVectorSize);
  std::iota(expected.minipayload.begin(), expected.minipayload.end(), 0);
  expected.segments.resize(kHeaderMaxVectorSize);
  std::iota(expected.segments.begin(), expected.segments.end(), 0);
  expected.buffers.resize(kHeaderMaxVectorSize);
  std::iota(expected.buffers.begin(), expected.buffers.end(), 0);
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(expected, buffer);
  EXPECT_EQ(buffer.size(), kHeaderMaxLength);
  ASSERT_TRUE(ok);
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_TRUE(actual.timestamp);
  EXPECT_EQ(*actual.timestamp, *expected.timestamp);
  EXPECT_TRUE(actual.uuid);
  EXPECT_EQ(*actual.uuid, *expected.uuid);
  EXPECT_EQ(actual.tag, expected.tag);
  EXPECT_TRUE(actual.acqtime);
  EXPECT_EQ(*actual.acqtime, *expected.acqtime);
  EXPECT_EQ(actual.minipayload, expected.minipayload);
  EXPECT_EQ(actual.segments, expected.segments);
  EXPECT_EQ(actual.buffers, expected.buffers);
}

TEST(Serialization, TagPadding) {
  serialization::Header expected;
  expected.tag = "aaa";
  expected.acqtime = 123456789;
  std::vector<uint8_t> buffer(kHeaderMaxLength);
  std::fill(buffer.begin(), buffer.end(), 137);
  const bool ok = Serialize(expected, buffer);
  ASSERT_TRUE(ok);
  ASSERT_EQ(buffer.size(), 16);
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_EQ(actual.tag, expected.tag);
  EXPECT_TRUE(actual.acqtime);
  EXPECT_EQ(*actual.acqtime, *expected.acqtime);
}

void TestTag(const std::string& tag, bool expected_ok = true) {
  serialization::Header expected;
  expected.tag = tag;
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(expected, buffer);
  ASSERT_EQ(ok, expected_ok);
  if (!expected_ok) {
    return;
  }
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_EQ(actual.tag, expected.tag);
}

TEST(Serialization, WriteReadTag) {
  TestTag("");
  TestTag("1");
  TestTag("11");
  TestTag("111");
  TestTag("1111");
  TestTag("11111");
  TestTag(std::string(kHeaderMaxVectorSize, '1'));
  TestTag(std::string(kHeaderMaxVectorSize + 1, '1'), false);
}

void TestMinipayload(size_t count, size_t buffer_length, bool expected_ok = true) {
  serialization::Header expected;
  expected.minipayload.resize(count);
  std::iota(expected.minipayload.begin(), expected.minipayload.end(), 0);
  std::vector<uint8_t> buffer;
  const bool ok = Serialize(expected, buffer);
  ASSERT_EQ(ok, expected_ok);
  EXPECT_EQ(buffer.size(), buffer_length);
  if (!expected_ok) {
    return;
  }
  serialization::Header actual;
  const bool ok2 = Deserialize(buffer, actual);
  ASSERT_TRUE(ok2);
  EXPECT_EQ(actual.minipayload, expected.minipayload);
}

TEST(Serialization, Minipayload) {
  TestMinipayload(0, 4);
  TestMinipayload(1, 6);
  TestMinipayload(2, 7);
  TestMinipayload(3, 8);
  TestMinipayload(4, 9);
  TestMinipayload(100, 105);
  TestMinipayload(kHeaderMaxVectorSize, kHeaderMaxVectorSize + 5);
  TestMinipayload(kHeaderMaxVectorSize + 1, 0, false);
}

TEST(Serialization, TimestampUuidNoTip) {
  const int magic = TIP_1_TIMESTAMP | TIP_2_UUID;
  serialization::Header expected;
  expected.timestamp = 123456789;
  expected.uuid = Uuid::Generate();
  std::vector<uint8_t> buffer;
  const bool ok = SerializeWithoutTip(expected, magic, buffer);
  EXPECT_EQ(buffer.size(), 24);
  ASSERT_TRUE(ok);
  serialization::Header actual;
  const bool ok2 = DeserializeWithoutTip(buffer, magic, actual);
  ASSERT_TRUE(ok2);
  EXPECT_TRUE(actual.timestamp);
  EXPECT_EQ(*actual.timestamp, *expected.timestamp);
  EXPECT_TRUE(actual.uuid);
  EXPECT_EQ(*actual.uuid, *expected.uuid);
}

}  // namespace serialization
}  // namespace isaac
