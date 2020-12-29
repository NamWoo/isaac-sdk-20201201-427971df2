/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <vector>

#include "capnp/message.h"
#include "capnp/serialize-packed.h"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "engine/gems/serialization/tests/test.capnp.h"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

TEST(Serialization, StringToCapnpBuffer) {
  // initialize a message
  ::capnp::MallocMessageBuilder message;
  CapnpTest::Builder test = message.initRoot<CapnpTest>();
  test.setNumber(42);
  test.setAmount(3.1415);

  // segments to string
  CpuBuffer bytes;
  CapnpSegmentsToFlatArray(message.getSegmentsForOutput(), bytes);
  // string to segment lengths and buffer
  std::vector<size_t> segment_lengths;
  std::vector<uint8_t> buffer;
  FlatArrayToCapnpBuffer(std::move(bytes), segment_lengths, buffer);

  // copied from message.cpp
  std::vector<kj::ArrayPtr<const ::capnp::word>> segments;
  segments.reserve(segment_lengths.size());
  size_t offset = 0;
  for (size_t i = 0; i < segment_lengths.size(); i++) {
    const size_t length = segment_lengths[i];
    const size_t seg_begin = offset;
    const size_t seg_end = offset + length;
    offset += length;
    ASSERT(seg_begin % sizeof(::capnp::word) == 0, "corrupted data");
    ASSERT(seg_end % sizeof(::capnp::word) == 0, "corrupted data");
    const ::capnp::word* buffer_words_begin =
        reinterpret_cast<const ::capnp::word*>(buffer.data() + seg_begin);
    const ::capnp::word* buffer_words_end =
        reinterpret_cast<const ::capnp::word*>(buffer.data() + seg_end);
    segments.push_back(kj::ArrayPtr<const ::capnp::word>(buffer_words_begin, buffer_words_end));
  }
  kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> capnp_segments(segments.data(),
                                                                       segments.size());
  auto reader = new ::capnp::SegmentArrayMessageReader(capnp_segments);

  // check if the values are preserved
  auto test_reader = reader->getRoot<CapnpTest>();
  int number = test_reader.getNumber();
  double amount = test_reader.getAmount();
  ASSERT_NEAR(amount, 3.1415, 1e-15);
  ASSERT_EQ(number, 42);
}

}  // namespace serialization
}  // namespace isaac
