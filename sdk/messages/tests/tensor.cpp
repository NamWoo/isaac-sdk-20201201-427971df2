/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "messages/tensor.hpp"

#include <vector>

#include "capnp/message.h"
#include "gtest/gtest.h"

namespace isaac {

TEST(Tensor, Tensor3iToFromProto) {
  Tensor3i tensor(23, 49, 18);

  ::capnp::MallocMessageBuilder message;
  std::vector<SharedBuffer> buffers;
  ToProto(std::move(tensor), message.initRoot<TensorProto>(), buffers);

  ::capnp::SegmentArrayMessageReader reader(message.getSegmentsForOutput());
  TensorConstView3i view;
  EXPECT_TRUE(FromProto(reader.getRoot<TensorProto>(), buffers, view));
  EXPECT_EQ(view.dimensions(), tensor.dimensions());
}

TEST(Tensor, UniversalTensorToFromProto) {
  Tensor3i tensor(23, 49, 18);

  ::capnp::MallocMessageBuilder message;
  std::vector<SharedBuffer> buffers;
  ToProto(std::move(tensor), message.initRoot<TensorProto>(), buffers);

  ::capnp::SegmentArrayMessageReader reader(message.getSegmentsForOutput());
  CpuUniversalTensorConstView dynamic_view;
  EXPECT_TRUE(FromProto(reader.getRoot<TensorProto>(), buffers, dynamic_view));
  EXPECT_EQ(dynamic_view.dimensions(), tensor.dimensions());
  EXPECT_EQ(dynamic_view.element_type(), ElementType::kInt32);
}

TEST(Tensor, Tensor3iToFromProtoFailure) {
  Tensor3i tensor(23, 49, 18);

  ::capnp::MallocMessageBuilder message;
  std::vector<SharedBuffer> buffers;
  ToProto(std::move(tensor), message.initRoot<TensorProto>(), buffers);

  ::capnp::SegmentArrayMessageReader reader(message.getSegmentsForOutput());
  TensorConstView3f view;
  EXPECT_FALSE(FromProto(reader.getRoot<TensorProto>(), buffers, view));
}

}  // namespace isaac
