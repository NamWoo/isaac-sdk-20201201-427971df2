/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/composite/gems/typed_composite_view.hpp"

#include "capnp/serialize.h"
#include "engine/core/tensor/tensor.hpp"
#include "gtest/gtest.h"
#include "messages/element_type.hpp"
#include "messages/math.hpp"
#include "messages/tensor.hpp"
#include "packages/composite/gems/measure.hpp"
#include "packages/composite/gems/parser.hpp"
#include "packages/composite/gems/serializer.hpp"

namespace isaac {

template <template<int> class Base>
struct Foo : public Base<3> {
  ISAAC_COMPOSITE_SCALAR(0, t);
  ISAAC_COMPOSITE_SCALAR(1, px);
  ISAAC_COMPOSITE_SCALAR(2, py);
};

TEST(CompositeParser, TypedCompositeView) {
  // Source schema
  const composite::Schema source_schema({
    composite::Quantity::Vector("foo", composite::Measure::kSpeed, 2),
    composite::Quantity::Vector("bar", composite::Measure::kSpeed, 2),
    composite::Quantity::Vector("foo", composite::Measure::kPosition, 2),
    composite::Quantity::Scalar("time", composite::Measure::kTime)
  });

  // Create a message and write data
  ::capnp::MallocMessageBuilder malloc_builder;
  std::vector<SharedBuffer> buffers;
  auto proto_builder = malloc_builder.getRoot<::CompositeProto>();
  WriteSchema(source_schema, proto_builder);
  {
    Tensor2d tensor(97, 7);
    for (int row = 0; row < tensor.dimensions()[0]; row++) {
      const double time = static_cast<double>(row) * 0.01;
      tensor(row, 0) = std::sin(time * 1.5 + 0.3);
      tensor(row, 1) = std::sin(time * 2.0 + 0.2);
      tensor(row, 2) = std::sin(time * 3.0 - 0.1);
      tensor(row, 3) = std::sin(time * 3.5 - 0.2);
      tensor(row, 4) = std::sin(time * 4.0 - 0.3);
      tensor(row, 5) = std::sin(time * 4.5 - 0.4);
      tensor(row, 6) = time;
    }
    ToProto(std::move(tensor), proto_builder.initValues(), buffers);
  }

  // Generates serialized data
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  ::capnp::SegmentArrayMessageReader reader(malloc_builder.getSegmentsForOutput(), options);
  auto composite_reader = reader.getRoot<::CompositeProto>();

  // Parsing and checking
  composite::Parser parser;

  parser.requestSchema(composite::Schema({
    composite::Quantity::Scalar("time", composite::Measure::kTime),
    composite::Quantity::Vector("foo", composite::Measure::kPosition, 2)
  }));

  composite::TypedCompositeView<const double, Foo> result;
  const bool ok = parser.parse(composite_reader, buffers, result);
  ASSERT_TRUE(ok);

  EXPECT_EQ(result.size(), 97);

  for (int row = 0; row < result.size(); row++) {
    auto state = result[row];
    const double time = static_cast<double>(row) * 0.01;
    EXPECT_EQ(state.t(), time);
    EXPECT_EQ(state.px(), std::sin(time * 4.0 - 0.3));
    EXPECT_EQ(state.py(), std::sin(time * 4.5 - 0.4));
  }
}

}  // namespace isaac
