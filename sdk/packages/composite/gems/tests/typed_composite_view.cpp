/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/composite/gems/typed_composite_view.hpp"

#include "engine/core/tensor/tensor.hpp"
#include "gtest/gtest.h"

namespace isaac {

template <template<int> class Base>
struct Foo : public Base<2> {
  ISAAC_COMPOSITE_SCALAR(0, hello);
  ISAAC_COMPOSITE_SCALAR(1, world);
};
using FooComposite = composite::TypedCompositeView<double, Foo>;

template <template<int> class Base>
struct Bar : public Base<4> {
  ISAAC_COMPOSITE_SCALAR(0, a);
  ISAAC_COMPOSITE_SCALAR(1, b);
  ISAAC_COMPOSITE_SCALAR(2, c);
  ISAAC_COMPOSITE_SCALAR(3, d);
};
using BarComposite = composite::TypedCompositeView<double, Bar>;

TEST(Composite, Test) {
  // Wrong rank
  Tensor3d wrong_rank_tensor(10, 4, 4);
  EXPECT_DEATH(BarComposite::FromTensor(wrong_rank_tensor), ".*");

  // Does not compile
  // Tensor2i wrong_type_tensor(4, 5);
  // std::ignore = BarComposite::FromTensor(wrong_type_tensor);

  // Wrong dimension
  Tensor2d wrong_dimension_tensor(10, 3);
  EXPECT_DEATH(BarComposite::FromTensor(wrong_dimension_tensor), ".*");

  Tensor2d tensor(10, 4);
  BarComposite bars = BarComposite::FromTensor(tensor);
  for (int i = 0; i < tensor.dimensions()[0]; i++) {
    const double q = static_cast<double>(i);
    bars[i].a() = q;
    bars[i].b() = 2.0*q;
    bars[i].c() = 3.0*q;
    bars[i].d() = 4.0*q;
  }

  const std::array<int, 2> invalid_offsets{1, 4};
  const std::array<int, 2> offsets{1, 3};

  {
    FooComposite null(nullptr, 0, 4, offsets);
    EXPECT_EQ(null.size(), 0);
  }
  EXPECT_DEATH(FooComposite(nullptr, tensor.dimensions()[0],
                            tensor.dimensions()[1], offsets), ".*");
  EXPECT_DEATH(FooComposite(tensor.element_wise_begin(), tensor.dimensions()[0],
                            tensor.dimensions()[1], invalid_offsets), ".*");

  const FooComposite x = FooComposite::FromTensor(tensor, offsets);
  for (int i = 0; i < x.size(); i++) {
    const double q = static_cast<double>(i);
    ASSERT_EQ(x[i].hello(), 2.0 * q);
    ASSERT_EQ(x[i].world(), 4.0 * q);
  }
}

}  // namespace isaac
