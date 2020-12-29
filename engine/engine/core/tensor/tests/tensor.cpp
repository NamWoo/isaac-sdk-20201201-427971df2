/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/tensor/tensor.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace isaac {

TEST(Tensor, Rank) {
  Tensor1d t1(13);
  EXPECT_EQ(t1.order(), 1);
  EXPECT_EQ(t1.dimensions().size(), 1);
  EXPECT_EQ(t1.dimensions()[0], 13);

  Tensor2d t2(13, 17);
  EXPECT_EQ(t2.order(), 2);
  EXPECT_EQ(t2.dimensions().size(), 2);
  EXPECT_EQ(t2.dimensions()[0], 13);
  EXPECT_EQ(t2.dimensions()[1], 17);

  Tensor3d t3(13, 17, 19);
  EXPECT_EQ(t3.order(), 3);
  EXPECT_EQ(t3.dimensions().size(), 3);
  EXPECT_EQ(t3.dimensions()[0], 13);
  EXPECT_EQ(t3.dimensions()[1], 17);
  EXPECT_EQ(t3.dimensions()[2], 19);

  Tensor4d t4(13, 17, 19, 23);
  EXPECT_EQ(t4.order(), 4);
  EXPECT_EQ(t4.dimensions().size(), 4);
  EXPECT_EQ(t4.dimensions()[0], 13);
  EXPECT_EQ(t4.dimensions()[1], 17);
  EXPECT_EQ(t4.dimensions()[2], 19);
  EXPECT_EQ(t4.dimensions()[3], 23);

  Tensor<double, 14> t14;
  EXPECT_EQ(t14.order(), 14);
  EXPECT_EQ(t14.dimensions().size(), 14);
}

TEST(Tensor, FixedSize) {
  constexpr unsigned kTwenty = 20;
  TensorBase<float, tensor::Dimensions<kTwenty, 30, 3>, CpuBuffer> t1;
  EXPECT_EQ(t1.order(), 3);
  EXPECT_EQ(t1.dimensions().size(), 3);
  EXPECT_EQ(t1.dimensions()[0], kTwenty);
  EXPECT_EQ(t1.dimensions()[1], 30);
  EXPECT_EQ(t1.dimensions()[2], 3);

  constexpr unsigned kUnsignedTwenty = static_cast<unsigned>(kTwenty);
  TensorBase<float, tensor::Dimensions<kUnsignedTwenty, 30, 3>, CpuBuffer> t2;
  EXPECT_EQ(t2.order(), 3);
  EXPECT_EQ(t2.dimensions()[0], kTwenty);

  constexpr size_t kSizeTwenty = static_cast<size_t>(kTwenty);
  TensorBase<float, tensor::Dimensions<kSizeTwenty, 30, 3>, CpuBuffer> t3;
  EXPECT_EQ(t3.order(), 3);
  EXPECT_EQ(t3.dimensions()[0], kTwenty);
}

TEST(Tensor, MemoryAccess) {
  {
    // default row order test
    Tensor4i t(2, 3, 4, 5);
    ASSERT_EQ(t.dimensions().size(), 4);
    ASSERT_EQ(t.dimensions()[0], 2);
    ASSERT_EQ(t.dimensions()[1], 3);
    ASSERT_EQ(t.dimensions()[2], 4);
    ASSERT_EQ(t.dimensions()[3], 5);
    ASSERT_EQ(t.data().size(), 4*5*4*3*2);

    int count = 0;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 4; ++k) {
          for (int l = 0; l < 5; ++l) {
            t(i, j, k, l) = ++count;
          }
        }
      }
    }

    t(0, 0, 0, 4) = 2;
    t(1, 2, 3, 4) = 15;
    t(1, 1, 3, 1) = 17;

    const Vector4i shifts{3*4*5, 4*5, 5, 1};

    EXPECT_EQ(*(t.element_wise_begin() + shifts.dot(Vector4i{0, 0, 0, 4})), 2);
    EXPECT_EQ(*(t.element_wise_begin() + shifts.dot(Vector4i{1, 2, 3, 4})), 15);
    EXPECT_EQ(*(t.element_wise_begin() + shifts.dot(Vector4i{1, 1, 3, 1})), 17);
  }
}

TEST(Tensor, Allocation) {
  Tensor3d t3;
  Tensor1d t1;
  t1 = Tensor1d(15);
  t3 = Tensor3d(30, 60, 15);
  auto dim3d = t3.dimensions();
  auto dim1d = t1.dimensions();
  EXPECT_EQ(dim3d[0], 30);
  EXPECT_EQ(dim3d[1], 60);
  EXPECT_EQ(dim3d[2], 15);
  EXPECT_EQ(dim1d[0], 15);
}

TEST(Tensor, Validation) {
  Tensor1d t1;
  t1 = Tensor1d(0);
  auto dim1d = t1.dimensions();
  EXPECT_EQ(dim1d[0], 0);
}

TEST(Tensor, Views) {
  Tensor4i t(2, 3, 4, 5);
  t(0, 0, 0, 4) = 2;
  t(1, 2, 3, 4) = 15;

  TensorView4i v = t.view();
  EXPECT_EQ(v(0, 0, 0, 4), 2);
  EXPECT_EQ(v(1, 2, 3, 4), 15);

  v(0, 0, 0, 4) = 22;
  EXPECT_EQ(v(0, 0, 0, 4), 22);
}

TEST(Tensor, ConstViews) {
  Tensor4i t(2, 3, 4, 5);
  t(0, 0, 0, 4) = 2;
  t(1, 2, 3, 4) = 15;

  TensorConstView4i v = t.const_view();

  EXPECT_EQ(v(0, 0, 0, 4), 2);
  EXPECT_EQ(v(1, 2, 3, 4), 15);
}

TEST(Tensor, ConstAccess) {
  Tensor3d t3(20, 40, 59);
  Tensor1d t1(30);
  const Tensor1d& tref1 = t1;
  const Tensor3d& tref3 = t3;
  t3(3, 5, 1) = 12.0;
  EXPECT_EQ(tref3(3, 5, 1), 12.0);
  EXPECT_EQ(tref3(3, 5, 1), 12.0);

  t1(3) = 12.0;
  EXPECT_EQ(tref1(3), 12.0);
}

TEST(Tensor, ConvertViewToConstView) {
  Vector3<int> dims = {12, 13, 14};
  Tensor3ub t(dims);
  TensorView3ub view1 = t.view();
  TensorConstView3ub view2 = view1;
  EXPECT_EQ(view2.dimensions(), dims);
}

TEST(Tensor, EmptyView) {
  Vector3<int> dims = {0, 0, 0};
  TensorView3ub view;
  EXPECT_EQ(view.element_count(), 0);
  EXPECT_EQ(dims, view.dimensions());
}

TEST(Tensor, EmptyConstView) {
  Vector3<int> dims = {0, 0, 0};
  TensorConstView3ub view;
  EXPECT_EQ(view.element_count(), 0);
  EXPECT_EQ(dims, view.dimensions());
}

TEST(Tensor, CopyView) {
  Vector3<int> dims = {12, 13, 14};
  Tensor3ub t(dims);
  TensorView3ub view1 = t.view();
  TensorView3ub view2;
  view2 = view1;
  EXPECT_NE(view2.element_count(), 0);
  EXPECT_EQ(view2.dimensions(), dims);
}

TEST(Tensor, CopyConstView) {
  Vector3<int> dims = {12, 13, 14};
  Tensor3ub t(dims);
  TensorConstView3ub view1 = t.const_view();
  TensorConstView3ub view2;
  view2 = view1;
  EXPECT_NE(view2.element_count(), 0);
  EXPECT_EQ(view2.dimensions(), dims);
}

namespace {

void FooReadable(const TensorConstView1ub& view) {}

void FooWriteable(const TensorView1ub& view) {}

template <typename Dimensions, typename Container>
void FooTemplated(const TensorBase<uint8_t, Dimensions, Container>& view) {}

}  // namespace

TEST(Tensor, CallFunctionReadable) {
  Tensor1ub t(10);
  FooReadable(t);
  FooReadable(t.view());
  FooReadable(t.const_view());
}

TEST(Tensor, CallFunctionWriteable) {
  Tensor1ub t(10);
  FooWriteable(t);
  FooWriteable(t.view());
  // FooWriteable(t.const_view());  // should not compile
}

TEST(Tensor, CallFunctionTemplated) {
  Tensor1ub t1(10);
  FooTemplated(t1);
  FooTemplated(t1.view());
  FooTemplated(t1.const_view());
  Tensor3ub t3(10, 20, 3);
  FooTemplated(t3);
  FooTemplated(t3.view());
  FooTemplated(t3.const_view());
}

TEST(Tensor, ResizeReallocatesIfNewDimsAreDifferent) {
  Tensor3ub t(10, 15, 3);
  EXPECT_EQ(t.element_count(), 10 * 15 * 3);
  t.resize(11, 20, 3);
  EXPECT_EQ(t.element_count(), 11 * 20 * 3);
  t.resize(10, 15, 3);
  EXPECT_EQ(t.element_count(), 10 * 15 * 3);
}

TEST(Tensor, Slice) {
  Tensor3i tensor(9, 7, 3);
  for (int i0 = 0; i0 < tensor.dimensions()[0]; i0++) {
    for (int i1 = 0; i1 < tensor.dimensions()[1]; i1++) {
      for (int i2 = 0; i2 < tensor.dimensions()[2]; i2++) {
        tensor(i0, i1, i2) = i0 * 10000 + i1 * 100 + i2;
      }
    }
  }
  for (int i0 = 0; i0 < tensor.dimensions()[0]; i0++) {
    TensorConstView2i slice = tensor.slice(i0);
    ASSERT_EQ(tensor.dimensions()[1], slice.dimensions()[0]);
    ASSERT_EQ(tensor.dimensions()[2], slice.dimensions()[1]);
    for (int i1 = 0; i1 < tensor.dimensions()[0]; i1++) {
      for (int i2 = 0; i2 < tensor.dimensions()[1]; i2++) {
        ASSERT_EQ(slice(i1, i2), tensor(i0, i1, i2));
      }
    }
  }
}

TEST(Tensor, VectorAccess1) {
  // Check that a operator(row, col) for a rank 3 tensor returns a size 3 vector.
  const Vector3f expected{-0.1f, 1.2f, -2.3f};
  Tensor3f tensor(20, 30, 3);
  tensor(4, 6, 0) = expected[0];
  tensor(4, 6, 1) = expected[1];
  tensor(4, 6, 2) = expected[2];
  const Vector3f actual = tensor(4, 6);
  EXPECT_EQ(expected[0], actual[0]);
  EXPECT_EQ(expected[1], actual[1]);
  EXPECT_EQ(expected[2], actual[2]);
}

TEST(Tensor, VectorAccess2) {
  // Check that a operator(row, col) for a rank 3 tensor can be used to assign a size 3 vector.
  const Vector3f expected{-0.1f, 1.2f, -2.3f};
  Tensor3f tensor(20, 30, 3);
  tensor(4, 6) = expected;
  const Vector3f actual = tensor(4, 6);
  EXPECT_EQ(expected[0], actual[0]);
  EXPECT_EQ(expected[1], actual[1]);
  EXPECT_EQ(expected[2], actual[2]);
}

TEST(Tensor, VectorAccess3) {
  // Check that operator(row, col) for a rank 3 tensor with compile-time channel size returns a 3x1
  // vector with compile-time row count.
  TensorBase<float, tensor::Dimensions<20, tensor::kDynamic, 3>, CpuBuffer> tensor(20, 30, 3);
  EXPECT_EQ(tensor.order(), 3);
  EXPECT_EQ(tensor.dimensions().size(), 3);
  EXPECT_EQ(tensor.dimensions()[0], 20);
  EXPECT_EQ(tensor.dimensions()[1], 30);
  EXPECT_EQ(tensor.dimensions()[2], 3);
  EXPECT_EQ(tensor(4, 6).RowsAtCompileTime, 3);
  EXPECT_EQ(tensor(4, 6).ColsAtCompileTime, 1);
  EXPECT_EQ(tensor(4, 6).rows(), 3);
  EXPECT_EQ(tensor(4, 6).cols(), 1);
}

TEST(Tensor, VectorAccess4) {
  // Check that operator(row, col) for a rank 3 tensor with dynamic channel size returns a Xx1
  // vector with runtime row count.
  Tensor3f tensor(20, 30, 3);
  EXPECT_EQ(tensor.order(), 3);
  EXPECT_EQ(tensor.dimensions().size(), 3);
  EXPECT_EQ(tensor.dimensions()[0], 20);
  EXPECT_EQ(tensor.dimensions()[1], 30);
  EXPECT_EQ(tensor.dimensions()[2], 3);
  EXPECT_EQ(tensor(4, 6).RowsAtCompileTime, -1);
  EXPECT_EQ(tensor(4, 6).ColsAtCompileTime, 1);
  EXPECT_EQ(tensor(4, 6).rows(), 3);
  EXPECT_EQ(tensor(4, 6).cols(), 1);
}

TEST(Tensor, VectorAccess5) {
  // Check that operator(row, col) for a rank 2 tensor represents a single element (read/write).
  Tensor2f tensor(20, 30);
  EXPECT_EQ(tensor.order(), 2);
  EXPECT_EQ(tensor.dimensions().size(), 2);
  EXPECT_EQ(tensor.dimensions()[0], 20);
  EXPECT_EQ(tensor.dimensions()[1], 30);
  constexpr float kExpected1 = -1.7f;
  tensor(4, 6) = kExpected1;
  EXPECT_EQ(tensor(4, 6), kExpected1);
  constexpr float kExpected2 = 0.3f;
  tensor(4, 6) = kExpected2;
  EXPECT_EQ(tensor(4, 6), kExpected2);
}

TEST(Tensor, MatrixAccess) {
  Tensor4i tensor(3, 4, 5, 6);
  for (int i0 = 0; i0 < tensor.dimensions()[0]; i0++) {
    for (int i1 = 0; i1 < tensor.dimensions()[1]; i1++) {
      for (int i2 = 0; i2 < tensor.dimensions()[2]; i2++) {
        for (int i3 = 0; i3 < tensor.dimensions()[3]; i3++) {
          tensor(i0, i1, i2, i3) = i0 * 1000 + i1 * 100 + i2 * 10 + i3;
        }
      }
    }
  }
  for (int i0 = 0; i0 < tensor.dimensions()[0]; i0++) {
    for (int i1 = 0; i1 < tensor.dimensions()[1]; i1++) {
      Eigen::Matrix<int, 5, 6> matrix = tensor(i0, i1);
      for (int i2 = 0; i2 < tensor.dimensions()[2]; i2++) {
        for (int i3 = 0; i3 < tensor.dimensions()[3]; i3++) {
          EXPECT_EQ(matrix(i2, i3), tensor(i0, i1, i2, i3));
        }
      }
    }
  }
  for (int i0 = 0; i0 < tensor.dimensions()[0]; i0++) {
    for (int i1 = 0; i1 < tensor.dimensions()[1]; i1++) {
      for (int i2 = 0; i2 < tensor.dimensions()[2]; i2++) {
        Eigen::Matrix<int, 1, 6> matrix = tensor(i0, i1, i2);
        for (int i3 = 0; i3 < tensor.dimensions()[3]; i3++) {
          EXPECT_EQ(matrix(i3), tensor(i0, i1, i2, i3));
        }
      }
    }
  }
}

TEST(Tensor, LinearAlgebra) {
  Tensor3d data(10, 100, 100);
  for (int k = 0; k < data.dimensions()[0]; k++) {
    const double center = static_cast<double>(k) * 0.2 - 1.0;
    const auto delta = VectorXd::LinSpaced(100, -1.0, +1.0) - VectorXd::Constant(100, center);
    data(k) = delta * delta.transpose() + MatrixXd::Constant(100, 100, 0.7);
    for (int i = 0; i < data.dimensions()[1]; i++) {
      for (int j = 0; j < data.dimensions()[2]; j++) {
        const double y = static_cast<double>(j) * 2.0 / 99.0 - 1.0;
        const double x = static_cast<double>(i) * 2.0 / 99.0 - 1.0;
        const double expected = (x - center) * (y - center) + 0.7;
        ASSERT_NEAR(data(k, i, j), expected, 1e-9);
      }
    }
  }
}

}  // namespace isaac
