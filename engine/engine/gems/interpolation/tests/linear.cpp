/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <random>

#include "engine/core/image/image.hpp"
#include "engine/gems/interpolation/bilinear_approximated_function.hpp"
#include "engine/gems/interpolation/linear_approximated_function.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Cache, Basics) {
  LinearApproximatedFunction<double,double,32> f(0.0, 1.0, [](double x) { return x; });
  EXPECT_NEAR(f(-1.7), 0.0, 1e-9);
  EXPECT_NEAR(f(0.0), 0.0, 1e-9);
  EXPECT_NEAR(f(0.2), 0.2, 1e-9);
  EXPECT_NEAR(f(1.0), 1.0, 1e-9);
  EXPECT_NEAR(f(1.4), 1.0, 1e-9);
}

TEST(BilinearApproximatedFunction, precompute) {
  Image1d img(10,10);
  const int rows = img.rows();
  const int cols = img.cols();
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < img.cols(); col++) {
      img(row, col) = row + col;
    }
  }
  BilinearApproximatedFunction<double, true, Image1d> interpolation(std::move(img), rows, cols);
  std::mt19937 gen;
  std::uniform_real_distribution<double> distribution(1.0, static_cast<double>(rows - 1));
  for (int i = 0; i < 1000; i++) {
    const double row = distribution(gen);
    const double col = distribution(gen);
    const double v = interpolation(row, col);
    EXPECT_NEAR(v, row + col, 1e-10);
  }
}

TEST(BilinearApproximatedFunction, no_precompute) {
  Image1d img(10,10);
  const int rows = img.rows();
  const int cols = img.cols();
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < img.cols(); col++) {
      img(row, col) = row + col;
    }
  }
  BilinearApproximatedFunction<double, false, Image1d> interpolation(std::move(img), rows, cols);
  std::mt19937 gen;
  std::uniform_real_distribution<double> distribution(1.0, static_cast<double>(rows - 1));
  for (int i = 0; i < 1000; i++) {
    const double row = distribution(gen);
    const double col = distribution(gen);
    const double v = interpolation(row, col);
    EXPECT_NEAR(v, row + col, 1e-10);
  }
}

}  // namespace isaac
