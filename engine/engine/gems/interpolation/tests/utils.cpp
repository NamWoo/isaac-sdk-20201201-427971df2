/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <random>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/interpolation/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Utils, Interpolate) {
  EXPECT_FLOAT_EQ(Interpolate(-0.5, 2.8, 4.2), 2.1);
  EXPECT_FLOAT_EQ(Interpolate(0.0, 2.8, 4.2), 2.8);
  EXPECT_FLOAT_EQ(Interpolate(0.5, 2.8, 4.2), 3.5);
  EXPECT_FLOAT_EQ(Interpolate(1.0, 2.8, 4.2), 4.2);
  EXPECT_FLOAT_EQ(Interpolate(2.0, 2.8, 4.2), 5.6);
}

TEST(Utils, RescaleX) {
  EXPECT_DEATH(Rescale(0.0, 0.0, 0.0), ".?");
  EXPECT_FLOAT_EQ(Rescale(0.0, 2.8, 4.2), -2.0);
  EXPECT_FLOAT_EQ(Rescale(2.8, 2.8, 4.2), 0.0);
  EXPECT_FLOAT_EQ(Rescale(3.5, 2.8, 4.2), 0.5);
  EXPECT_FLOAT_EQ(Rescale(4.2, 2.8, 4.2), 1.0);
  EXPECT_FLOAT_EQ(Rescale(5.6, 2.8, 4.2), 2.0);
}

TEST(Utils, RescaleXY) {
  EXPECT_FLOAT_EQ(Rescale(0.0, 2.8, 4.2, -1.0, 1.0), -5.0);
  EXPECT_FLOAT_EQ(Rescale(2.8, 2.8, 4.2, -1.0, 1.0), -1.0);
  EXPECT_FLOAT_EQ(Rescale(3.5, 2.8, 4.2, -1.0, 1.0), 0.0);
  EXPECT_FLOAT_EQ(Rescale(4.2, 2.8, 4.2, -1.0, 1.0), 1.0);
  EXPECT_FLOAT_EQ(Rescale(5.6, 2.8, 4.2, -1.0, 1.0), 3.0);
}

TEST(Utils, RescaleFromInteger) {
  EXPECT_FLOAT_EQ(RescaleFromInteger(-1, 5, 1.3, 2.8), 1.0);
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 1.3, 2.8), 1.3);
  EXPECT_FLOAT_EQ(RescaleFromInteger(1, 5, 1.3, 2.8), 1.6);
  EXPECT_FLOAT_EQ(RescaleFromInteger(4, 5, 1.3, 2.8), 2.5);
  EXPECT_FLOAT_EQ(RescaleFromInteger(5, 5, 1.3, 2.8), 2.8);
  EXPECT_FLOAT_EQ(RescaleFromInteger(10, 5, 1.3, 2.8), 4.3);
}

TEST(Utils, RescaleFromIntegerOffset) {
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 0.0, 1.3, 2.8), 1.30);
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 0.1, 1.3, 2.8), 1.33);
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 0.5, 1.3, 2.8), 1.45);
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 0.9, 1.3, 2.8), 1.57);
  EXPECT_FLOAT_EQ(RescaleFromInteger(0, 5, 1.0, 1.3, 2.8), 1.60);
  EXPECT_FLOAT_EQ(RescaleFromInteger(1, 5, 0.4, 1.3, 2.8), 1.72);
}

TEST(Utils, RescaleToInteger) {
  EXPECT_EQ(RescaleToInteger(1.0001, 1.3, 2.8, 5), -1);
  EXPECT_EQ(RescaleToInteger(1.2999, 1.3, 2.8, 5), -1);
  EXPECT_EQ(RescaleToInteger(1.3, 1.3, 2.8, 5), 0);
  EXPECT_EQ(RescaleToInteger(1.5999, 1.3, 2.8, 5), 0);
  EXPECT_EQ(RescaleToInteger(1.6, 1.3, 2.8, 5), 1);
  EXPECT_EQ(RescaleToInteger(2.8, 1.3, 2.8, 5), 5);
  EXPECT_EQ(RescaleToInteger(4.3, 1.3, 2.8, 5), 10);
}

TEST(Utils, RescaleFromToInteger) {
  const double x = 50.0;
  const int n = RescaleToInteger(x, 0.0, 127.5, 0xff);
  const double y = RescaleFromInteger(n, 0xff, 0.0, 127.5);
  EXPECT_NEAR(x, y, 1e-9);
}

TEST(WrapPi, tests) {
  EXPECT_NEAR(WrapPi(Pi<double>), Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(-Pi<double>), Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(0.5), 0.5, 1e-7);
  EXPECT_NEAR(WrapPi(-0.5), -0.5, 1e-7);
  EXPECT_NEAR(WrapPi(Pi<double> + 0.5), 0.5 - Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(-Pi<double> - 0.5), -0.5 + Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(-10.0 * Pi<double> - 0.1), -0.1, 1e-7);
  EXPECT_NEAR(WrapPi(-12.0 * Pi<double> + 0.2), +0.2, 1e-7);
  EXPECT_NEAR(WrapPi(+14.0 * Pi<double> - 0.3), -0.3, 1e-7);
  EXPECT_NEAR(WrapPi(+16.0 * Pi<double> + 0.4), +0.4, 1e-7);

  EXPECT_NEAR(WrapPi(-11.0 * Pi<double> - 0.5), -0.5 + Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(-13.0 * Pi<double> + 0.6), +0.6 - Pi<double>, 1e-7);
  EXPECT_NEAR(WrapPi(+15.0 * Pi<double> - 0.7), -0.7 + Pi<double>, 1e-7);

  EXPECT_NEAR(WrapPi(+17.0 * Pi<double> + 0.8), +0.8 - Pi<double>, 1e-7);
}

}  // namespace isaac
