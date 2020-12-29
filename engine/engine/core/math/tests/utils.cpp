/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/utils.hpp"

#include <random>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/core/time.hpp"
#include "gtest/gtest.h"

namespace isaac {

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

TEST(WrapPi, range) {
  std::mt19937 gen;
  std::normal_distribution<double> distribution(0.0, 100.0);
  for (int i = 0; i < 1000; i++) {
    const double angle = distribution(gen);
    EXPECT_LE(WrapPi(angle), Pi<double>);
    EXPECT_GE(WrapPi(angle), -Pi<double>);
  }
}

TEST(Utils, AngleVector) {
  EXPECT_NEAR(AngleVector(0.0).x(), 1.0, 1e-15);
  EXPECT_NEAR(AngleVector(0.0).y(), 0.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(90.0)).x(), 0.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(90.0)).y(), 1.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(180.0)).x(), -1.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(180.0)).y(), 0.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(270.0)).x(), 0.0, 1e-15);
  EXPECT_NEAR(AngleVector(DegToRad(270.0)).y(), -1.0, 1e-15);
}

TEST(Utils, Clamp) {
  EXPECT_NEAR(Clamp(0.3, -0.4, 1.3), 0.3, 1e-15);
  EXPECT_NEAR(Clamp(-0.7, -0.4, 1.3), -.4, 1e-15);
  EXPECT_NEAR(Clamp(2.3, -0.4, 1.3), 1.3, 1e-15);
  EXPECT_NEAR(Clamp01(0.3), 0.3, 1e-15);
  EXPECT_NEAR(Clamp01(-0.2), 0.0, 1e-15);
  EXPECT_NEAR(Clamp01(1.7), 1.0, 1e-15);
}

TEST(Utils, FloorToInt) {
  EXPECT_EQ(FloorToInt(-2.1), -3);
  EXPECT_EQ(FloorToInt(-2.0), -2);
  EXPECT_EQ(FloorToInt(-1.1), -2);
  EXPECT_EQ(FloorToInt(-1.0), -1);
  EXPECT_EQ(FloorToInt(-0.1), -1);
  EXPECT_EQ(FloorToInt(0.0), 0);
  EXPECT_EQ(FloorToInt(0.1), 0);
  EXPECT_EQ(FloorToInt(1.0), 1);
  EXPECT_EQ(FloorToInt(1.3), 1);
  EXPECT_EQ(FloorToInt(2.0), 2);
}

TEST(MathUtils, FastRoundDoubleToInt32) {
  EXPECT_EQ(FastRoundDoubleToInt32(3.2), 3);
  EXPECT_EQ(FastRoundDoubleToInt32(3.6), 4);
  EXPECT_EQ(FastRoundDoubleToInt32(3.5), 4);
  EXPECT_EQ(FastRoundDoubleToInt32(0.2), 0);
  EXPECT_EQ(FastRoundDoubleToInt32(-0.2), 0);
  EXPECT_EQ(FastRoundDoubleToInt32(-0.6), -1);
}

TEST(Utils, PositiveModulo) {
  EXPECT_EQ(PositiveModulo(0, 5), 0);
  EXPECT_EQ(PositiveModulo(4, 5), 4);
  EXPECT_EQ(PositiveModulo(5, 5), 0);
  EXPECT_EQ(PositiveModulo(-1, 5), 4);
  EXPECT_EQ(PositiveModulo(-5, 5), 0);
}

TEST(Utils, CeilDivision) {
  EXPECT_EQ(CeilDivision(6, 2), 3);
  EXPECT_EQ(CeilDivision(6, 4), 2);
  EXPECT_EQ(CeilDivision(5, 0), -1);
  EXPECT_EQ(CeilDivision(-6, 3), -1);
  EXPECT_EQ(CeilDivision(6, -3), -1);
  EXPECT_EQ(CeilDivision(6, -4), -1);
  EXPECT_EQ(CeilDivision(-6, -1), -1);
}

TEST(Utils, SafeClampAndCast) {
  // Base cases
  EXPECT_EQ(SafeClampAndCast(-1.0, 13, 42), 13);
  EXPECT_EQ(SafeClampAndCast(17.987, 13, 42), 17);
  EXPECT_EQ(SafeClampAndCast(100.0, 13, 42), 42);
  EXPECT_EQ(SafeClampAndCast(-1.0, -13, 42), -1);
  // Edges cases:
  EXPECT_EQ(SafeClampAndCast(12.0, 13, 42), 13);
  EXPECT_EQ(SafeClampAndCast(12.9999, 13, 42), 13);
  EXPECT_EQ(SafeClampAndCast(13.0001, 13, 42), 13);
  EXPECT_EQ(SafeClampAndCast(41.9999, 13, 42), 41);
  EXPECT_EQ(SafeClampAndCast(42.0001, 13, 42), 42);
  EXPECT_EQ(SafeClampAndCast(43.0, 13, 42), 42);
  // Overflow + special values
  for (double i = 42.0; i < 1e100; i *= 1.5) {
    EXPECT_EQ(SafeClampAndCast(i, 13, 42), 42);
    EXPECT_EQ(SafeClampAndCast(-i, 13, 42), 13);
  }
  EXPECT_EQ(SafeClampAndCast(std::numeric_limits<double>::max(), 13, 42), 42);
  EXPECT_EQ(SafeClampAndCast(std::numeric_limits<double>::infinity(), 13, 42), 42);
  EXPECT_EQ(SafeClampAndCast(std::numeric_limits<double>::quiet_NaN(), 13, 42), 42);
  EXPECT_EQ(SafeClampAndCast(std::numeric_limits<double>::lowest(), 13, 42), 13);
  EXPECT_EQ(SafeClampAndCast(-std::numeric_limits<double>::infinity(), 13, 42), 13);
}

TEST(Utils, MeanAngle) {
  // Sanity
  EXPECT_NEAR(MeanAngle(0.0, 0.0),  0.0, 0.01);
  // 0 equals 2 * pi
  EXPECT_NEAR(MeanAngle(0.0, 6.28),  0.0, 0.01);
  // Average is between 0 and pi
  EXPECT_NEAR(MeanAngle(0.0, 3.14),  1.57, 0.01);
  // Average is between 0 and pi
  EXPECT_NEAR(MeanAngle(3.14, 0.0),  1.57, 0.01);
  // Average is between 0 and -pi
  EXPECT_NEAR(MeanAngle(0.0, 3.15), -1.57, 0.01);
  // Average is between 0 and -pi, close to -pi
  EXPECT_NEAR(MeanAngle(3.14, 3.15),  -3.14, 0.01);
  // Test cases from https://stackoverflow.com/a/1159336/5692048
  EXPECT_NEAR(MeanAngle(DegToRad(0.0), DegToRad(180.0)),  DegToRad(-90.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(180.0), DegToRad(0.0)),  DegToRad(90.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(180.0), DegToRad(1.0)),  DegToRad(90.5), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(1.0), DegToRad(180.0)),  DegToRad(90.5), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(20.0), DegToRad(350.0)),  DegToRad(5.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(350.0), DegToRad(20.0)),  DegToRad(5.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(20.0), DegToRad(10.0)),  DegToRad(15.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(350.0), DegToRad(2.0)),  DegToRad(-4.0), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(359.0), DegToRad(0.0)),  DegToRad(-0.5), 1e-6);
  EXPECT_NEAR(MeanAngle(DegToRad(180.0), DegToRad(180.0)),  DegToRad(180.0), 1e-6);
}

}  // namespace isaac
