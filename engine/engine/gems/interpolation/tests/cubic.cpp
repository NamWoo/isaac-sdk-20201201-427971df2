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
#include "engine/gems/interpolation/bicubic_approximated_function.hpp"
#include "engine/gems/interpolation/cubic.hpp"
#include "engine/gems/interpolation/cubic_approximated_function.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(CubicInterpolation, edge_values) {
  std::mt19937 gen;
  std::normal_distribution<double> distribution(0.0, 100.0);
  for (int i = 0; i < 1000; i++) {
    const double x0 = distribution(gen);
    const double x1 = distribution(gen);
    const double xd0 = distribution(gen);
    const double xd1 = distribution(gen);
    EXPECT_NEAR(x0, CubicInterpolation(0.0, x0, x1, xd0, xd1), 1e-10);
    EXPECT_NEAR(x1, CubicInterpolation(1.0, x0, x1, xd0, xd1), 1e-10);
    const auto coeff = CubicCoefficients(x0, x1, xd0, xd1);
    const Vector4d zero(1.0, 0.0, 0.0, 0.0);
    const Vector4d one(1.0, 1.0, 1.0, 1.0);
    const Vector4d zerod(0.0, 1.0, 0.0, 0.0);
    const Vector4d oned(0.0, 1.0, 2.0, 3.0);
    EXPECT_NEAR(x0, zero.dot(coeff), 1e-10);
    EXPECT_NEAR(xd0, zerod.dot(coeff), 1e-10);
    EXPECT_NEAR(x1, one.dot(coeff), 1e-10);
    EXPECT_NEAR(xd1, oned.dot(coeff), 1e-10);
  }
}

TEST(CubicInterpolation, edge_values_vector) {
  std::mt19937 gen;
  std::normal_distribution<double> distribution(0.0, 100.0);
  for (int i = 0; i < 1000; i++) {
    const double x0 = distribution(gen);
    const double x1 = distribution(gen);
    const double xd0 = distribution(gen);
    const double xd1 = distribution(gen);
    const Vector4d vec(x0, x1, xd0, xd1);
    EXPECT_NEAR(x0, CubicInterpolation(0.0, vec), 1e-10);
    EXPECT_NEAR(x1, CubicInterpolation(1.0, vec), 1e-10);
    const auto coeff = CubicCoefficients(vec);
    const Vector4d zero(1.0, 0.0, 0.0, 0.0);
    const Vector4d one(1.0, 1.0, 1.0, 1.0);
    const Vector4d zerod(0.0, 1.0, 0.0, 0.0);
    const Vector4d oned(0.0, 1.0, 2.0, 3.0);
    EXPECT_NEAR(x0, zero.dot(coeff), 1e-10);
    EXPECT_NEAR(xd0, zerod.dot(coeff), 1e-10);
    EXPECT_NEAR(x1, one.dot(coeff), 1e-10);
    EXPECT_NEAR(xd1, oned.dot(coeff), 1e-10);
  }
}

TEST(BicubicInterpolation, edge_values_vector) {
  std::mt19937 gen;
  std::normal_distribution<double> distribution(0.0, 100.0);
  for (int i = 0; i < 1000; i++) {
    Matrix4d mat;
    for (int i = 0; i < 16; i++) {
      mat(i/4, i%4) = distribution(gen);
    }
    EXPECT_NEAR(mat(0, 0), BicubicInterpolation(0.0, 0.0, mat), 1e-10);
    EXPECT_NEAR(mat(1, 0), BicubicInterpolation(1.0, 0.0, mat), 1e-10);
    EXPECT_NEAR(mat(1, 1), BicubicInterpolation(1.0, 1.0, mat), 1e-10);
    EXPECT_NEAR(mat(0, 1), BicubicInterpolation(0.0, 1.0, mat), 1e-10);
    const auto coeff = BicubicCoefficients(mat);
    const Vector4d zero(1.0, 0.0, 0.0, 0.0);
    const Vector4d one(1.0, 1.0, 1.0, 1.0);
    const Vector4d zerod(0.0, 1.0, 0.0, 0.0);
    const Vector4d oned(0.0, 1.0, 2.0, 3.0);

    EXPECT_NEAR(mat(0, 0), zero.transpose() * coeff * zero, 1e-10);
    EXPECT_NEAR(mat(1, 0), one.transpose() * coeff * zero, 1e-10);
    EXPECT_NEAR(mat(1, 1), one.transpose() * coeff * one, 1e-10);
    EXPECT_NEAR(mat(0, 1), zero.transpose() * coeff * one, 1e-10);

    EXPECT_NEAR(mat(2, 0), zerod.transpose() * coeff * zero, 1e-10);
    EXPECT_NEAR(mat(3, 0), oned.transpose() * coeff * zero, 1e-10);
    EXPECT_NEAR(mat(3, 1), oned.transpose() * coeff * one, 1e-10);
    EXPECT_NEAR(mat(2, 1), zerod.transpose() * coeff * one, 1e-10);

    EXPECT_NEAR(mat(2, 2), zerod.transpose() * coeff * zerod, 1e-10);
    EXPECT_NEAR(mat(3, 2), oned.transpose() * coeff * zerod, 1e-10);
    EXPECT_NEAR(mat(3, 3), oned.transpose() * coeff * oned, 1e-10);
    EXPECT_NEAR(mat(2, 3), zerod.transpose() * coeff * oned, 1e-10);

    EXPECT_NEAR(mat(0, 2), zero.transpose() * coeff * zerod, 1e-10);
    EXPECT_NEAR(mat(1, 2), one.transpose() * coeff * zerod, 1e-10);
    EXPECT_NEAR(mat(1, 3), one.transpose() * coeff * oned, 1e-10);
    EXPECT_NEAR(mat(0, 3), zero.transpose() * coeff * oned, 1e-10);
  }
}

TEST(BicubicApproximatedFunction, row_col_increase) {
  Image1d img(10,10);
  const size_t rows = img.rows();
  const size_t cols = img.cols();
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      img(row, col) = row + col;
    }
  }
  BicubicApproximatedFunction<double, Image1d> interpolation(std::move(img), rows, cols);
  std::mt19937 gen;
  std::uniform_real_distribution<double> distribution(1.0, static_cast<double>(rows - 1));
  for (int i = 0; i < 1000; i++) {
    const double row = distribution(gen);
    const double col = distribution(gen);
    const double v = interpolation.get(row, col);
    const Vector2d grad = interpolation.gradient(row, col);
    const Matrix2d hess = interpolation.hessian(row, col);
    EXPECT_NEAR(v, row + col, 1e-10);
    EXPECT_NEAR(grad.x(), 1.0, 1e-10);
    EXPECT_NEAR(grad.y(), 1.0, 1e-10);
    EXPECT_NEAR(hess(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(hess(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(hess(1, 1), 0.0, 1e-10);
  }
}

TEST(BicubicApproximatedFunction, row_increase) {
  Image1d img(10,10);
  const size_t rows = img.rows();
  const size_t cols = img.cols();
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      img(row, col) = row;
    }
  }
  BicubicApproximatedFunction<double, Image1d> interpolation(std::move(img), rows, cols);
  std::mt19937 gen;
  std::uniform_real_distribution<double> distribution(1.0, static_cast<double>(rows - 1));
  for (int i = 0; i < 1000; i++) {
    const double row = distribution(gen);
    const double col = distribution(gen);
    const double v = interpolation.get(row, col);
    const Vector2d grad = interpolation.gradient(row, col);
    const Matrix2d hess = interpolation.hessian(row, col);
    EXPECT_NEAR(v, row, 1e-10);
    EXPECT_NEAR(grad.x(), 1.0, 1e-10);
    EXPECT_NEAR(grad.y(), 0.0, 1e-10);
    EXPECT_NEAR(hess(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(hess(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(hess(1, 1), 0.0, 1e-10);
  }
}

TEST(BicubicApproximatedFunction, col_increase) {
  Image1d img(10,10);
  const size_t rows = img.rows();
  const size_t cols = img.cols();
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      img(row, col) = col;
    }
  }
  BicubicApproximatedFunction<double, Image1d> interpolation(std::move(img), rows, cols);
  std::mt19937 gen;
  std::uniform_real_distribution<double> distribution(1.0, static_cast<double>(rows - 1));
  for (int i = 0; i < 1000; i++) {
    const double row = distribution(gen);
    const double col = distribution(gen);
    const double v = interpolation.get(row, col);
    const Vector2d grad = interpolation.gradient(row, col);
    const Matrix2d hess = interpolation.hessian(row, col);
    EXPECT_NEAR(v, col, 1e-10);
    EXPECT_NEAR(grad.x(), 0.0, 1e-10);
    EXPECT_NEAR(grad.y(), 1.0, 1e-10);
    EXPECT_NEAR(hess(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(hess(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(hess(1, 1), 0.0, 1e-10);
  }
}

TEST(BicubicApproximatedFunction, random) {
  Image<long double, 1> img(100,100);
  const size_t rows = img.rows();
  const size_t cols = img.cols();
  std::mt19937 gen;
  std::uniform_real_distribution<long double> distribution(0.0, 5.0);
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      img(row, col) = distribution(gen);
    }
  }
  BicubicApproximatedFunction<long double, Image<long double, 1>> interpolation(
      std::move(img), rows, cols);
  const long double kEps = 1e-6;
  for (int i = 0; i < 10000; i++) {
    const long double row = distribution(gen) * 15.0;
    const long double col = distribution(gen) * 15.0;
    const long double v = interpolation.get(row, col);
    const Vector2<long double> grad = interpolation.gradient(row, col);
    const Matrix2<long double> hess = interpolation.hessian(row, col);
    const long double vx = interpolation.get(row + kEps, col);
    const long double vy = interpolation.get(row, col + kEps);

    const long double vxx = interpolation.get(row + 2 * kEps, col);
    const long double vyy = interpolation.get(row, col + 2 * kEps);
    const long double vxy = interpolation.get(row + kEps, col + kEps);

    EXPECT_NEAR(grad.x(), (vx - v) / kEps, 1e-4);
    EXPECT_NEAR(grad.y(), (vy - v) / kEps, 1e-4);
    EXPECT_NEAR(hess(0, 0), (v + vxx - 2.0 * vx) / (kEps * kEps), 1e-4);
    EXPECT_NEAR(hess(0, 1), (v + vxy - vx - vy) / (kEps * kEps), 1e-4);
    EXPECT_NEAR(hess(1, 1), (v + vyy - 2.0 * vy) / (kEps * kEps), 1e-4);
  }
}

TEST(CubicApproximatedFunction, linear) {
  CubicApproximatedFunction<double,double,32> f(0.0, 1.0, [](double x) { return x; });
  EXPECT_NEAR(f(-1.7), -1.7, 1e-9);
  EXPECT_NEAR(f(0.0), 0.0, 1e-9);
  EXPECT_NEAR(f(0.2), 0.2, 1e-9);
  EXPECT_NEAR(f(0.314), 0.314, 1e-9);
  EXPECT_NEAR(f(1.0), 1.0, 1e-9);
  EXPECT_NEAR(f(1.4), 1.4, 1e-9);
}

TEST(CubicApproximatedFunction, cubic_approximated) {
  auto lamda = [](double x) { return ((0.321 * x + 1.23) * x + 0.789 ) * x + 42.0; };
  CubicApproximatedFunction<double,double,32> f(0.0, 1.0, lamda);
  EXPECT_NEAR(f(-1.7), lamda(0.0) - 1.7 * f.derivative(0.0), 1e-5);
  EXPECT_NEAR(f(0.0), lamda(0.0), 1e-5);
  EXPECT_NEAR(f(0.2), lamda(0.2), 1e-5);
  EXPECT_NEAR(f(0.314), lamda(0.314), 1e-5);
  EXPECT_NEAR(f(1.0), lamda(1.0), 1e-5);
  EXPECT_NEAR(f(1.4), lamda(1.0) + 0.4 * f.derivative(1.0), 1e-5);
}

TEST(CubicApproximatedFunction, cubic) {
  auto lamda = [](double x) { return ((0.321 * x + 1.23) * x + 0.789 ) * x + 42.0; };
  auto grad = [](double x) { return ((3.0 * 0.321 * x + 2.0 * 1.23) * x + 0.789 ); };
  CubicApproximatedFunction<double,double,32> f(0.0, 1.0, lamda, grad);
  EXPECT_NEAR(f(-1.7), lamda(0.0) - 1.7 * f.derivative(0.0), 1e-9);
  EXPECT_NEAR(f(0.0), lamda(0.0), 1e-9);
  EXPECT_NEAR(f(0.2), lamda(0.2), 1e-9);
  EXPECT_NEAR(f(0.314), lamda(0.314), 1e-9);
  EXPECT_NEAR(f(1.0), lamda(1.0), 1e-9);
  EXPECT_NEAR(f(1.4), lamda(1.0) + 0.4 * f.derivative(1.0), 1e-9);

  EXPECT_NEAR(f.derivative(-1.7), grad(0.0), 1e-9);
  EXPECT_NEAR(f.derivative(0.0), grad(0.0), 1e-9);
  EXPECT_NEAR(f.derivative(0.2), grad(0.2), 1e-9);
  EXPECT_NEAR(f.derivative(0.314), grad(0.314), 1e-9);
  EXPECT_NEAR(f.derivative(1.0), grad(1.0), 1e-9);
  EXPECT_NEAR(f.derivative(1.4), grad(1.0), 1e-9);
}

TEST(CubicApproximatedFunction, n_1) {
  auto lamda = [](double x) { return 3.0 * x; };
  CubicApproximatedFunction<double,double,1> f(0.0, 1.0, lamda);
  EXPECT_NEAR(f(-1.7), lamda(-1.7), 1e-9);
  EXPECT_NEAR(f(0.0), lamda(0.0), 1e-9);
  EXPECT_NEAR(f(0.2), lamda(0.2), 1e-9);
  EXPECT_NEAR(f(0.314), lamda(0.314), 1e-9);
  EXPECT_NEAR(f(1.0), lamda(1.0), 1e-9);
  EXPECT_NEAR(f(1.4), lamda(1.4), 1e-9);
}

}  // namespace isaac
