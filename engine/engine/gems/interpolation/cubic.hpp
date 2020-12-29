/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/types.hpp"

namespace isaac {

// Returns the coefficients of the 3rd degree polynom P(x) = a * x^3 + b * x^2 + c * x + d such as:
// P(0) = x0
// P(1) = x1
// P'(0) = xd0
// P'(1) = xd1
// Returns {d, c, b, a}
template<typename K>
inline Vector4<K> CubicCoefficients(K x0, K x1, K xd0, K xd1) {
  return Vector4<K>(
      x0,
      xd0,
      K(-3) * x0 + K(3)  * x1 - K(2) * xd0 - xd1,
      K(2)  * x0 + K(-2) * x1 +        xd0 + xd1);
}

// Same as above but take the constraints as a vector
template<typename K>
inline Vector4<K> CubicCoefficients(const Vector4<K>& constraints) {
  return CubicCoefficients(constraints[0], constraints[1], constraints[2], constraints[3]);
}

// Computes the value P(x) such as P is a 3rd degree polynom that follow the constraints as describe
// above.
template<typename K>
inline K CubicInterpolation(K x, K x0, K x1, K xd0, K xd1) {
  return Vector4<K>(K(1), x, x*x, x*x*x).dot(CubicCoefficients(x0, x1, xd0, xd1));
}

// Same as above but take the constraints as a vector
template<typename K>
inline K CubicInterpolation(K x, const Vector4<K>& constraints) {
  return CubicInterpolation(x, constraints[0], constraints[1], constraints[2], constraints[3]);
}

template<typename K>
K CubicInterpolation(const Vector4<K>& constraints, K x) {
  return Vector4<K>(K(1), x, x*x, x*x*x).dot(CubicCoefficients(constraints));
}

// Evaluates the function at a given position given the coefficients.
template<typename K>
K CubicInterpolationEvaluation(K x, const Vector4<K>& coefficients) {
  const K x2 = x * x;
  const K x3 = x2 * x;
  return Vector4<K>(K(1), x, x2, x3).dot(coefficients);
}

// Evaluates the graduebt at a given position given the coefficients.
template<typename K>
K CubicInterpolationGradient(K x, const Vector4<K>& coefficients) {
  return Vector3<K>(K(1), K(2) * x, K(3) * x * x).dot(coefficients.template tail<3>());
}

// https://en.wikipedia.org/wiki/Bicubic_interpolation
// Returns the coefficients m of P(X, Y) = sum(m(i, j) * X^i * Y^j, i = 0 .. 3, j = 0 .. 3)
// constraints =
// |  P(0, 0)    P(0, 1)    Py(0, 0)    Py(0, 1)   |
// |  P(1, 0)    P(1, 1)    Py(1, 0)    Py(1, 1)   |
// |  Px(0, 0)   Px(0, 1)   Pxy(0, 0)   Pxy(0, 1)  |
// |  Px(1, 0)   Px(1, 1)   Pxy(1, 0)   Pxy(1, 1)  |
// Where Px means dP/dx, Pxy = d²P/dxdy
template<typename K>
Matrix4<K> BicubicCoefficients(const Matrix4<K>& constraints) {
  constexpr K kMatrix[16] = {
    K(1),  K(0),  K(0),  K(0),
    K(0),  K(0),  K(1),  K(0),
    K(-3), K(3),  K(-2), K(-1),
    K(2),  K(-2), K(1),  K(1),
  };
  Eigen::Map<const Matrix4<K>> mat(kMatrix);
  return mat.transpose() * constraints * mat;
}

// Computes P(x, y) as define above.
template<typename K>
K BicubicInterpolation(K x, K y, const Matrix4<K>& constraints) {
  return Vector4<K>(K(1), x, x*x, x*x*x).transpose() *
         BicubicCoefficients(constraints) *
         Vector4<K>(K(1), y, y*y, y*y*y);
}

// Computes P(x, y) as define above.
template<typename K>
K BicubicInterpolationEvaluation(K x, K y, const Matrix4<K>& coefficients) {
  const K x2 = x * x;
  const K x3 = x2 * x;
  const K y2 = y * y;
  const K y3 = y2 * y;
  return Vector4<K>(K(1), x, x2, x3).transpose() * coefficients * Vector4<K>(K(1), y, y2, y3);
}

// Computes the gradient at a given position
template<typename K>
Vector2<K> BicubicInterpolationGradient(K x, K y, const Matrix4<K>& coefficients) {
  const K x2 = x * x;
  const K x3 = x2 * x;
  const K y2 = y * y;
  const K y3 = y2 * y;
  const K gx = Vector3<K>(K(1), K(2) * x, K(3) * x2).transpose() *
               coefficients. template bottomRightCorner<3, 4>() *
               Vector4<K>(K(1), y, y2, y3);
  const K gy = Vector<K, 4>(K(1), x, x2, x3).transpose() *
               coefficients.template bottomRightCorner<4, 3>() *
               Vector<K, 3>(K(1), K(2) * y, K(3) * y2);
  return Vector2<K>{gx, gy};
}

// Computes the hessian at a given position
template<typename K>
Matrix2<K> BicubicInterpolationHessian(K x, K y, const Matrix4<K>& coefficients) {
  const K x2 = x * x;
  const K x3 = x2 * x;
  const K y2 = y * y;
  const K y3 = y2 * y;
  Matrix2<K> H;
  H(0, 0) = Vector<K, 2>(K(2), K(6) * x).transpose() *
            coefficients.template bottomRightCorner<2, 4>() *
            Vector<K, 4>(K(1), y, y2, y3);
  H(0, 1) = Vector<K, 3>(K(1), K(2) * x, K(3) * x2).transpose() *
            coefficients.template bottomRightCorner<3, 3>() *
            Vector<K, 3>(K(1), K(2) * y, K(3) * y2);
  H(1, 0) = H(0, 1);
  H(1, 1) = Vector<K, 4>(K(1), x, x2, x3).transpose() *
            coefficients.template bottomRightCorner<4, 2>() *
            Vector<K, 2>(K(2), K(6) * y);
  return H;
}

}  // namespace isaac
