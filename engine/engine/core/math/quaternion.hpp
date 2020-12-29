/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// Returns the coefficients of the quaternion in the order (w, x, y, z).
template <typename K>
Vector4<K> QuaternionCoefficients(const Quaternion<K>& q) {
  return Vector4<K>{q.w(), q.x(), q.y(), q.z()};
}

// The exponential map for unit quaternions. Note that this function is defined without a commonly
// appearing factor of 1/2. Thus exp(a) gives a unit quaternion which rotates around an axis a by
// an angle of 2*|a|. The factor 1/2 is used in the context of spatial rotations, but is not
// inhert to quaternions.
template <typename K>
Quaternion<K> QuaternionExp(const Vector3<K>& a) {
  const K n = a.norm();
  const K cos_n = std::cos(n);
  const K sin_n = std::sin(n);
  K factor1;
  if (n < K(10) * MachineEpsilon<K>) {
    // Use series expansion to avoid singularity.
    //   sin(n)/n -> 1 - n^2/6
    // However as we are already dealing with a very small n we can just approximate with 1.
    factor1 = K(1);
  } else {
    factor1 = sin_n / n;
  }
  return Quaternion<K>(cos_n, factor1*a.x(), factor1*a.y(), factor1*a.z());
}

// The logarithm map for quaternions which is essentially the opposite of the exponential map. If
// a non-unit quaternion is passed only the imaginary part is returned which represents the
// logaritm of the corresponding normalized quaternion. In case the zero quaternion is passed the
// zero vector is returned.
template <typename K>
Vector3<K> QuaternionLog(const Quaternion<K>& q) {
  const Vector3<K> axes = QuaternionCoefficients(q).template tail<3>();
  const K sin_x = axes.norm();
  K factor;
  if (sin_x < K(10) * MachineEpsilon<K>) {
    // We have cos(x) = sqrt(1 - sin(x)^2).
    // Thus compute the approximation for:
    //    atan2(a*sqrt(1 - z^2), a*z) / z
    // The series expansion is:
    //    1 + z^2/6
    // which is 1 as z^2 < MachineEpsilon
    // This is strictly speaking only true for a > 0, but we decide to return 0 vector for 0 input
    // quaternion.
    factor = K(1);
  } else {
    factor = std::atan2(sin_x, q.w()) / sin_x;
  }
  return factor * axes;
}

// Computes f1 = sin(x)/x and f2 = cos(x)/x^2 - sin(x)/x^3
template <typename K>
void QuaternionExpJacobianFactorsExact(K x, K& factor1, K& factor2) {
  factor1 = std::sin(x) / x;
  factor2 = (std::cos(x) - factor1)/(x*x);
}

// Like QuaternionExpJacobianFactors but uses a second order Taylor expansion around x=0.
template <typename K>
void QuaternionExpJacobianFactorsApproximation(K x, K& factor1, K& factor2) {
  const K x2 = x*x;
  factor1 = K(1) - x2/K(6);
  factor2 = x2/K(30) - K(1)/K(3);
}

// Like QuaternionExpJacobianFactorsExact but chooses approximation based on magnitude of x.
template <typename K>
void QuaternionExpJacobianFactors(K x, K& factor1, K& factor2) {
  if (x < std::pow(MachineEpsilon<K>, K(1)/K(3))) {
    QuaternionExpJacobianFactorsApproximation(x, factor1, factor2);
  } else {
    QuaternionExpJacobianFactorsExact(x, factor1, factor2);
  }
}

// The gradient of the exponential map with respect to its coefficients
template <typename K>
Matrix43<K> QuaternionExpJacobian(const Vector3<K>& a) {
  K factor1, factor2;
  QuaternionExpJacobianFactors(a.norm(), factor1, factor2);
  Matrix43<K> result;
  result.template block<1, 3>(0, 0) = -factor1 * a;
  result.template block<3, 3>(1, 0) = (factor2 * a) * a.transpose();
  result(1, 0) += factor1;
  result(2, 1) += factor1;
  result(3, 2) += factor1;
  return result;
}

// Computes a matrix which creates multiplication of quaternions
// In particular with f(a, b) := a * b (multiplication of quaternions) this function returns
// the matrix M such that f(a, b) = M(a) * b.
// This matrix is also the Jacobian of quaternion multiplication with respect to the second
// factor, i.e. (J_x f)(a, x) = M(a).
template <typename K>
Matrix4<K> QuaternionProductMatrixLeft(const Quaternion<K>& q) {
  const K w = q.w();
  const K x = q.x();
  const K y = q.y();
  const K z = q.z();
  Matrix4<K> result;
  result <<
       w, -x, -y, -z,
       x,  w, -z,  y,
       y,  z,  w, -x,
       z, -y,  x,  w;
  return result;
}

// Computes a matrix which creates multiplication of quaternions
// In particular with f(a, b) := a * b (multiplication of quaternions) this function returns
// the matrix M such that f(a, b) = M(b) * a.
// This matrix is also the Jacobian of quaternion multiplication with respect to the first
// factor, i.e. (J_x f)(x, b) = M(b).
template <typename K>
Matrix4<K> QuaternionProductMatrixRight(const Quaternion<K>& q) {
  const K w = q.w();
  const K x = q.x();
  const K y = q.y();
  const K z = q.z();
  Matrix4<K> result;
  result <<
       w, -x, -y, -z,
       x,  w,  z, -y,
       y, -z,  w,  x,
       z,  y, -x,  w;
  return result;
}

}  // namespace isaac
