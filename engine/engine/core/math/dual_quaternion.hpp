/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/quaternion.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// Class for dual quaternion (https://en.wikipedia.org/wiki/Dual_quaternion)
// Dual quaternion are used to represent a 3D transformation (similar to Pose3) but also provide
// easier mathematic operations on them.
template <typename K>
class DualQuaternion {
 public:
  using Scalar = K;
  DualQuaternion() = default;

  // Creates a dual quaternion from scalars
  DualQuaternion(K rw, K rx, K ry, K rz, K dw, K dx, K dy, K dz)
      : real_(rw, rx, ry, rz), dual_(dw, dx, dy, dz) {}

  // Creates a dual quaterion from a real part and a dual part.
  DualQuaternion(const Quaternion<K>& real, const Quaternion<K>& dual) : real_(real), dual_(dual) {}

  // Creates a dual quaternion from a list of coefficients. The order is:
  // (rq, rx, ry, rz, dq, dx, dy, dz)
  static DualQuaternion FromCoefficients(const Vector8<K>& coeffs) {
    return DualQuaternion(Quaternion<K>(coeffs[0], coeffs[1], coeffs[2], coeffs[3]),
                          Quaternion<K>(coeffs[4], coeffs[5], coeffs[6], coeffs[7]));
  }

  // Creates a dual quaternion from a 3D pose
  static DualQuaternion FromQuaternions(const Quaternion<K>& real, const Quaternion<K>& dual) {
    return DualQuaternion(real, dual);
  }

  // Creates a dual quaternion from a 3D pose
  static DualQuaternion FromPose3(const Pose3<K>& pose) {
    return DualQuaternion(pose.rotation.quaternion(),
                          Quaternion<K>(K(0), pose.translation.x() / K(2),
                                        pose.translation.y() / K(2), pose.translation.z() / K(2)) *
                          pose.rotation.quaternion());
  }

  // Creates the identity dual quaternion
  static DualQuaternion Identity() {
    return DualQuaternion(Quaternion<K>::Identity(), Quaternion<K>(K(0), K(0), K(0), K(0)));
  }

  // Returns the inverse dual quaternion.
  DualQuaternion inverse() const {
    const Quaternion<K> real_inverse = real_.inverse();
    const Quaternion<K> tmp = real_inverse * dual_ * real_inverse;
    return DualQuaternion(real_inverse, -tmp);
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  DualQuaternion<S> cast() const {
    return DualQuaternion<S>(real_.template cast<S>(), dual_.template cast<S>());
  }
  template <typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const DualQuaternion& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Returns the Pose3 associated to this dual quaternion.
  Pose3<K> toPose3() const {
    const K norm = real_.norm();
    ASSERT(!IsAlmostZero(norm), "Zero dual quaternion can not be converted to Pose3");
    const Quaternion<K> real_0(real_.coeffs() / norm);
    const Quaternion<K> vector = dual_ * real_0.conjugate();
    return Pose3<K>{SO3<K>::FromNormalizedQuaternion(real_0), (K(2) / norm) * vector.vec()};
  }

  // Composition of two dual quaternions
  friend DualQuaternion operator*(const DualQuaternion& lhs, const DualQuaternion& rhs) {
    return DualQuaternion(lhs.real_ * rhs.real_, lhs.real_ * rhs.dual_ + lhs.dual_ * rhs.real_);
  }

  // Rotates a vector 3D by the given rotation
  friend Vector3<K> operator*(const DualQuaternion& q, const Vector3<K>& v) {
    // TODO(rjanardhana): faster implementation
    DualQuaternion q_conj(q.real_.conjugate(), -q.dual_.conjugate());
    return (q * DualQuaternion(K(1), K(0), K(0), K(0), K(0), v.x(), v.y(), v.z()) * q_conj)
            .coeffs().tail(3);
  }

  // Addition of two dual quaternions
  friend DualQuaternion operator+(const DualQuaternion& lhs, const DualQuaternion& rhs) {
    return DualQuaternion(lhs.real_ + rhs.real_, lhs.dual_ + rhs.dual_);
  }
  // dual quaternion division by a scalar
  friend DualQuaternion operator/(const DualQuaternion& lhs, const K rhs) {
    return DualQuaternion::FromCoefficients(lhs.coeffs() / rhs);
  }
  // dual quaternion multiplication by a scalar
  friend DualQuaternion operator*(const K& lhs, const DualQuaternion& rhs) {
    return DualQuaternion::FromCoefficients(rhs.coeffs() * lhs);
  }
  // dual quaternion multiplication by a scalar
  friend DualQuaternion operator*(const DualQuaternion& lhs, const K rhs) {
    return DualQuaternion::FromCoefficients(lhs.coeffs() * rhs);
  }
  // Substraction of two dual quaternions
  friend DualQuaternion operator-(const DualQuaternion& lhs, const DualQuaternion& rhs) {
    return DualQuaternion(lhs.real_ - rhs.real_, lhs.dual_ - rhs.dual_);
  }

  // Returns the real part of this dual quaternion
  const Quaternion<K>& real() const { return real_; }

  // Returns the dual part of this dual quaternion
  const Quaternion<K>& dual() const { return dual_; }

  // Returns the dual part of this dual quaternion
  const Vector8<K> coeffs() const {
    Vector8<K> ret;
    ret[0] = real_.w();
    ret[1] = real_.x();
    ret[2] = real_.y();
    ret[3] = real_.z();
    ret[4] = dual_.w();
    ret[5] = dual_.x();
    ret[6] = dual_.y();
    ret[7] = dual_.z();
    return ret;
  }

  // Computes a matrix which creates multiplication of dual quaternions
  // In particular with f(a, b) := a * b (multiplication of dual quaternions) this function returns
  // the matrix M such that f(a, b) = M(a) * b.
  // This matrix is also the Jacobian of dual quaternion multiplication with respect to the second
  // factor, i.e. (J_x f)(a, x) = M(a).
  Matrix8<K> productMatrixLeft() const;

  // Computes a matrix which creates multiplication of dual quaternions
  // In particular with f(a, b) := a * b (multiplication of dual quaternions) this function returns
  // the matrix M such that f(a, b) = M(b) * a.
  // This matrix is also the Jacobian of dual quaternion multiplication with respect to the first
  // factor, i.e. (J_x f)(x, b) = M(b).
  Matrix8<K> productMatrixRight() const;

  // Computes a matrix which is jacobian of dual quaternion with vector
  Matrix<K, 8, 3> vectorMultiplicationJacobian(const Vector3<K>& v) const;

  // Normalizes the dual quaternion with the norm of the rotation coefficients
  const DualQuaternion<K> normalizeByRotation() const {
    const K norm = real_.norm();
    ASSERT(!IsAlmostZero(norm), "Zero dual quaternion can not be normalized");
    const Quaternion<K> real_0(real_.coeffs() / norm);
    const Quaternion<K> dual_0(dual_.coeffs() / norm);
    return DualQuaternion::FromQuaternions(real_0, dual_0);
  }

  // The exponential map for dual quaternions. It maps an element from the 6D tangent space to a
  // unit dual quaternion. For a dual quaternion h = q + e*p the exponential map is defined as:
  //   exp(h) = exp(q + e*p) = exp(q) + e*p*exp(q)
  // where exp(q) is the exponential function for quaternions.
  static DualQuaternion<K> Exp(const Vector6<K>& v) {
    const Quaternion<K> real = QuaternionExp(v.template head<3>().eval());
    const Quaternion<K> p(K(0), v[3], v[4], v[5]);
    return DualQuaternion<K>(real, p * real);
  }

  // The Jacobian of the exponential map with respect to its coefficients. The exponential map is
  // a function exp: R^6 -> R^8, thus the Jacobian is a matrix in R^6x8.
  // The gradient of J_h exp(h) can be computed as follows:
  // Let h = q + e*p.
  // Let's first observe that:
  //   p*exp(q) = M_l(p) * exp(q)
  //   p*exp(q) = M_r(exp(q)) * V(p), with V(x,y,z) := (0, x, y, z)
  // where M_l and M_r are 4x4 matrices which create quaternion multiplication.
  // Thus:
  //   J_q p*exp(q) = J_q M_l(p) * exp(q) = JM_l(p) * Jexp(q)
  //   J_p p*exp(q) = J_p M_r(exp(q)) * V(p) = JM_r(exp(q)) * JV(p) with JV(p) = (0, I_3)
  // It follows that J_h exp(h) is equal to the following 6x8 matrix:
  //   (           Jexp(q) ,                      0  )
  //   ( JM_l(p) * Jexp(q) , JM_r(exp(q)) * (0, I_3) )
  static Matrix<K, 8, 6> ExpJacobian(const Vector6<K>& v) {
    const Quaterniond vp(K(0), v[3], v[4], v[5]);
    Matrix<K, 8, 6> result;
    result.template block<4, 3>(0, 0) = QuaternionExpJacobian(v.template head<3>().eval());
    result.template block<4, 3>(4, 0) =
        QuaternionProductMatrixLeft(vp) * result.template block<4, 3>(0, 0);
    result.template block<4, 3>(0, 3) = Matrix43<K>::Zero();
    result.template block<4, 3>(4, 3) = QuaternionProductMatrixRight(
          QuaternionExp(v.template head<3>().eval())).template block<4, 3>(0, 1);
    return result;
  }

 public:
  Quaternion<K> real_;
  Quaternion<K> dual_;
};

using DualQuaternionD = DualQuaternion<double>;
using DualQuaternionF = DualQuaternion<float>;

}  // namespace isaac
