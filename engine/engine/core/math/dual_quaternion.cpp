/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "dual_quaternion.hpp"

#include "engine/core/math/quaternion.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

template <typename K>
Matrix8<K> DualQuaternion<K>::productMatrixLeft() const {
  Matrix8<K> result;
  result.template block<4, 4>(0, 0) = QuaternionProductMatrixLeft(real_);
  result.template block<4, 4>(0, 4).setZero();
  result.template block<4, 4>(4, 0) = QuaternionProductMatrixLeft(dual_);
  result.template block<4, 4>(4, 4) = result.template block<4, 4>(0, 0);
  return result;
}

template <typename K>
Matrix8<K> DualQuaternion<K>::productMatrixRight() const {
  Matrix8<K> result;
  result.template block<4, 4>(0, 0) = QuaternionProductMatrixRight(real_);
  result.template block<4, 4>(0, 4).setZero();
  result.template block<4, 4>(4, 0) = QuaternionProductMatrixRight(dual_);
  result.template block<4, 4>(4, 4) = result.template block<4, 4>(0, 0);
  return result;
}

template Matrix8<double> DualQuaternion<double>::productMatrixLeft() const;
template Matrix8<float> DualQuaternion<float>::productMatrixLeft() const;
template Matrix8<double> DualQuaternion<double>::productMatrixRight() const;
template Matrix8<float> DualQuaternion<float>::productMatrixRight() const;

template <typename K>
Matrix<K, 8, 3> DualQuaternion<K>::vectorMultiplicationJacobian(
    const Vector3<K>& v) const {
  const Vector8<K> c = coeffs();
  Matrix<K, 8, 3> result;
  result <<
      K(2)  * (c[5] + c[0] * v[0] - c[3] * v[1] + c[2] * v[2]),
      K(2)  * (c[6] + c[3] * v[0] + c[0] * v[1] - c[1] * v[2]),
      K(2)  * (c[7] - c[2] * v[0] + c[1] * v[1] + c[0] * v[2]),

      K(-2) * (c[4] - c[1] * v[0] - c[2] * v[1] - c[3] * v[2]),
      K(-2) * (c[7] - c[2] * v[0] + c[1] * v[1] + c[0] * v[2]),
      K(2)  * (c[6] + c[3] * v[0] + c[0] * v[1] - c[1] * v[2]),

      K(2)  * (c[7] - c[2] * v[0] + c[1] * v[1] + c[0] * v[2]),
      K(-2) * (c[4] - c[1] * v[0] - c[2] * v[1] - c[3] * v[2]),
      K(-2) * (c[5] + c[0] * v[0] - c[3] * v[1] + c[2] * v[2]),

      K(-2) * (c[6] + c[3] * v[0] + c[0] * v[1] - c[1] * v[2]),
      K(2)  * (c[5] + c[0] * v[0] - c[3] * v[1] + c[2] * v[2]),
      K(-2) * (c[4] - c[1] * v[0] - c[2] * v[1] - c[3] * v[2]),

      K(-2) * c[1],
      K(-2) * c[2],
      K(-2) * c[3],

      K(2)  * c[0],
      K(2)  * c[3],
      K(-2) * c[2],

      K(-2) * c[3],
      K(2)  * c[0],
      K(2)  * c[1],

      K(2)  * c[2],
      K(-2) * c[1],
      K(2)  * c[0];
  return result;
}

template
Matrix<double, 8, 3> DualQuaternion<double>::vectorMultiplicationJacobian(
    const Vector3<double>& v) const;
template
Matrix<float, 8, 3> DualQuaternion<float>::vectorMultiplicationJacobian(
    const Vector3<float>& v) const;

}  // namespace isaac
