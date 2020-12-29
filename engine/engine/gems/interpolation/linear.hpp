/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

namespace isaac {

// Returns a value between a and b at the relative position q.
//
// Note: This function will also work for q outside of the unit interval.
template<typename K, typename T>
T Interpolate(K q, T a, T b) {
  return a + q * (b - a);
}

// Coefficients for bi-linear interpolation
template <typename K>
void BilinearInterpolationCoefficients(K pa, K pb, K& p_00, K& p_01, K& p_10, K& p_11) {
  p_11 = pa * pb;
  p_01 = pb - p_11;
  p_10 = pa - p_11;
  p_00 = K(1) - pa - pb + p_11;
}

// Bi-linear interpolation for v_ij by pa along the i-axis and by pb along the j-axis
template <typename K, typename T>
T BilinearInterpolation(K pa, K pb, const T& v_00, const T& v_01, const T& v_10, const T& v_11) {
  K p_00, p_01, p_10, p_11;
  BilinearInterpolationCoefficients(pa, pb, p_00, p_01, p_10, p_11);
  return p_00*v_00 + p_01*v_01 + p_10*v_10 + p_11*v_11;
}

}  // namespace isaac
