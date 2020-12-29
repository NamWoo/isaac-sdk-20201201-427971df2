/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <utility>

#include "engine/core/math/types.hpp"
#include "engine/gems/interpolation/cubic.hpp"
#include "engine/gems/interpolation/utils.hpp"

namespace isaac {

// Helper class to approximate a C-infinity function with a list of precomputed function values.
//  K: type of the domain (function input)
//  T: type of the co-domain (function output)
//  N: number of keypoints to use
template <typename K, typename T, size_t N>
class CubicApproximatedFunction {
 public:
  static_assert(N >= 1, "N must be greater than 1");
  // Creates an empty function cache with undefined values
  CubicApproximatedFunction() : x_min_{K(0)}, x_max_{K(1)} {}

  // Creates a function cache for the range [xmin,xmax] for the given function f: K -> T
  template <typename F>
  CubicApproximatedFunction(K x_min, K x_max, F f) { initialize(x_min, x_max, f); }

  // Same as above but also use a defined gradient function.
  template <typename F, typename G>
  CubicApproximatedFunction(K x_min, K x_max, F f, G grad) { initialize(x_min, x_max, f, grad); }

  template <typename F>
  void initialize(K x_min, K x_max, F f);

  template <typename F, typename G>
  void initialize(K x_min, K x_max, F f, G grad);

  // Gets the approximate function value at the given value. This will use cubic interpolation
  // between keypoints, and linear continuation outside of the domain range.
  T operator()(K x) const;

  // Returns the first derivative of the interpolated value at a given position.
  T derivative(K x) const;

 private:
  K x_min_, x_max_;
  K step_size_, step_size_inv_;
  std::array<Vector4<T>, N> cache_;
};

// -------------------------------------------------------------------------------------------------

// Implementation of the CubicApproximatedFunction
template <typename K, typename T, size_t N>
template <typename F, typename G>
void CubicApproximatedFunction<K, T, N>::initialize(K x_min, K x_max, F f, G grad) {
  ASSERT(x_min < x_max, "Range must not be empty");
  x_min_ = x_min;
  x_max_ = x_max;
  step_size_ = (x_max - x_min) / N;
  step_size_inv_ = K(1) / step_size_;
  if (N == 1) {
    cache_[0] = CubicCoefficients(f(x_min), f(x_max), grad(x_min), grad(x_max));
    return;
  }
  T prev_value = f(x_min_);
  T prev_grad = grad(x_min_) * step_size_;
  for (size_t i = 0; i < N; i++) {
    const K x = RescaleFromInteger(i + 1, N, x_min_, x_max_);
    const T curr_value = f(x);
    const T curr_grad = grad(x) * step_size_;
    cache_[i] = CubicCoefficients(prev_value, curr_value, prev_grad, curr_grad);
    prev_value = std::move(curr_value);
    prev_grad = std::move(curr_grad);
  }
}

template <typename K, typename T, size_t N>
template <typename F>
void CubicApproximatedFunction<K, T, N>::initialize(K x_min, K x_max, F f) {
  ASSERT(x_min < x_max, "Range must not be empty");
  x_min_ = x_min;
  x_max_ = x_max;
  step_size_ = (x_max - x_min) / N;
  step_size_inv_ = K(1) / step_size_;
  if (N == 1) {
    // Perform linear interpolation
    cache_[0](0) = f(x_min_);
    cache_[0](1) = f(x_max_) - cache_[0](0);
    cache_[0](2) = cache_[0](2) = K(0)*T();
    return;
  }
  T values[4];
  values[0] = f(RescaleFromInteger(0ul, N, x_min_, x_max_));
  values[1] = f(RescaleFromInteger(1ul, N, x_min_, x_max_));
  values[2] = f(RescaleFromInteger(2ul, N, x_min_, x_max_));
  // First and last value need special case as the gradient is not available
  cache_[0] = CubicCoefficients(values[0], values[1],
                                values[1] - values[0], (values[2] - values[0]) / K(2));
  for (size_t i=3; i <= N; i++) {
    values[i % 4] = f(RescaleFromInteger(i, N, x_min_, x_max_));
    cache_[i - 2] = CubicCoefficients(values[(i-2)%4], values[(i-1)%4],
                                      (values[(i-1)%4] - values[(i-3)%4]) / K(2),
                                      (values[i%4] - values[(i-2)%4]) / K(2));
  }
  cache_.back() = CubicCoefficients(values[(N-1)%4], values[N%4],
                                    (values[N%4] - values[(N-2)%4]) / K(2),
                                    (values[N%4] - values[(N-1)%4]));
}

template <typename K, typename T, size_t N>
T CubicApproximatedFunction<K, T, N>::operator()(K x) const {
  K fractional;
  const int i = RescaleToInteger(x, x_min_, x_max_, static_cast<int>(N), fractional);
  if (i < 0) {
    fractional += static_cast<K>(i);
    return cache_[0](0) + fractional * cache_[0](1);
  }
  if (static_cast<size_t>(i) >= cache_.size()) {
    fractional += static_cast<K>(i - cache_.size());
    return CubicInterpolationEvaluation(K(1), cache_.back()) +
           fractional * CubicInterpolationGradient(K(1), cache_.back());
  }
  return CubicInterpolationEvaluation(fractional, cache_[i]);
}

template <typename K, typename T, size_t N>
T CubicApproximatedFunction<K, T, N>::derivative(K x) const {
  K fractional;
  const int i = RescaleToInteger(x, x_min_, x_max_, static_cast<int>(N), fractional);
  T diff;
  if (i < 0) {
    diff = cache_[0](1);
  } else if (static_cast<size_t>(i) >= cache_.size()) {
    diff = CubicInterpolationGradient(K(1), cache_.back());
  } else {
    diff = CubicInterpolationGradient(fractional, cache_[i]);
  }
  return diff * step_size_inv_;
}

}  // namespace isaac
