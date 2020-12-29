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

#include "engine/core/assert.hpp"
#include "engine/gems/interpolation/linear.hpp"
#include "engine/gems/interpolation/utils.hpp"

namespace isaac {

// Helper class to approximate a function with a list of precomputed function values
//  K: type of the domain (function input)
//  T: type of the co-domain (function output)
//  N: number of keypoints to use
template <typename K, typename T, size_t N>
class LinearApproximatedFunction {
 public:
  // Creates an empty function cache with undefined values
  LinearApproximatedFunction() : x_min_{K(0)}, x_max_{K(1)} {}
  // Creates a function cache for the range [xmin,xmax] for the given function f: K -> T
  template <typename F>
  LinearApproximatedFunction(K x_min, K x_max, F f) {
    initialize(x_min, x_max, f);
  }
  template <typename F>
  void initialize(K x_min, K x_max, F f);

  // Gets the approximate function value at the given value. This will use linear interpolation
  // between keypoints, and constant continuation outside of the domain.
  T operator()(K x) const;

 private:
  K x_min_, x_max_;
  std::array<T, N> cache_;
};

// -------------------------------------------------------------------------------------------------

// Implementation of the LinearApproximatedFunction
template <typename K, typename T, size_t N>
template <typename F>
void LinearApproximatedFunction<K, T, N>::initialize(K x_min, K x_max, F f) {
  ASSERT(x_min < x_max, "Range must not be empty");
  x_min_ = x_min;
  x_max_ = x_max;
  for (size_t i = 0; i < N; i++) {
    cache_[i] = f(RescaleFromInteger(i, N - 1, x_min_, x_max_));
  }
}

template <typename K, typename T, size_t N>
T LinearApproximatedFunction<K, T, N>::operator()(K x) const {
  if (cache_.empty()) {
    return K(0);
  }
  if (cache_.size() == 1) {
    return cache_.front();
  }
  K fractional;
  const int i = RescaleToInteger(x, x_min_, x_max_, static_cast<int>(N) - 1, fractional);
  if (i < 0) {
    return cache_.front();
  }
  if (static_cast<size_t>(i + 1) >= cache_.size()) {
    return cache_.back();
  }
  return Interpolate(fractional, cache_[i], cache_[i + 1]);
}

}  // namespace isaac
