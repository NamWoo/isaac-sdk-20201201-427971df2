/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/gems/interpolation/linear.hpp"

namespace isaac {

// Returns x rescaled to run from 0 to 1 over the range xmin to xmax
//
// Note: This function will also work if x is is not in the interval [xmin,xmax] or if xmin > xmax.
template<typename K>
K Rescale(K x, K xmin, K xmax) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  ASSERT(!IsAlmostZero(xmin - xmax), "xmin and xmax must not be equal");
  return (x - xmin) / (xmax - xmin);
}

// Returns x rescaled to run from ymin to ymax over the range xmin to max.
//
// Note: This function will also work if x is is not in the interval [xmin,xmax], if xmin > xmax,
//       and if ymin > ymax or ymin == ymax.
template<typename K>
K Rescale(K x, K xmin, K xmax, K ymin, K ymax) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  return Interpolate(Rescale(x, xmin, xmax), ymin, ymax);
}

// Computes metric position in a discrete grid
//
// Let's say a 1-dimensional grid with n cells spawns a real interval [xmin, xmax]. This function
// computes the real position for a given grid cell. We assume that the grid cells anchor point is
// on the left edge of the cell.
//
// i=   0    1    2    3     n-2  n-1  n
//      [----|----|----| ... |----|----]
// x=   xmin                           xmax
//
// Note: This function will also work if xmin > xmax, for negative i, or negative n.
template<typename K, typename I>
K RescaleFromInteger(I i, I n, K xmin, K xmax) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  ASSERT(n != 0, "Number of cells must not be 0");
  return Interpolate(static_cast<K>(i) / static_cast<K>(n), xmin, xmax);
}
// Similar to RescaleFromInteger, but offsets the cell center by the given amount.
template<typename K, typename I>
K RescaleFromInteger(I i, I n, K offset, K xmin, K xmax) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  ASSERT(n != 0, "Number of cells must not be 0");
  return Interpolate((static_cast<K>(i) + offset) / static_cast<K>(n), xmin, xmax);
}
// Corresponding inverse operation to RescaleFromInteger. For a value x in the interval [xmin,xmax]
// it returns the corresponding integer cell.
// `fractional` will contain the fractional position in the cell.
//
// Note: This function will also work if xmin > xmax or if n is zero or negative.
template<typename K, typename I>
I RescaleToInteger(K x, K xmin, K xmax, I n) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  return std::floor(Rescale(x, xmin, xmax) * static_cast<K>(n));
}
template<typename K, typename I>
I RescaleToInteger(K x, K xmin, K xmax, I n, K& fractional) {
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  const K p = Rescale(x, xmin, xmax) * static_cast<K>(n);
  const I i = std::floor(p);
  fractional = p - static_cast<K>(i);
  return i;
}

}  // namespace isaac
