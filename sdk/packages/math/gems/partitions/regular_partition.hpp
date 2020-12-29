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
#include "engine/core/math/utils.hpp"

namespace isaac {
namespace partitions {

// A regular partition which maps 1-dimensional Cartesian coordinates to an array of equally sized
// cells. The Cartesian coordinate x is mapped to the cell r(x/s) where s is the size of a cell and
// r(x) rounds x to the nearest integer.
// For example if the cell size is 1.0, it would mean that the cell 2 is covering the interval
// [-1.5 | +2.5[ in Cartesian space.
template <typename Scalar, typename Index = int>
class RegularPartition {
 public:
  // Default constructor creates a single bucket unit interval
  RegularPartition(Scalar cell_size = Scalar(1)) {
    setCellSize(cell_size);
  }

  // The size of a cell
  Scalar cell_size() const { return cell_size_; }
  // Changes the cell size
  void setCellSize(Scalar cell_size) {
    ASSERT(cell_size > Scalar(0), "Cell size must be greater than 0, was %f", cell_size);
    cell_size_ = cell_size;
    cell_size_inv_ = Scalar(1) / cell_size;
  }

  // Computes smooth cell coordinates for a Cartesian coordinate
  Scalar toContinuous(Scalar value) const {
    return cell_size_inv_ * value;
  }
  // Computes the index of the cell which contains the given Cartesian coordinate. For example for
  // a cell size of 1.0 for all values in the range [-2.5,+2.5[ this function would return the grid
  // index 2.
  Index toNearest(Scalar value) const {
    return static_cast<Index>(FastRoundDoubleToInt32(static_cast<double>(toContinuous(value))));
  }
  // Computes the index of a cell and a remainder for a given Cartesian coordinate such that the
  // result can be used in linear interpolation. For example for a coordinate of 2.3 this would
  // return 2 as index and 0.3 as remainder. These values can be used directly to linearly
  // interpolate a value at the given coordinate. In general with `i = toLinear(x, r)` we can
  // compute `g(x) = (1 - r) * g[i] + r * g[i + 1]`. Note that this function returns a different
  // index than the function toNearest.
  Index toLinear(Scalar value, Scalar& remainder) const {
    value = toContinuous(value);
    const Scalar floor_value = std::floor(value);
    remainder = value - floor_value;
    return static_cast<Index>(floor_value);
  }

  // Computes the mid point value for a bucket. For example for cell 2 and a cell size of 1.0 this
  // will return 2.0.
  Scalar cellCenter(Index bucket) const {
    return cell_size_ * static_cast<Scalar>(bucket);
  }
  // Computes the lower bound for a bucket. Warning: this will not consider openess or closeness
  // for the cell interval, but return the halfpoint. For example for cell -2 and a cell size of 1.0
  // this will return -2.5, although this value is not actually in the interval and toNearest would
  // return -3 for a value of -2.5.
  Scalar cellLower(Index bucket) const {
    return cell_size_ * (static_cast<Scalar>(bucket) - Scalar(0.5));
  }
  // Computes the upper bound for a bucket. Warning: this will not consider openess or closeness
  // for the cell interval, but return the halfpoint. For example for cell 2 and a cell size of 1.0
  // this will return 2.5, although this value is not actually in the interval and toNearest would
  // return 3 for a value of 2.5.
  Scalar cellUpper(Index bucket) const {
    return cell_size_ * (static_cast<Scalar>(bucket) + Scalar(0.5));
  }

 private:
  Scalar cell_size_;
  Scalar cell_size_inv_;
};

}  // namespace partitions
}  // namespace isaac
