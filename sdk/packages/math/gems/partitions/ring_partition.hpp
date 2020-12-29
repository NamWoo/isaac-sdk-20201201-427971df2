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
#include "engine/core/constants.hpp"
#include "engine/core/math/utils.hpp"
#include "packages/math/gems/partitions/regular_interval_partition.hpp"

namespace isaac {
namespace partitions {

// A regular partition of a looping interval. Note that it is assumed that the length of the loop
// is small enough so that we can work with 32-bit integer bucket indices (which is likely the
// case).
template <typename Scalar, typename Index = int>
class RingPartition {
 public:
  // Creates a regular ring partition for radians with given resolution
  static RingPartition Angular(Index num_cells) {
    return RingPartition{TwoPi<Scalar>, num_cells};
  }

  // Default constructor creates a single bucket unit interval
  RingPartition()
      : RingPartition(Scalar(1), 1) {}

  // Creates a regular ring partition from the length of the ring and a bucket size
  RingPartition(Scalar length, Index num_cells)
      : regular_(RegularIntervalPartition<Scalar>::FromCellCount(Scalar(0), length, num_cells)) {}

  Index num_cells() const { return regular_.num_cells(); }
  Scalar length() const { return regular_.max(); }
  Scalar delta() const { return regular_.delta(); }

  // Computes the bucket in which a value falls (wrapped around correctly)
  Index toNearest(Scalar value) const {
    return PositiveModulo(regular_.toNearest(value), num_cells());
  }
  Index toLinear(Scalar value, Scalar& remainder) const {
    return PositiveModulo(regular_.toLinear(value, remainder), num_cells());
  }

  // Computes the mid point value for a bucket (wrapped around correctly)
  Scalar cellCenter(Index bucket) const {
    return regular_.cellCenter(PositiveModulo(bucket, num_cells()));
  }

  // Checks if a bucket coordinate is inside the partition
  bool inRange(Index bucket) const { return true; }

 private:
  RegularIntervalPartition<Scalar, Index> regular_;
};

}  // namespace partitions
}  // namespace isaac
