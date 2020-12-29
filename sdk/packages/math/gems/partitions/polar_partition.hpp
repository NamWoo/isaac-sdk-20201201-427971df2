/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>

#include "packages/math/gems/partitions/regular_interval_partition.hpp"
#include "packages/math/gems/partitions/ring_partition.hpp"

namespace isaac {
namespace partitions {

// A polar coordinate partition using given partitions for sector angle and range partition.
template <typename Scalar, typename Index,
          // partition type for sectors, i.e. heading angle
          template<class, class> class SectorPartitionT,
          // partition type for ranges, i.e. distance from origin
          template<class, class> class RangePartitionT>
class PolarPartition {
 public:
  using Vector2i = Vector2<Index>;
  using Vector2k = Vector2<Scalar>;

  // Computes the cell in which a 2D point falls (no bound checks)
  Vector2i toNearest(const Vector2k& position) const {
    const Scalar angle = heading(position.x(), position.y());
    const Scalar range = position.norm();
    return Vector2i{
      sector_partition.toNearest(angle),
      range_partition.toNearest(range)
    };
  }

  // Computes the mid point value for a bucket (no bound checks)
  Vector2k cellCenter(const Vector2i& bucket) const {
    const Scalar angle = sector_partition.cellCenter(bucket[0]);
    const Scalar range = range_partition.cellCenter(bucket[1]);
    return range * direction(angle);
  }

  // Checks if a bucket coordinate is inside the partition
  bool inRange(const Vector2i& bucket) const {
    return sector_partition.inRange(bucket[0]) && range_partition.inRange(bucket[1]);
  }

  SectorPartitionT<Scalar, Index> sector_partition;
  RangePartitionT<Scalar, Index> range_partition;

 private:
  // Computes the heading for point (x,y)
  Scalar heading(Scalar x, Scalar y) const {
    // TODO use a look-up table
    if (IsAlmostZero(x) && IsAlmostZero(y)) {
      return Scalar(0);  // TODO
    } else {
      return std::atan2(y, x);
    }
  }

  // Computes the direction for given angle
  Vector2k direction(Scalar angle) const {
    // TODO use a look-up table
    return {std::cos(angle), std::sin(angle)};
  }
};

template <typename Scalar, typename Index = int>
using RegularPolarPartition
    = PolarPartition<Scalar, Index, RingPartition, RegularIntervalPartition>;

template <typename Scalar, typename Index = int>
using RegularPolarSlicePartition
    = PolarPartition<Scalar, Index, RegularIntervalPartition, RegularIntervalPartition>;

}  // namespace partitions
}  // namespace isaac
