/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <tuple>

#include "engine/core/math/types.hpp"
#include "packages/math/gems/partitions/plane_partition.hpp"

namespace isaac {
namespace partitions {

namespace space_details {

// A partition of 3-dimensional space based on partitions defined by the chosen policy.
template <typename Scalar, typename IndexType, typename Policy>
struct Space3PartitionBase : public Policy {
  using Vector3i = Vector3<IndexType>;
  using Vector3k = Vector3<Scalar>;

  // Converts from world coordinates to grid coordinates
  Vector3k toContinuous(const Vector3k& point) const {
    return Vector3k{this->template partition<0>().toContinuous(point[0]),
                    this->template partition<1>().toContinuous(point[1]),
                    this->template partition<2>().toContinuous(point[2])};
  }

  // Converts from map coordinates to integer cell coordinate
  Vector3i toNearest(const Vector3k& point) const {
    return Vector3i{this->template partition<0>().toNearest(point[0]),
                    this->template partition<1>().toNearest(point[1]),
                    this->template partition<2>().toNearest(point[2])};
  }
  // Converts from map coordinates to integer cell coordinate and also returns the fractional part
  Vector3i toLinear(const Vector3k& point, Vector3k& fractional) const {
    return Vector3i{this->template partition<0>().toLinear(point[0], fractional[0]),
                    this->template partition<1>().toLinear(point[1], fractional[1]),
                    this->template partition<2>().toLinear(point[2], fractional[2])};
  }

  // Gets map coordinates for the center of a cell
  Vector3k cellCenter(const Vector3i& cell) const {
    return Vector3k{this->template partition<0>().cellCenter(cell[0]),
                    this->template partition<1>().cellCenter(cell[1]),
                    this->template partition<2>().cellCenter(cell[2])};
  }

  // The dimensions of a cell, i.e. the side lengths of the cell area
  Vector3k cellDimensions() const {
    return Vector3k{this->template partition<0>().cell_size(),
                    this->template partition<1>().cell_size(),
                    this->template partition<2>().cell_size()};
  }

  // Measures the distance between two grid coordinates
  Scalar cartesianDistance(const Vector3i& cell_1, const Vector3i& cell_2) const {
    return (cellCenter(cell_1) - cellCenter(cell_2)).norm();
  }
};

}  // namespace space_details

// A partition for three-dimensional space
template <typename Scalar, typename Index,
          // partition for the first dimension
          template<class, class> class Partition1,
          // partition for the second dimension
          template<class, class> class Partition2,
          // partition for the third dimension
          template<class, class> class Partition3>
using Space3Partition = space_details::Space3PartitionBase<Scalar, Index,
    space_details::TuplePartitionPolicy<Partition1<Scalar, Index>,
                                        Partition2<Scalar, Index>,
                                        Partition3<Scalar, Index>>>;

}  // namespace partitions
}  // namespace isaac
