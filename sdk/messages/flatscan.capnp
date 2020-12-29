#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x9a10476e7d828ca0;

# A 2D range scan which is essentially a flat version of the 3D RangeScanProto
struct FlatscanProto {
  # Angles (in radians) under which rays are shot
  angles @0: List(Float32);
  # Return distance of the ray
  ranges @1: List(Float32);
  # Beams with a range smaller than or equal to this distance are considered to have returned an
  # invalid measurement.
  invalidRangeThreshold @2: Float64;
  # Beams with a range larger than or equal to this distance are considered to not have hit an
  # obstacle within the maximum possible range of the sensor.
  outOfRangeThreshold @3: Float64;
  # Return the visibility of a given ray (the longest valid distance of a beams in this direction)
  # This field is optional, however if it is set, it must have the same size as ranges and angles.
  visibilities @4: List(Float32);
}
