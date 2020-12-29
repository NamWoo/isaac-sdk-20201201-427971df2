#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xad79d0e15fcdd8a7;

using import "tensor.capnp".TensorProto;

# A message about laser range scans published for example from a LIDAR sensor
# The message does not have to capture a full 360 degree scan, as most sensors publish partial
# slices at high frequency instead of waiting to complete a full scan.
struct RangeScanProto {
  # Normalized distance to target (see rangeScale) of the scanned rays. Data type is 16-bit integer.
  # For each angle theta there is a ray for every phi angle. So the total number of rays is
  # length(theta) * length(phi). First rank is theta and second rank is phi.
  ranges @0: TensorProto;
  # Normalized ray return intensity (see intensityScale) of the scanned rays. Data type is 8-bit
  # integer. For each angle theta there is a ray for every phi angle. So the total number of rays is
  # length(theta) * length(phi). First rank is theta and second rank is phi.
  # This entry is optional. Default is full intensity for all rays.
  intensities @1: TensorProto;
  # table of theta (horizontal) angles
  theta @2: List(Float32);
  # table of phi (vertical) angles
  phi @3: List(Float32);
  # Scale factor which can be used to convert a range value from a 16-bit integer to meters. The
  # conversion formula is: range[meters] = range[normalized] / 0xFFFF * rangeScale
  rangeDenormalizer @4: Float32;
  # Scale factor which can be used to convert an intensity value from an 8-bit integer to meters.
  # The conversion formula is: intensity = intensity[normalized] / 0xFF * intensityScale
  intensityDenormalizer @5: Float32;
  # Delay in microseconds between firing
  deltaTime @6: UInt16;
  # Beams with a range smaller than or equal to this distance (in meters) are considered to have
  # returned an invalid measurement.
  invalidRangeThreshold @7: Float64;
  # Beams with a range larger than or equal to this distance (in meters) are considered to not have
  # hit an obstacle within the maximum possible range of the sensor.
  outOfRangeThreshold @8: Float64;
}
