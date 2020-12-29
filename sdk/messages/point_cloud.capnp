#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x8846db30e403e12a;

using import "tensor.capnp".TensorProto;

# A message containing information about a set of points , also called "point cloud". Points can
# have various attributes. This message provides the position of the point in 3D space, a normal,
# a color and an intensity. Point attributes are each stored as NxM tensors. N is the number
# points which has to be identical for all attributes or 0. M is the dimension of the corresponding
# attribute.
struct PointCloudProto {
  # The position of points. Most commonly XYZ stored as three 32-bit floats.
  positions @0:  TensorProto;
  # A normal of the point. Most commonly a unit vector stored as three 32-bit floats.
  normals @1: TensorProto;
  # A color value for each point. Most commonly an RGB value stored as three 32-bit unit floats.
  colors @2: TensorProto;
  # The intensity for each point used for example by LiDAR sensors. Most commonly a single 32-bit
  # float.
  intensities @3: TensorProto;
}
