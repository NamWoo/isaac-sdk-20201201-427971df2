#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xb16e3374d43c79dd;

using import "element_type.capnp".ElementType;

# An image produced for example by a camera
struct ImageProto {
  # Type of channel elements
  elementType @0: ElementType;

  # Number of rows in the image
  rows @1: UInt16;
  # Number of columns in the image
  cols @2: UInt16;
  # Number of channels per pixel
  channels @3: UInt16;

  # Index of buffer which contains the image data
  dataBufferIndex @4: UInt16;
}
