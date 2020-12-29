#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xd7c613f65b8ebd79;

using import "element_type.capnp".ElementType;

# A n-dimensional tensor
struct TensorProto {
  # Type of channel elements
  elementType @0: ElementType;
  # Dimensions of the tensor
  sizes @1: List(Int32);
  # deprecated - not used anymore
  scanlineStride @2: UInt32;
  # Index of buffer which contains the tensor data
  dataBufferIndex @3: UInt16;
}
