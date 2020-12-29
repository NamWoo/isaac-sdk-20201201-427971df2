#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xaee793b3d97a7d06;

# A unique identifier following the UUID specifications
struct UuidProto {
  # First 8 bytes out of 16
  lower @0: UInt64;
  # Second 8 bytes out of 16
  upper @1: UInt64;
}
