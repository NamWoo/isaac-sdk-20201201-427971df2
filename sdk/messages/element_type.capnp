#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
####################################################################################
@0xe3dd4334718a3123;

# The choices available as element type for data types such as images, tensors, or sample clouds
enum ElementType {
  unknown @11;
  uint8 @0;
  uint16 @1;
  uint32 @5;
  uint64 @6;
  int8 @7;
  int16 @8;
  int32 @9;
  int64 @10;
  float16 @4;
  float32 @2;
  float64 @3;
}
