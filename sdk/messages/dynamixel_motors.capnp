#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x8e7a7cd9b3869052;

# Commands commands for a set of dynamixel motors running for example as a dasy chain.
struct DynamixelMotorsProto {
  # Motor protos
  struct MotorProto {
    # Motor ID
    id @0 :Int64;
    # Current position
    position @1 :Int16;
    # Tick in milliseconds since startup
    tick @2 :Int64;
    # Is the servo moving
    moving @3 :Bool;
  }

  # A single command per motor
  motors @0: List(MotorProto);
}
