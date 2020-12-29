#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xd4ae0ae1354bfbe1;

using import "../../../../messages/uuid.capnp".UuidProto;

struct HeaderProto {
  timestamp @0: Int64;
  uuid @1: UuidProto;
  tag @2: Text;
  acqtime @3: Int64;
  minipayload @4: Data;
  segments @5: List(UInt16);
  buffers @6: List(UInt32);
}
