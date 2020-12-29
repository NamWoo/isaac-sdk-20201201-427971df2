#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x80f2c08f086cc01a;

using import "tensor.capnp".TensorProto;

# A message used to transport states of dynamic systems. This is used closely together with the
# state gem to be found in //engine/gems/state. For example you can define the robot state using the
# compile-time types defined in that gem, and then serialize that state into a message. The
# receiver of that message can deserialize the state back into the compile-time robot state type.
struct StateProto {
  # A densely packed representation of the state data as floating point numbers. The lowest rank
  # indicates state element dimension, the second rank is time for a potential timeseries, and
  # the third rank is for the batch index.
  pack @0: TensorProto;
  # The schema describing the format of the state vector
  schema @1: Text;
  # Alternative way to pass the data (for python)
  data @2: List(Float64);
}
