#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xed7e5fd83e7c7faa;

using import "element_type.capnp".ElementType;
using import "tensor.capnp".TensorProto;
using import "math.capnp".VectorXiProto;

# A type containing measurable quantities for a bag of entities.
struct CompositeProto {
  # Describes the unit or type which was used to measure a value.
  enum Measure {
    none @0;
    # [s]
    time @1;
    # [kg]
    mass @2;
    # [1] or [m]
    position @3;
    # [1/s] or [m/s]
    speed @4;
    # [1/s^2] or [m/s^2]
    acceleration @5;
    # [rad]
    rotation @6;
    # [rad/s]
    angularSpeed @7;
    # [rad/s^2]
    angularAcceleration @8;
    # a normal
    normal @9;
    # a color
    color @10;
  }

  # Meta data for a value describing its semantic meaning
  struct Quantity {
    # The name of the entity of this quantity
    entity @0: Text;
    # Describes the machine representation type of the quantity
    elementType @1: ElementType;
    # A hint for the semantic meaning of the quantity
    measure @2: Measure;
    # The dimensions of a potentially multi-dimensional quantity. Can be omitted for scalars.
    dimensions @3: VectorXiProto;
  }

  # A list of all quantities in this composite
  quantities @0: List(Quantity);

  # A hash representing the schema which can be used to check if two schemas are identical
  schemaHash @1: Text;

  # The quantity values as described in the quantity schema. Can a vector, a matrix for representing
  # time series, or a rank-3 tensor for batches of time series
  values @2: TensorProto;
}

