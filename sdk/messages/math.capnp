#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xe57f42d4f2141d79;

# A quaternion using 32-bit floats for example to represent rotations
struct QuaternionfProto {
  w @0 : Float32;
  x @1 : Float32;
  y @2 : Float32;
  z @3 : Float32;
}

# A quaternion using 64-bit doubles for example to represent rotations
struct QuaterniondProto {
  w @0 : Float64;
  x @1 : Float64;
  y @2 : Float64;
  z @3 : Float64;
}

# A 2-dimensional vector using 32-bit floats
struct Vector2fProto {
  x @0 : Float32;
  y @1 : Float32;
}

# A 3-dimensional vector using 32-bit floats
struct Vector3fProto {
  x @0 : Float32;
  y @1 : Float32;
  z @2 : Float32;
}

# A 4-dimensional vector using 32-bit floats
struct Vector4fProto {
  x @0 : Float32;
  y @1 : Float32;
  z @2 : Float32;
  w @3 : Float32;
}

# A X-dimensional vector using 32-bit floats
struct VectorXfProto {
  coefficients @0 : List(Float32);
}

# A 2-dimensional vector using 64-bit doubles
struct Vector2dProto {
  x @0 : Float64;
  y @1 : Float64;
}

# A 3-dimensional vector using 64-bit doubles
struct Vector3dProto {
  x @0 : Float64;
  y @1 : Float64;
  z @2 : Float64;
}

# A 4-dimensional vector using 64-bit doubles
struct Vector4dProto {
  x @0 : Float64;
  y @1 : Float64;
  z @2 : Float64;
  w @3 : Float64;
}

# A X-dimensional vector using 64-bit doubles
struct VectorXdProto {
  coefficients @0 : List(Float64);
}

# A 2-dimensional vector using 32-bit integers
struct Vector2iProto {
  x @0 : Int32;
  y @1 : Int32;
}

# A 3-dimensional vector using 32-bit integers
struct Vector3iProto {
  x @0 : Int32;
  y @1 : Int32;
  z @2 : Int32;
}

# A 4-dimensional vector using 32-bit integers
struct Vector4iProto {
  x @0 : Int32;
  y @1 : Int32;
  z @2 : Int32;
  w @3 : Int32;
}

# A X-dimensional vector using 32-bit integers
struct VectorXiProto {
  coefficients @0 : List(Int32);
}

# A 3-dimensional vector using unsigned 8-bit integers
struct Vector3ubProto {
  x @0 : UInt8;
  y @1 : UInt8;
  z @2 : UInt8;
}

# A matrix with dimensions 3x3 using 64-bit floating points
struct Matrix3dProto {
  row1 @0: Vector3dProto;
  row2 @1: Vector3dProto;
  row3 @2: Vector3dProto;
}

# A pose for 2-dimensional space with rotation and translation using 32-bit floats
struct Pose2fProto {
  rotation @0 : SO2fProto;
  translation @1 : Vector2fProto;
}

# A pose for 2-dimensional space with rotation and translation using 64-bit doubles
struct Pose2dProto {
  rotation @0 : SO2dProto;
  translation @1 : Vector2dProto;
}

# A pose for 3-dimensional space with rotation and translation using 32-bit floats
struct Pose3fProto {
  rotation @0 : SO3fProto;
  translation @1 : Vector3fProto;
}

# A pose for 3-dimensional space with rotation and translation using 64-bit doubles
struct Pose3dProto {
  rotation @0 : SO3dProto;
  translation @1 : Vector3dProto;
}

# A rotation in 2-dimensional Euclidean space using 32-bit floats
struct SO2fProto {
  # unit complex number (cos(a), sin(a)) for rotation angle a
  q @0: Vector2fProto;
}

# A rotation in 2-dimensional Euclidean space using 64-bit doubles
struct SO2dProto {
  # unit complex number (cos(a), sin(a)) for rotation angle a
  q @0: Vector2dProto;
}

# A rotation in 3-dimensional Euclidean space using 32-bit floats
struct SO3fProto {
  # a normalized quaternion
  q @0: QuaternionfProto;
}

# A rotation in 3-dimensional Euclidean space using 64-bit doubles
struct SO3dProto {
  # a normalized quaternion
  q @0: QuaterniondProto;
}

# A type containing mean and covariance for a Pose2
struct Pose2MeanAndCovariance {
  # The mean of the distribution
  mean @0: Pose2dProto;
  # The covariance of the distribution in the reference frame of the mean.
  covariance @1: Matrix3dProto;
}

# A list of samples of type Pose2
struct Pose2Samples {
  # The state for every sample
  states @0: List(Pose2dProto);
  # A weight for every sample. Optional and default is equal weighting.
  weights @1: List(Float64);
}
