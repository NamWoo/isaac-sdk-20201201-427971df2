#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xb4c48bd565c8e9c8;

# Message published by an inertial measurement unit (IMU)
struct ImuProto {
  # The linear acceleration of the body frame along the primary axes
  linearAccelerationX @0 :Float32;
  linearAccelerationY @1 :Float32;
  linearAccelerationZ @2 :Float32;
  # The angular velocity of the body frame around the primary axes
  angularVelocityX @3 :Float32;
  angularVelocityY @4 :Float32;
  angularVelocityZ @5 :Float32;
  # Optional angles as integrated by a potential internal estimator
  angleYaw @6 :Float32;
  anglePitch @7 :Float32;
  angleRoll @8 :Float32;
}
