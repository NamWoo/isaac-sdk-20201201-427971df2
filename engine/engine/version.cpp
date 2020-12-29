/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "engine/version.hpp"

extern "C" {

constexpr char ISAAC_SDK_VERSION[] = "release-2020.2";

const char* IsaacSdkVersion() {
  return ISAAC_SDK_VERSION;
}
}
