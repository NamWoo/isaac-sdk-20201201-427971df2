/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "cuda_runtime.h"

namespace isaac {

// Computes a positive integer x such that (x - 1) * b < a <= x * b
template<typename T>
T DivRoundUp(T a, T b) {
  return (a + b - T{1}) / b;
}

// Same as DivRoundUp but for all three elements. This is for example useful in computing
// launch parameters for CUDA kernels.
inline dim3 DivRoundUp(dim3 a, dim3 b) {
  return {
    DivRoundUp(a.x, b.x),
    DivRoundUp(a.y, b.y),
    DivRoundUp(a.z, b.z)
  };
}

}  // namespace isaac
