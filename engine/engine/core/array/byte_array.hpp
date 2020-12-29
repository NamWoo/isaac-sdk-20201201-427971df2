/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/array/cpu_array.hpp"
#include "engine/core/array/cpu_array_view.hpp"
#include "engine/core/byte.hpp"

namespace isaac {

// A byte array which owns its memory.
using ByteArray = CpuArray<byte>;

// A mutable array of bytes which does not own its memory
using ByteArrayView = CpuArrayView<byte>;

// A non-mutable array of bytes which does not own its memory
using ByteArrayConstView = ConstCpuArrayView<byte>;

}  // namespace isaac
