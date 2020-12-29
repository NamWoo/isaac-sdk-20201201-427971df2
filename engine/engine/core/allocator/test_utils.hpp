/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>

#include "engine/core/allocator/allocator_base.hpp"

namespace isaac {

// Does `batch` allocations using the callback to get the size to allocate. Then deallocates
// all allocated buffers. Repeates this `count` times.
void BatchAllocDealloc(AllocatorBase& allocator, int count, int batch,
                       std::function<size_t()> size_cb);

// Gives a random size in the order ot 20k using a gamma distribution and some rounding to get
// similar values with higher probability. If `pool_size` is not 0 generates a fixed pool of sizes
// and returns from it in round robin fashion.
std::function<size_t()> RandomSizeGamma(size_t pool_size = 0);

}  // namespace isaac
