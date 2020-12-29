/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cuda_malloc_allocator.hpp"

#include <cstdlib>

#include "cuda_runtime.h"  // NOLINT
#include "engine/core/assert.hpp"

namespace isaac {

auto CudaMallocAllocator::allocateBytes(size_t size) -> pointer_t {
  byte* pointer;
  const cudaError_t error = cudaMalloc(&pointer, size);
  ASSERT(error == cudaSuccess, "Could not allocate memory. Error: %d", error);
  return pointer;
}

void CudaMallocAllocator::deallocateBytes(pointer_t pointer, size_t size) {
  if (pointer != nullptr) {
    const cudaError_t error = cudaFree(pointer);
    ASSERT(error == cudaSuccess, "Could not free memory. Error: %d", error);
  }
}

}  // namespace isaac
