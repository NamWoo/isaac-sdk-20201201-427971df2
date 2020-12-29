/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "malloc_allocator.hpp"

#include <cstdlib>

namespace isaac {

auto MallocAllocator::allocateBytes(size_t size) -> pointer_t {
  if (size == 0) return nullptr;
  return reinterpret_cast<pointer_t>(std::malloc(size));
}

void MallocAllocator::deallocateBytes(pointer_t handle, size_t size) {
  if (handle == nullptr || size == 0) return;
  std::free(reinterpret_cast<void*>(handle));
}

}  // namespace isaac
