/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/allocator/allocator_base.hpp"

namespace isaac {

// An cuda allocator using malloc
class CudaMallocAllocator : public AllocatorBase {
 public:
  pointer_t allocateBytes(size_t size) override;
  void deallocateBytes(pointer_t handle, size_t size) override;
};

}  // namespace isaac
