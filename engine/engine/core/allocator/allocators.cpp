/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "allocators.hpp"

#include <memory>

#include "engine/core/allocator/cached_allocator.hpp"
#include "engine/core/allocator/cuda_malloc_allocator.hpp"
#include "engine/core/allocator/malloc_allocator.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/singleton.hpp"

namespace isaac {

// A helper class to use an allocator with a singleton
struct CachedMallocAllocatorSingletonHelper {
  CachedMallocAllocatorSingletonHelper() {
    impl = std::make_unique<CachedAllocator>(std::make_unique<MallocAllocator>());
  }
  std::unique_ptr<AllocatorBase> impl;
};

byte* CpuAllocator::Allocate(size_t size) {
  return GetCpuAllocator()->allocateBytes(size);
}

void CpuAllocator::Deallocate(byte* pointer, size_t size) {
  return GetCpuAllocator()->deallocateBytes(pointer, size);
}

AllocatorBase* GetCpuAllocator() {
  return Singleton<CachedMallocAllocatorSingletonHelper>::Get().impl.get();
}

// A helper class to use an allocator with a singleton
struct CachedCudaMallocAllocatorSingletonHelper {
  CachedCudaMallocAllocatorSingletonHelper() {
    impl = std::make_unique<CachedAllocator>(std::make_unique<CudaMallocAllocator>());
  }
  std::unique_ptr<AllocatorBase> impl;
};

byte* CudaAllocator::Allocate(size_t size) {
  return GetCudaAllocator()->allocateBytes(size);
}

void CudaAllocator::Deallocate(byte* pointer, size_t size) {
  return GetCudaAllocator()->deallocateBytes(pointer, size);
}

AllocatorBase* GetCudaAllocator() {
  return Singleton<CachedCudaMallocAllocatorSingletonHelper>::Get().impl.get();
}

}  // namespace isaac
