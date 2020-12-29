/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "algorithm.hpp"

#include "cuda_runtime.h"  // NOLINT
#include "engine/core/assert.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

namespace {

// Finds the correct memory copy kind for the given storage modes of source and target
cudaMemcpyKind StorageModeToCudaMemcpyKind(BufferStorageMode source_storage,
                                           BufferStorageMode target_storage) {
  if (source_storage == BufferStorageMode::Host && target_storage == BufferStorageMode::Host) {
    return cudaMemcpyHostToHost;
  }
  if (source_storage == BufferStorageMode::Cuda && target_storage == BufferStorageMode::Host) {
    return cudaMemcpyDeviceToHost;
  }
  if (source_storage == BufferStorageMode::Host && target_storage == BufferStorageMode::Cuda) {
    return cudaMemcpyHostToDevice;
  }
  if (source_storage == BufferStorageMode::Cuda && target_storage == BufferStorageMode::Cuda) {
    return cudaMemcpyDeviceToDevice;
  }
  PANIC("Invalid storage modes");
}

}  // namespace

void CopyArrayRaw(const void* source, BufferStorageMode source_storage,
                  void* target, BufferStorageMode target_storage, size_t size) {
  // We can currently use cuda memory copy as it supports all copy modes so far. This might change
  // once we have more storage options.
  cudaMemcpy(target, source, size, StorageModeToCudaMemcpyKind(source_storage, target_storage));
}

void CopyMatrixRaw(const void* source, size_t source_stride, BufferStorageMode source_storage,
                   void* target, size_t target_stride, BufferStorageMode target_storage,
                   size_t rows, size_t row_size) {
  // We can currently use cuda memory copy as it supports all copy modes so far. This might change
  // once we have more storage options.
  const cudaError_t error = cudaMemcpy2D(
      target, target_stride, source, source_stride, row_size, rows,
      StorageModeToCudaMemcpyKind(source_storage, target_storage));
  ASSERT(error == cudaSuccess, "Could not copy memory. Error: %d", error);
}

}  // namespace isaac
