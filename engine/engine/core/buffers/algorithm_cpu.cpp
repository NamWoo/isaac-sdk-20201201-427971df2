/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "algorithm.hpp"

#include <cstring>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

void CopyArrayRaw(const void* source, BufferStorageMode source_mode,
                  void* target, BufferStorageMode target_storage, size_t size) {
  // Only device-device copies supported
  ASSERT(source_mode == BufferStorageMode::Host && target_storage == BufferStorageMode::Host,
         "When GPU is disabled only host memory is supported");
  std::memcpy(target, source, size);
}

void CopyMatrixRaw(const void* source, size_t source_stride, BufferStorageMode source_mode,
                   void* target, size_t target_stride, BufferStorageMode target_mode,
                   size_t rows, size_t row_size) {
  if (source_stride == target_stride) {
    // If stride is equal copy the whole buffer (including possible access stride)
    CopyArrayRaw(source, source_mode, target, target_mode, rows * source_stride);
  } else {
    // If stride are not equal copy row by row
    const byte* source_begin = reinterpret_cast<const byte*>(source);
    const byte* source_end = source_begin + rows * source_stride;
    byte* target_begin = reinterpret_cast<byte*>(target);
    for (; source_begin != source_end; source_begin += source_stride,
                                       target_begin += target_stride) {
      CopyArrayRaw(source_begin, source_mode, target_begin, target_mode, row_size);
    }
  }
}

}  // namespace isaac
