/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cstring>

#include "engine/core/array/byte_array.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// Copy tensors with compatibale memory layouts
template <typename K, typename SourceDimensions, typename SourceBuffer,
          typename TargetDimensions, typename TargetBuffer>
void Copy(const TensorBase<K, SourceDimensions, SourceBuffer>& source,
          TensorBase<K, TargetDimensions, TargetBuffer>& target) {
  // Asserts that tensors have identical dimensions
  ASSERT(source.dimensions() == target.dimensions(), "Tensor dimensions mismatch: %zd elements "
         "vs %zd elements", source.element_count(), target.element_count());
  // Copy the bytes
  CopyArrayRaw(source.data().begin(), BufferTraits<SourceBuffer>::kStorageMode,
               target.data().begin(), BufferTraits<TargetBuffer>::kStorageMode,
               source.data().size());
}

// Copy tensors with compatibale memory layouts
template <typename K, typename SourceDimensions, typename SourceBuffer,
          typename TargetDimensions, typename TargetPointer>
void Copy(const TensorBase<K, SourceDimensions, SourceBuffer>& source,
          TensorBase<K, TargetDimensions, detail::BufferBase<TargetPointer>> target) {
  // Asserts that tensors have identical dimensions
  ASSERT(source.dimensions() == target.dimensions(), "Tensor dimensions mismatch: %zd elements "
         "vs %zd elements", source.element_count(), target.element_count());
  // Copy the bytes
  CopyArrayRaw(source.data().begin(), BufferTraits<SourceBuffer>::kStorageMode,
               target.data().begin(), BufferTraits<detail::BufferBase<TargetPointer>>::kStorageMode,
               source.data().size());
}

// Fills a tensor with the given value.
template <typename K, typename Dimensions,  typename Container>
void Fill(TensorBase<K, Dimensions, Container>& tensor, K value) {
  static_assert(TensorBase<K, Dimensions, Container>::kIsMutable,
                "Cannot Fill const buffer");
  std::fill(tensor.element_wise_begin(), tensor.element_wise_end(), value);
}

// flattens the tensor into a buffer with all stride pitch removed.
template <typename K, typename Dimensions,  typename Container>
void FlattenData(const TensorBase<K, Dimensions, Container>& tensor, ByteArray* out) {
  ASSERT(out->size() == tensor.data().size(),
         "Buffer must be large enough to hold results");
  std::memcpy(out, tensor.data().begin(), tensor.data().size());
}

}  // namespace isaac
