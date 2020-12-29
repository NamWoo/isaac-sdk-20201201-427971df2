/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <type_traits>

#include "engine/core/array/cpu_array_view.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

// Separate type for CUDA-based memory views to allow compile-time checks.
template <typename K>
class CudaArrayView : public CpuArrayView<K> {
  using value_type = K;
  using CpuArrayView<K>::CpuArrayView;
};

// Separate type for CUDA-based memory views to allow compile-time checks.
template <typename K>
using CudaMemoryConstView = CudaArrayView<std::add_const_t<K>>;

template <typename T>
struct BufferTraits<CudaArrayView<T>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = !std::is_const<T>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CudaArrayView<T>;
  using buffer_const_view_t = CudaMemoryConstView<T>;
};

}  // namespace isaac
