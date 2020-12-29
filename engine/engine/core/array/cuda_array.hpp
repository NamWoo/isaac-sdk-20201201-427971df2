/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <type_traits>

#include "engine/core/array/cpu_array_view.hpp"
#include "engine/core/array/cuda_array_view.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

// A helper struct to manage GPU device memory
struct RawCudaArray {
  // Disallow copies
  RawCudaArray(const RawCudaArray&) = delete;
  RawCudaArray& operator=(const RawCudaArray&) = delete;

  // Constructors for empty objects or given size
  RawCudaArray();
  RawCudaArray(size_t num_bytes);

  virtual ~RawCudaArray();

  // The size of the buffer in bytes
  size_t num_bytes() const { return num_bytes_; }
  // A pointer to the device memory pointer
  const void* pointer() const { return ptr_; }
  void* pointer() { return ptr_; }

  // Copies bytes from the given host buffer to the device in blocking mode
  void copyToDevice(const void* host_source);
  // Copies bytes from the device to the given host buffer in blocking mode
  void copyToHost(void* host_target);

  // Resizes the underlying buffer potentially reallocating memory and invalidating the buffer
  void resize(size_t num_bytes);

 private:
  size_t num_bytes_;
  void* ptr_;
};

// A convenience type which handles a typed array of GPU device memory
template <typename T>
struct CudaArray {
  using value_type = T;
  using mutable_view_t = CudaArrayView<T>;
  using const_view_t = CudaArrayView<std::add_const_t<T>>;

  // By default construct a buffer with no elements, otherwise create it with the desired size
  CudaArray() : size_(0) {
    device_memory_ = std::make_unique<RawCudaArray>();
  }
  CudaArray(size_t size) : size_(size) {
    device_memory_ = std::make_unique<RawCudaArray>(size_ * sizeof(T));
  }

  // The number of elements in the buffer
  size_t size() const { return size_; }
  // Pointer to the first element in the buffer
  const T* pointer() const {
    return static_cast<const T*>(device_memory_->pointer());
  }
  T* pointer() {
    return static_cast<T*>(device_memory_->pointer());
  }
  // Pointer to the first element in the buffer
  const T* begin() const {
    return pointer();
  }
  T* begin() {
    return pointer();
  }
  // Pointer after the last element in the buffer
  const T* end() const {
    return pointer() + size();
  }
  T* end() {
    return pointer() + size();
  }

  // Converts the object to a buffer
  operator const T*() const {
    return pointer();
  }
  operator T*() {
    return pointer();
  }

  // Creates a view which provides read and write access from this buffer object.
  mutable_view_t view() {
    return mutable_view_t{begin(), size()};
  }
  const_view_t view() const {
    return const_view_t{begin(), size()};
  }
  // Creates a view which only provides read access from this buffer object.
  const_view_t const_view() const {
    return const_view_t{begin(), size()};
  }

  // Resizes the underlying buffer to hold the given number of elements. This potentially
  // reallocates the buffer thus invalidating the pointer.
  void resize(size_t size) {
    device_memory_->resize(size * sizeof(T));
    size_ = size;
  }

  // Copies all elements from the given host buffer to the device
  void copyToDevice(const T* host_source) {
    device_memory_->copyToDevice(static_cast<const void*>(host_source));
  }
  // Copies all elements from the device to the given host buffer
  void copyToHost(T* host_target) {
    device_memory_->copyToHost(static_cast<void*>(host_target));
  }

 private:
  size_t size_;
  std::unique_ptr<RawCudaArray> device_memory_;
};

template <typename T>
struct BufferTraits<CudaArray<T>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = typename CudaArray<T>::mutable_view_t;
  using buffer_const_view_t = typename CudaArray<T>::const_view_t;
};

}  // namespace isaac
