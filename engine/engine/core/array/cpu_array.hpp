/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdlib>

#include "engine/core/array/cpu_array_view.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

// A block of memory storing a certain number of elements of the given type. This type uses
// malloc and free to handle memory allocation. This is a non-copyable object and it needs to be
// moved or used via a unique_ptr. The contents of the buffer can always be modified. Most functions
// should not operate on this type, but use CpuArrayView or ConstCpuArrayView instead.
template <typename T>
class CpuArray {
 public:
  // Type of values. Uses naming-convention from STD for compatibility.
  using value_type = T;

  using mutable_view_t = CpuArrayView<T>;
  using const_view_t = CpuArrayView<const T>;

  // Creates a CpuArray from an existing buffer pointer. The CpuArray will take ownership and
  // free the buffer when the time has come.
  // WARNING: This is dangerous as the wrong allocator might be called.
  static CpuArray Own(T* pointer, size_t size) {
    return CpuArray(pointer, size);
  }

  // Default constructor creates a "dead" and empty object
  CpuArray() : data_(nullptr), size_(0) {}

  // This constructor allocates memory for the given number of elements of type T.
  CpuArray(size_t size) {
    size_ = size;
    allocate();
  }

  // Disable copy
  CpuArray(const CpuArray&) = delete;
  CpuArray& operator=(const CpuArray&) = delete;

  // Move constructor
  CpuArray(CpuArray&& other) {
    data_ = other.begin();
    size_ = other.size();
    other.data_ = nullptr;
    other.size_ = 0;
  }
  // Move assignment operator
  CpuArray& operator=(CpuArray&& other) {
    free();
    data_ = other.begin();
    size_ = other.size();
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  // The destructor uses the free functor to clean up
  ~CpuArray() {
    free();
  }

  // Returns true if the buffer contains any elements
  bool empty() const { return size_ == 0; }
  // The number of elements stored in this buffer
  size_t size() const { return size_; }
  // Pointer to the first element
  const T* begin() const { return data_; }
  T* begin() { return data_; }
  // Pointer behind the last element
  const T* end() const { return data_ + size_; }
  T* end() { return data_ + size_; }

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

  // Allow conversion from owning to view / const view
  operator mutable_view_t() {
    return mutable_view_t{begin(), size()};
  }
  // Allow conversion from owning or mutable view to const view
  operator const_view_t() const {
    return const_view_t{begin(), size()};
  }

  // Casts to a buffer object of different type and transfers ownership to the newly created object.
  // The object cast from is invalidated. The source and target element type must be compatible,
  // i.e. size() * sizeof(T) must be a multiple of sizeof(S).
  template <typename S>
  CpuArray<S> move_reinterpret() {
    // Cast to new type using view
    auto this_view_s = this->view().template reinterpret<S>();
    // invalidate this object
    data_ = nullptr;
    size_ = 0;
    // move out result object
    return CpuArray<S>(this_view_s.begin(), this_view_s.size());
  }

  // Resizes the memory block to hold the given number of elements. This operation might invalidate
  // the existing memory. It will not copy existing elements to the newly allocated memory.
  void resize(size_t new_size) {
    if (size() == new_size) {
      return;
    }
    free();
    size_ = new_size;
    allocate();
  }

 private:
  // Allow versions of Buffers with different element type to act as friend. Necessary for example
  // for `move_reinterpret`.
  template <typename S>
  friend class CpuArray;

  // Creates a buffer with the given data
  CpuArray(T* data, size_t size)
      : data_(data), size_(size) {}

  // Allocates enough bytes to hold the number of elements
  void allocate() {
    data_ = reinterpret_cast<T*>(std::malloc(sizeof(T) * size_));
    ASSERT(data_, "Could not allocate memory");
  }

  // Frees all allocated resources if necessary.
  void free() {
    std::free(begin());
  }

  T* data_;
  size_t size_;
};

template <typename T>
struct BufferTraits<CpuArray<T>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = typename CpuArray<T>::mutable_view_t;
  using buffer_const_view_t = typename CpuArray<std::add_const_t<T>>::const_view_t;
};

}  // namespace isaac
