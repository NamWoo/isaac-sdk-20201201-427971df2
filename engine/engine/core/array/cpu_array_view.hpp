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

#include "engine/core/assert.hpp"
#include "engine/core/buffers/buffer.hpp"
#include "engine/core/buffers/traits.hpp"

namespace isaac {

// A light-weight type which holds a pointer to data together with its size.
template <typename T>
class CpuArrayView {
 public:
  // Type of values. Uses naming-convention from STD for compatibility.
  using value_type = T;

  // Creates an "empty" view
  CpuArrayView() : pointer_(nullptr), size_(0) {}
  // Creates a view using the given pointer and number of elements.
  CpuArrayView(T* pointer, size_t size) : pointer_(pointer), size_(size) {}
  // Conversion constructor to create views of aligned buffers
  // TODO: There is no actual type safety in anywhere to ensure that
  // memory located in incorrect places is not passed to the wrong view type.
  template <typename Pointer>
  CpuArrayView(const detail::BufferBase<Pointer>& buffer) {
    const size_t num_bytes = buffer.size();
    ASSERT(num_bytes % sizeof(T) == 0, "Incompatible element size: size=%zd, sizeof(T)=%zd",
           num_bytes, sizeof(T));
    size_ = num_bytes / sizeof(T);
    pointer_ = reinterpret_cast<T*>(buffer.pointer().get());
  }

  // Returns true if the buffer contains any elements
  bool empty() const { return size_ == 0; }
  // The number of elements stored in this buffer
  size_t size() const { return size_; }
  // Pointer to the first element
  T* begin() const { return pointer_; }
  // Pointer behind the last element
  T* end() const { return pointer_ + size_; }

  // Casts to a different type element. The source and target element type must be compatible,
  // i.e. size() * sizeof(T) must be a multiple of sizeof(S).
  template <typename S>
  CpuArrayView<S> reinterpret() const {
    static_assert(sizeof(S) != 0, "sizeof(S) must not be 0");
    const size_t num_bytes = size() * sizeof(T);
    ASSERT(num_bytes % sizeof(S) == 0, "Incompatible element size: size=%zd, sizeof(S)=%zd",
           num_bytes, sizeof(S));
    return {reinterpret_cast<S*>(begin()), num_bytes / sizeof(S)};
  }

  // Creates a sub-view starting at element `index`
  CpuArrayView sub(size_t index) const {
    ASSERT(index <= size(), "Out of range");
    return {begin() + index, size() - index};
  }

  // Allow conversion to const view
  operator CpuArrayView<std::add_const_t<T>>() const { return {begin(), size()}; }

 private:
  // The pointer to the first element
  T* pointer_;
  // Number of elements
  size_t size_;
};

// A non-mutable view on a chunk of memory
template <typename T>
using ConstCpuArrayView = CpuArrayView<std::add_const_t<T>>;

template <typename T>
struct BufferTraits<CpuArrayView<T>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = !std::is_const<T>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CpuArrayView<T>;
  using buffer_const_view_t = ConstCpuArrayView<T>;
};

template <typename Container>
auto View(const Container& container) {
  using buffer_const_view_t = typename BufferTraits<Container>::buffer_const_view_t;
  return buffer_const_view_t{container.begin(), container.size()};
}

template <typename Container>
auto View(Container& container) {
  using buffer_view_t = typename BufferTraits<Container>::buffer_view_t;
  return buffer_view_t{container.begin(), container.size()};
}

template <typename Container>
auto ConstView(const Container& container) {
  using buffer_const_view_t = typename BufferTraits<Container>::buffer_const_view_t;
  return buffer_const_view_t{container.begin(), container.size()};
}

}  // namespace isaac
