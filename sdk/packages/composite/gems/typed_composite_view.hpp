/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <numeric>

#include "engine/core/assert.hpp"

namespace isaac {
namespace composite {

// This macro is used to setup a custom type for compatibility with composites. It can be used to
// define a scalar quantity which maps into a composite. Multiple scalar quantities can be used
// to represent a multi-dimensional quantity like Vector3 or Pose2.
// Example:
//   template <template<int> class Base>
//   struct Foo : public Base<2> {
//     ISAAC_COMPOSITE_SCALAR(0, hello);
//     ISAAC_COMPOSITE_SCALAR(1, world);
//   };
#define ISAAC_COMPOSITE_SCALAR(INDEX, NAME) \
  const auto NAME() const { return this->operator[](INDEX); } \
  auto& NAME() { return this->operator[](INDEX); } \
  static constexpr int kI_##NAME = INDEX;

namespace details {

// A helper type to implement CompositeStateDimension.
template <int N>
struct TypedCompositeStateDimensionImpl {
  // The dimension of the state.
  static constexpr int kDimension = N;
};

}  // namespace details

// A helper template which can be used to get the dimension of a custom state at compile time.
template <template<template<int> class> class State>
constexpr int TypedCompositeStateDimension =
    State<details::TypedCompositeStateDimensionImpl>::kDimension;

// A C++ type to access data stored in a composite. This type offers type-safe access to the
// composite data. It is a view and does not own its memory.
template <typename K, template<template<int> class> class State>
struct TypedCompositeView {
 public:
  // The scalar type used for elements
  using scalar_t = K;
  // The dimension of the state.
  static constexpr int kDimension = TypedCompositeStateDimension<State>;

  // Type used to store element offsets
  using offsets_t = std::array<int, kDimension>;

  // A base class which is used to represent a mapping of a custom state type into a composite. A
  // list of offsets is used to remap access by index to the correct location in the underlying
  // data pointer.
  template <int Dummy>
  class TypedCompositeStateView {
   public:
    TypedCompositeStateView() = default;

    TypedCompositeStateView(const TypedCompositeStateView&) = delete;
    TypedCompositeStateView& operator=(const TypedCompositeStateView&) = delete;

    TypedCompositeStateView(TypedCompositeStateView&&) = default;
    TypedCompositeStateView& operator=(TypedCompositeStateView&&) = default;

    // Access to the state element at position `index`. `index` must be in the range {0, ..., N-1}.
    scalar_t operator[](int index) const { return data[(*offsets)[index]]; }
    scalar_t& operator[](int index) { return data[(*offsets)[index]]; }

    // Pointer to the composite view which created the state view
    scalar_t* data;
    // Pointer to offsets used
    const offsets_t* offsets;
  };

  // The state view type used for this composite. It is based on the given State type to allow
  // type-safe access.
  using state_view_t = State<TypedCompositeStateView>;

  // Constructs an empty composite view.
  TypedCompositeView()
  : data_(nullptr), size_(0), stride_(0), offsets_() {}

  // Constructs a state view based on the given memory `data`. `size` is the number of state vectors
  // which should be accessed by the view. States are stored continuously using an element stride
  // (not byte stride) of `stride`. The given offsets are used to access the desired elements
  // in the composite.
  TypedCompositeView(scalar_t* data, int size, int stride, const offsets_t& offsets)
  : data_(data), size_(size), stride_(stride), offsets_(offsets) {
    ASSERT(data_ != nullptr || size_ == 0, "Pointer must not be null for non-empty composite.");
    for (int offset : offsets) {
      ASSERT(0 <= offset && offset < stride, "Invalid offset: 0 <= %d < %d", offset, stride);
    }
  }

  // Creates from tensor. Use template typename instead of including tensor header here to avoid
  // adding dependency to users of TypedCompositeView that do not need tensor.
  template <typename T>
  static TypedCompositeView FromTensor(T& tensor, const std::array<int, kDimension>& offsets) {
    ASSERT(tensor.kRank == 2,
           "Tensor has rank %d, but it needs to be of rank 2 for TypedCompositeView.",
           tensor.kRank);
    const int size = tensor.dimensions()[0];
    const int stride = tensor.dimensions()[1];
    return TypedCompositeView(tensor.element_wise_begin(), size, stride, offsets);
  }
  // Creates from tensor with access to all elements, i.e., offsets are from 0 to stride-1.
  template <typename T>
  static TypedCompositeView FromTensor(T& tensor) {
    std::array<int, kDimension> offsets;
    std::iota(offsets.begin(), offsets.end(), 0);
    return FromTensor(tensor, offsets);
  }

  // Total number of elements in the composite view.
  int size() const {
    return size_;
  }

  // Gives access to the state at position `index`. `index` must be in the range [0, size()[.
  state_view_t operator[](int index) const {
    state_view_t result;
    result.data = data_ + stride_ * index;
    result.offsets = &offsets_;
    return result;
  }

 private:
  // A pointer to the underlying memory which holds state elements for all states.
  scalar_t* data_;
  // The total number of states which can be accessed.
  int size_;
  // The total number of elements which are stored per state. This can be larger than the number
  // of state elements which are accessed by State.
  int stride_;
  // Offsets which define the position of the accessed state elements in the given memory.
  offsets_t offsets_;
};

}  // namespace composite
}  // namespace isaac
