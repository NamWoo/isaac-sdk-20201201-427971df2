/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// A non-template tensor which can hold a const view on a tensor for a fixed storage order, but
// which is not templated on rank or element type.
//
// The Tensor class is a template type which can only be used if the rank and the element type of
// the tensor is known. However some functions can be written without knowning the actual type of
// the tensor. For example a deserialization function does not know yet what kind of tensor will be
// deserialized and the subsequent user might be able work with different types of tensors.
//
// Currently only const view tensors with a fixed storage mode are supported. This covers the
// primary use case of reading tensors from messages.
template <BufferStorageMode Storage>
class UniversalTensorConstView {
 public:
  // Type used for dimension indices
  using index_t = int;

  // Type for storing the array of dimensions of the tensor
  using dimensions_t = VectorX<index_t>;

  // Type for storing the buffer data. This is a const view with fixed storage order.
  using buffer_const_view_t = detail::BufferBase<
      detail::TaggedPointer<const byte, std::integral_constant<BufferStorageMode, Storage>>>;

  UniversalTensorConstView() : element_type_(ElementType::kUnknown) {}
  UniversalTensorConstView(ElementType element_type, const dimensions_t dimensions,
                           buffer_const_view_t buffer)
      : element_type_(element_type), dimensions_(dimensions), buffer_(buffer) {}

  template <typename K, typename Dimensions>
  UniversalTensorConstView(TensorBase<K, Dimensions, buffer_const_view_t> view)
      : element_type_(GetElementType<K>()), dimensions_(view.dimensions()), buffer_(view.data()) {}

  // The element type of the tensor
  ElementType element_type() const { return element_type_; }

  // The rank of the tensor. This is also the length of the dimensions.
  index_t rank() const { return dimensions_.size(); }

  // The dimensions of the tensor. Most significant dimension comes first in the array.
  const dimensions_t& dimensions() const { return dimensions_; }

  // The element type of the tensor
  const buffer_const_view_t& buffer() const { return buffer_; }

  // Checks if the tensor is compatible with the desired rank.
  // A tensor of lesser rank can always be interpreted as a tensor of higher rank.
  // A tensor which has leading dimensions of 1 can be interpreted as a tensor of lesser rank.
  // Trailing ones are ignored in this computation.
  // For example, a tensor of dimensions (1, 1, 3, 4) can have a compatible rank that is greater
  // than or equal to 2.
  // The target dimensions for a compatible rank of 5 would be (1, 1, 1, 3, 4) and the target
  // dimensions for a compatible rank of 2 would be (3, 4).
  // Please refer getCompatibleDimensions for more information.
  template <index_t Rank>
  bool isRankCompatible() const {
    // Check for boundary cases
    if (Rank <= 0) return false;
    if (Rank >= dimensions_.size()) return true;
    index_t leading_ones = dimensions_.size();
    // Count the numder of leading ones. Return true if the number of remaining dimensions are less
    // than or equal to the required rank.
    for (index_t i = 0; i < dimensions_.size(); i++) {
      if (dimensions_[i] != 1) {
        leading_ones = i;
        break;
      }
    }
    return dimensions_.size() <= Rank + leading_ones;
  }

  // Gets dimensions of the desired rank. See isRankCompatible for more information. This function
  // will not check if the rank is actually compatible.
  template <index_t Rank>
  Vector<index_t, Rank> getCompatibleDimensions() const {
    Vector<index_t, Rank> result;
    if (Rank <= dimensions_.size()) {
      // Example: dim = (1, 1, 3, 4), rank = 2 => target = (3, 4)
      result = dimensions_.tail<Rank>();
    } else {
      // Example: dim = (3, 4), rank = 3 => target = (1, 3, 4)
      result = Vector<index_t, Rank>::Constant(1);
      result.tail(dimensions_.size()) = dimensions_;
    }
    return result;
  }

  // Checks if the tensor has a specific rank and element type.
  template <typename K, index_t Rank>
  bool isOfType() const {
    return GetElementType<K>() == element_type() && isRankCompatible<Rank>();
  }

  // Tries to get a tensor with specified rank and element type. Returns nullopt if the stored
  // tensor has a different type.
  template <typename K, typename Dimensions>
  std::optional<TensorBase<K, Dimensions, buffer_const_view_t>> tryGet() const {
    constexpr index_t kRank = TensorBase<K, Dimensions, buffer_const_view_t>::kRank;
    if (!isOfType<K, kRank>()) {
      return std::nullopt;
    }
    // Compute target dimensions - also see isOfType function.
    Vector<index_t, kRank> target_dimensions = getCompatibleDimensions<kRank>();
    return TensorBase<K, Dimensions, buffer_const_view_t>(buffer(), target_dimensions);
  }

  // Gets a tensor with specified rank and element type. Asserts if the stored tensor has a
  // different type.
  template <typename K, typename Dimensions>
  TensorBase<K, Dimensions, buffer_const_view_t> get() const {
    constexpr index_t kRank = TensorBase<K, Dimensions, buffer_const_view_t>::kRank;
    auto maybe = tryGet<K, Dimensions>();
    ASSERT(maybe, "Tensor does not have expected type. Expected: (%s, %d). Actual: (%s, %d).",
           ElementTypeCStr(GetElementType<K>()), kRank,
           ElementTypeCStr(element_type_), dimensions_.size());
    return *maybe;
  }

  template <typename Tensor>
  bool isOfType() const {
    return isOfType<typename Tensor::element_t, Tensor::kRank>();
  }

  template <typename Tensor>
  std::optional<TensorBase<typename Tensor::element_t, typename Tensor::Dimensions,
                           buffer_const_view_t>>
  tryGet() const {
    return tryGet<typename Tensor::element_t, typename Tensor::Dimensions>();
  }

  template <typename Tensor>
  TensorBase<typename Tensor::element_t, typename Tensor::Dimensions, buffer_const_view_t>
  get() const {
    return get<typename Tensor::element_t, typename Tensor::Dimensions>();
  }

 private:
  ElementType element_type_;
  dimensions_t dimensions_;
  buffer_const_view_t buffer_;
};

// A dynamic tensor const view using host storage
using CpuUniversalTensorConstView = UniversalTensorConstView<BufferStorageMode::Host>;

// A dynamic tensor const view using device storage
using CudaUniversalTensorConstView = UniversalTensorConstView<BufferStorageMode::Cuda>;

}  // namespace isaac
