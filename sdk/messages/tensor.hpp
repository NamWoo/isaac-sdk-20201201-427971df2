/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/array/byte_array.hpp"
#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "messages/element_type.hpp"
#include "messages/tensor.capnp.h"

// Parses a tensor from a message. This version parses tensors of any element type and rank.
template <isaac::BufferStorageMode Storage>
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::UniversalTensorConstView<Storage>& universal_view);

// Creates a tensor from a proto. Will print errors and return false if the tensor type is not
// compatible with the proto.
template <typename K, typename Dimensions, typename BufferType>
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::TensorBase<K, Dimensions, BufferType>& tensor_view) {
  // This function is only allowed for const views
  static_assert(
      !isaac::BufferTraits<BufferType>::kIsMutable && !isaac::BufferTraits<BufferType>::kIsOwning,
      "Invalid buffer type. Buffer cannot own or mutate data");

  // Get the storage mode and make sure it is either host or device
  constexpr isaac::BufferStorageMode kStorageMode = isaac::BufferTraits<BufferType>::kStorageMode;
  static_assert(kStorageMode == isaac::BufferStorageMode::Host ||
                kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");

  // Parse the message into a dynamic tensor
  isaac::UniversalTensorConstView<kStorageMode> universal_view;
  if (!FromProto(reader, buffers, universal_view)) {
    return false;
  }

  // Try to get view of the requested element type and rank
  const auto maybe = universal_view.template tryGet<isaac::TensorBase<K, Dimensions, BufferType>>();
  if (!maybe) {
    LOG_ERROR("Received tensor does not have rank or element type as requested. "
              "Expected: (%s, %d). Actual: (%s, %d).",
              ElementTypeCStr(isaac::GetElementType<K>()), tensor_view.kRank,
              ElementTypeCStr(universal_view.element_type()), universal_view.rank());
    return false;
  }
  tensor_view = *maybe;
  return true;
}

// Writes an tensor to a TensorProto
template <typename K, typename Dimensions, typename BufferType>
void ToProto(isaac::TensorBase<K, Dimensions, BufferType> tensor,
             ::TensorProto::Builder builder, std::vector<isaac::SharedBuffer>& buffers) {
  static_assert(isaac::BufferTraits<BufferType>::kIsOwning, "Must own the data to send it.");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Set the elemenet ntype
  builder.setElementType(ToProto(isaac::GetElementType<K>()));
  // Set the tensor sizes
  const auto sizes = tensor.dimensions();
  auto dimensions = builder.initSizes(sizes.size());
  for (size_t i = 0; i < dimensions.size(); i++) {
    dimensions.set(i, sizes[i]);
  }
  // Add the tensor buffer to the list of buffers and make a copy of the tensor.
  builder.setDataBufferIndex(buffers.size());
  // TODO(bbutin) The scanline stride field should be removed from the message.
  builder.setScanlineStride(0);
  buffers.emplace_back(std::move(tensor.data()));
}
