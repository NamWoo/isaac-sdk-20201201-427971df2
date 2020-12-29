/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "tensor.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/array/byte_array.hpp"
#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "messages/element_type.hpp"
#include "messages/tensor.capnp.h"

template <isaac::BufferStorageMode Storage>
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::UniversalTensorConstView<Storage>& universal_view) {
  using universal_tensor_const_view_t = isaac::UniversalTensorConstView<Storage>;

  // Parse element type
  const isaac::ElementType element_type = FromProto(reader.getElementType());
  if (element_type == isaac::ElementType::kUnknown) {
    LOG_ERROR("Unknown element type");
    return false;
  }

  // Parse dimensions
  auto proto_sizes = reader.getSizes();
  typename universal_tensor_const_view_t::dimensions_t sizes(proto_sizes.size());
  typename universal_tensor_const_view_t::index_t expected_element_count = 1;
  for (size_t i = 0; i < proto_sizes.size(); i++) {
    sizes[i] = proto_sizes[i];
    expected_element_count *= sizes[i];
  }

  // Get buffer
  const uint32_t buffer_index = reader.getDataBufferIndex();
  if (buffer_index >= buffers.size()) {
    LOG_ERROR("Buffer index %u out of range (%zu): ", buffer_index, buffers.size());
    return false;
  }
  const auto source_buffer_view = buffers[buffer_index].const_view<
      typename universal_tensor_const_view_t::buffer_const_view_t>();

  // Check buffer length
  const size_t size_provided = source_buffer_view.size();
  const size_t size_expected = expected_element_count * ElementTypeByteCount(element_type);
  if (size_provided != size_expected) {
    LOG_ERROR("Tensor data size does not match. Proto provides %zu bytes while tensor expected "
              "%zu bytes.", size_provided, size_expected);
    return false;
  }

  universal_view = universal_tensor_const_view_t(element_type, sizes, source_buffer_view);
  return true;
}

template
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::CpuUniversalTensorConstView& universal_view);
template
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::CudaUniversalTensorConstView& universal_view);
