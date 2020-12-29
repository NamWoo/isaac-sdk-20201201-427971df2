/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "image.hpp"

#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>

#include "engine/core/array/byte_array.hpp"
#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "messages/element_type.hpp"
#include "messages/tensor.capnp.h"

template <isaac::BufferStorageMode Storage>
bool FromProto(::ImageProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::UniversalTensorConstView<Storage>& universal_view) {
  using universal_tensor_const_view_t = isaac::UniversalTensorConstView<Storage>;

  // Parse element type
  const isaac::ElementType element_type = FromProto(reader.getElementType());
  if (element_type == isaac::ElementType::kUnknown) {
    LOG_ERROR("Unknown element type");
    return false;
  }

  // Parse dimensions
  const int rows = reader.getRows();
  const int cols = reader.getCols();
  const int channels = reader.getChannels();
  isaac::VectorX<int> dimensions;
  int expected_element_count;
  if (channels == 1) {
    dimensions = isaac::Vector2<int>(rows, cols);
    expected_element_count = rows * cols;
  } else {
    dimensions = isaac::Vector3<int>(rows, cols, channels);
    expected_element_count = rows * cols * channels;
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

  universal_view = universal_tensor_const_view_t(element_type, dimensions, source_buffer_view);
  return true;
}

template
bool FromProto(::ImageProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::CpuUniversalTensorConstView& universal_view);
template
bool FromProto(::ImageProto::Reader reader, const std::vector<isaac::SharedBuffer>& buffers,
               isaac::CudaUniversalTensorConstView& universal_view);

bool CopyProto(::ImageProto::Reader source, const std::vector<isaac::SharedBuffer>& source_buffers,
               ::ImageProto::Builder target, std::vector<isaac::SharedBuffer>& target_buffers) {
  // Create a copy of the image buffer
  const size_t source_buffer_index = source.getDataBufferIndex();
  if (source_buffer_index >= source_buffers.size()) {
    return false;
  }
  const auto& src_buffer = source_buffers[source_buffer_index];
  ::isaac::CpuBuffer dst_buffer(src_buffer.host_buffer().size());
  std::memcpy(dst_buffer.begin(), src_buffer.host_buffer().begin(), src_buffer.size());

  target.setElementType(source.getElementType());
  target.setRows(source.getRows());
  target.setCols(source.getCols());
  target.setChannels(source.getChannels());

  // Add target buffer and choose set correct buffer index
  target.setDataBufferIndex(target_buffers.size());
  target_buffers.emplace_back(::isaac::SharedBuffer(std::move(dst_buffer)));

  return true;
}
