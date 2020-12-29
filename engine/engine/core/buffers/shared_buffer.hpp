/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <mutex>

#include "engine/core/buffers/buffer.hpp"
#include "engine/core/optional.hpp"

namespace isaac {

// A wrapper class that supports dual read access to cpu and gpu memory buffers. The buffer starts
// its lifetime with either a CPU or GPU buffer. When the buffer data is requested in the opposite
// storage type a copy will be executed.
class SharedBuffer {
 public:
  SharedBuffer() = delete;
  SharedBuffer(CudaBuffer buffer);
  SharedBuffer(CpuBuffer buffer);
  SharedBuffer(SharedBuffer&& rhs);

  // Access the data from the cpu.
  const CpuBuffer& host_buffer() const;
  // Access the data from the gpu.
  const CudaBuffer& cuda_buffer() const;

  // Returns true if the shared buffer is stored in host memory
  bool hasHostStorage() const { return cpu_buffer_ != std::nullopt; }
  // Returns true if the shared buffer is stored in device memory
  bool hasCudaStorage() const { return cuda_buffer_ != std::nullopt; }

  // Access a buffer of a specific type
  template <typename BufferConstView>
  BufferConstView const_view() const;

  // Get the amount of memory in bytes allocated
  size_t size() const;

  // Clones the buffer object. Will clone all available storage modes.
  SharedBuffer clone() const;

 private:
  // Lock to protect against concurent memory creation.
  mutable std::mutex data_access_lock_;
  // Storage for gpu memory.
  mutable std::optional<CudaBuffer> cuda_buffer_;
  // Storage for cpu memory.
  mutable std::optional<CpuBuffer> cpu_buffer_;
};

namespace shared_buffer_details {

// A helper class to implement the SharedBuffer::const_view() member function.
template <typename BufferType>
struct SharedBufferConstView;

template <>
struct SharedBufferConstView<CpuBufferConstView> {
  CpuBufferConstView operator()(const SharedBuffer& shared) {
    return shared.host_buffer().const_view();
  }
};

template <>
struct SharedBufferConstView<CudaBufferConstView> {
  CudaBufferConstView operator()(const SharedBuffer& shared) {
    return shared.cuda_buffer().const_view();
  }
};

}  // namespace shared_buffer_details

template <typename BufferConstView>
BufferConstView SharedBuffer::const_view() const {
  return shared_buffer_details::SharedBufferConstView<BufferConstView>()(*this);
}

}  // namespace isaac
