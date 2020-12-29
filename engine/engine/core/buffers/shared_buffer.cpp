/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "shared_buffer.hpp"

#include <memory>
#include <mutex>
#include <utility>

#include "engine/core/buffers/algorithm.hpp"

namespace isaac {

SharedBuffer::SharedBuffer(CudaBuffer buffer) {
  cuda_buffer_ = std::move(buffer);
}

SharedBuffer::SharedBuffer(CpuBuffer buffer) {
  cpu_buffer_ = std::move(buffer);
}

SharedBuffer::SharedBuffer(SharedBuffer&& buffer) {
  std::lock_guard<std::mutex> lock(buffer.data_access_lock_);
  cpu_buffer_ = std::move(buffer.cpu_buffer_);
  cuda_buffer_ = std::move(buffer.cuda_buffer_);
}

const CpuBuffer& SharedBuffer::host_buffer() const {
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (!cpu_buffer_) {
    ASSERT(cuda_buffer_, "CUDA buffer not available");
    cpu_buffer_ = CpuBuffer(cuda_buffer_->size());
    CopyArrayRaw(reinterpret_cast<const byte*>(cuda_buffer_->begin()), BufferStorageMode::Cuda,
                 reinterpret_cast<byte*>(cpu_buffer_->begin()), BufferStorageMode::Host,
                 cuda_buffer_->size());
  }
  return *cpu_buffer_;
}

const CudaBuffer& SharedBuffer::cuda_buffer() const {
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (!cuda_buffer_) {
    ASSERT(cpu_buffer_, "CPU buffer not available");
    cuda_buffer_ = CudaBuffer(cpu_buffer_->size());
    CopyArrayRaw(reinterpret_cast<const byte*>(cpu_buffer_->begin()), BufferStorageMode::Host,
                 reinterpret_cast<byte*>(cuda_buffer_->begin()), BufferStorageMode::Cuda,
                 cpu_buffer_->size());
  }
  return *cuda_buffer_;
}

size_t SharedBuffer::size() const {
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (cpu_buffer_) {
    return cpu_buffer_->size();
  }
  if (cuda_buffer_) {
    return cuda_buffer_->size();
  }
  return 0;
}

SharedBuffer SharedBuffer::clone() const {
  std::unique_ptr<SharedBuffer> result;
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (cpu_buffer_) {
    result = std::make_unique<SharedBuffer>(CpuBuffer(cpu_buffer_->size()));
    if (cuda_buffer_) {
      result->cuda_buffer_ = CudaBuffer(cuda_buffer_->size());
    }
  } else if (cuda_buffer_) {
    result = std::make_unique<SharedBuffer>(CudaBuffer(cuda_buffer_->size()));
    if (cpu_buffer_) {
      result->cpu_buffer_ = CpuBuffer(cpu_buffer_->size());
    }
  }
  if (cpu_buffer_) {
    CopyArrayRaw(cpu_buffer_->begin(), BufferStorageMode::Host,
                 result->cpu_buffer_->begin(), BufferStorageMode::Host,
                 cpu_buffer_->size());
  }
  if (cuda_buffer_) {
    CopyArrayRaw(cuda_buffer_->begin(), BufferStorageMode::Cuda,
                 result->cuda_buffer_->begin(), BufferStorageMode::Cuda,
                 cuda_buffer_->size());
  }
  return std::move(*result);
}

}  // namespace isaac
