/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cuda_array.hpp"

#include "cuda_runtime.h"  // NOLINT

namespace isaac {

RawCudaArray::RawCudaArray()
: num_bytes_(0), ptr_(nullptr) { }

RawCudaArray::RawCudaArray(size_t num_bytes) {
  cudaMalloc(reinterpret_cast<void **>(&ptr_), num_bytes);
  num_bytes_ = num_bytes;
}

RawCudaArray::~RawCudaArray() {
  if (ptr_) cudaFree(ptr_);
  num_bytes_ = 0;
}

void RawCudaArray::resize(size_t num_bytes) {
  if (num_bytes_ != num_bytes) {
    cudaFree(ptr_);
    cudaMalloc(reinterpret_cast<void **>(&ptr_), num_bytes);
    num_bytes_ = num_bytes;
  }
}

void RawCudaArray::copyToDevice(const void* host_source) {
  cudaMemcpy(ptr_, host_source, num_bytes_, cudaMemcpyHostToDevice);
}

void RawCudaArray::copyToHost(void* host_target) {
  cudaMemcpy(host_target, ptr_, num_bytes_, cudaMemcpyDeviceToHost);
}

}  // namespace isaac
