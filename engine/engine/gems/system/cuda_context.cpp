/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cuda_context.hpp"

#include <cuda_runtime_api.h>

#include "engine/core/assert.hpp"

namespace isaac {
namespace cuda {

CUcontext GetOrCreateCudaContext(int device_id) {
  CUcontext cuda_context = nullptr;
  CUresult result = cuCtxGetCurrent(&cuda_context);
  // If the cuda driver has not been initialized or if there is no cuda context bound to the
  // calling thread, try initializing the cuda
  if (result == CUDA_ERROR_NOT_INITIALIZED || (result == CUDA_SUCCESS && cuda_context == nullptr)) {
    cuInit(0);
    const cudaError_t error = cudaSetDevice(static_cast<CUdevice>(device_id));
    if (error != cudaSuccess) {
      LOG_ERROR("Failed to set cuda devicee %d");
      LOG_ERROR("Error: %s", cudaGetErrorName(error));
      LOG_ERROR("Description: %s", cudaGetErrorString(error));
      return nullptr;
    }
    result = cuCtxGetCurrent(&cuda_context);
  }

  if (result != CUDA_SUCCESS) {
    ASSERT(cuda_context == nullptr, "Context is supposed to be nullptr when there is an error");
    const char* error_name = nullptr;
    const char* error_string = nullptr;
    cuGetErrorName(result, &error_name);
    cuGetErrorString(result, &error_string);
    LOG_ERROR("Failed to get current cuda context");
    if (error_name != nullptr && error_string != nullptr) {
      LOG_ERROR("Error: %s", error_name);
      LOG_ERROR("Description: %s", error_string);
    }
  }

  return cuda_context;
}

}  // namespace cuda
}  // namespace isaac
