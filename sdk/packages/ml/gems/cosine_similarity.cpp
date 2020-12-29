/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cosine_similarity.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/tensor/utils.hpp"

namespace isaac {
namespace ml {

CosineSimilarity::CosineSimilarity(TensorConstView2f codebook_host) {
  initialize(codebook_host);
}

CosineSimilarity::CosineSimilarity(TensorView2f codebook_host, bool normalize) {
  const int num_samples = codebook_host.dimensions()[0];
  const int num_channels = codebook_host.dimensions()[1];
  if (normalize) {
    Eigen::Map<MatrixXf>(
        codebook_host.element_wise_begin(), num_channels, num_samples).colwise().normalize();
  }
  initialize(codebook_host.const_view());
}

void CosineSimilarity::initialize(TensorConstView2f codebook_host) {
  codebook_.resize(codebook_host.dimensions());
  temp_cuda_vector_.resize(codebook_host.dimensions()[0]);
  temp_host_vector_.resize(codebook_host.dimensions()[0]);
  Copy(codebook_host, codebook_);
}

std::optional<int> CosineSimilarity::nearestIndex(CudaTensorConstView1f vector_cuda,
                                                  cublasContext* handle) {
  // Multiply codebook matrix by vector (i.e. dot product of each row by the vector)
  float alpha = 1.f, beta = 0.f;
  cublasStatus_t cublas_result;
  cublas_result = cublasSgemv(handle, CUBLAS_OP_T, getChannelCount(), getSampleCount(), &alpha,
                              codebook_.element_wise_begin(), getChannelCount(),
                              vector_cuda.element_wise_begin(), 1 /*incx*/, &beta,
                              temp_cuda_vector_.element_wise_begin(), 1 /*incy*/);
  if (cublas_result != CUBLAS_STATUS_SUCCESS) {
    return std::nullopt;
  }

  // Find the index of the row with the max value (greatest cosine similarity)
  int index = 0;
  cublas_result = cublasIsamax(handle, getSampleCount(), temp_cuda_vector_.element_wise_begin(),
                               1 /*incx*/, &index);
  if (cublas_result != CUBLAS_STATUS_SUCCESS) {
    return std::nullopt;
  }

  // The result of cublasIsamax is 1-indexed, so subtract 1 to make it 0-indexed
  return index - 1;
}

void CosineSimilarity::nearestKIndices(CudaTensorConstView1f vector_cuda, int num_vectors,
                                       cublasContext* handle, std::vector<FeatureScore>& best_k) {
  best_k.clear();
  // In case only one vector is required, we can use the nearestIndex function which is faster.
  if (num_vectors == 1) {
    if (auto index = nearestIndex(vector_cuda, handle)) {
      // FIXME(bbutin) Provide the correct correlation.
      best_k.push_back({1.0f, *index});
    }
    return;
  }
  // Multiply codebook matrix by vector (i.e. dot product of each row by the vector)
  float alpha = 1.f, beta = 0.f;
  cublasStatus_t cublas_result;
  cublas_result = cublasSgemv(handle, CUBLAS_OP_T, getChannelCount(), getSampleCount(), &alpha,
                              codebook_.element_wise_begin(), getChannelCount(),
                              vector_cuda.element_wise_begin(), 1 /*incx*/, &beta,
                              temp_cuda_vector_.element_wise_begin(), 1 /*incy*/);
  if (cublas_result != CUBLAS_STATUS_SUCCESS) {
    return;
  }

  Copy(temp_cuda_vector_, temp_host_vector_);

  for (int idx = 0; idx < temp_host_vector_.element_count(); idx++) {
    best_k.push_back({temp_host_vector_(idx), idx});
  }

  if (best_k.size() > static_cast<size_t>(num_vectors)) {
    std::partial_sort(best_k.begin(), best_k.begin() + num_vectors, best_k.end());
    best_k.resize(num_vectors);
  } else {
    std::sort(best_k.begin(), best_k.end());
  }
}

}  // namespace ml
}  // namespace isaac
