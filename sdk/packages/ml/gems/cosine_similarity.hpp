/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <utility>
#include <vector>

#include "engine/core/optional.hpp"
#include "engine/core/tensor/tensor.hpp"

// Forward decl from cublas_api.h
struct cublasContext;

namespace isaac {
namespace ml {

// Helper class to get the closest vector in a codebook using the GPU.
class CosineSimilarity {
 public:
  // Helper structure to return the matching vector as well as the score
  struct FeatureScore {
    // The correlation score
    float correlation;
    // The index in the codebook
    int index;

    // Helper function to sort
    bool operator<(const FeatureScore& v) const {
      return v.correlation < correlation;
    }
  };

  // Assume the codebook_host is already normalized.
  CosineSimilarity(TensorConstView2f codebook_host);
  // If normalize is true, each column of the codebook_host will be normalized.
  CosineSimilarity(TensorView2f codebook_host, bool normalize);

  // Returns the index of the closest vector of the codebook.
  std::optional<int> nearestIndex(CudaTensorConstView1f vector_cuda, cublasContext* handle);

  // Returns the indices of the K closest vectors of the codebook.
  // If K is bigger than than the number
  void nearestKIndices(CudaTensorConstView1f vector_cuda, int num_vectors, cublasContext* handle,
                       std::vector<FeatureScore>& best_k);

  // Returns the number of rows.
  int getSampleCount() const { return codebook_.dimensions()[0]; }
  // Returns the number of columns.
  int getChannelCount() const { return codebook_.dimensions()[1]; }

 private:
  CosineSimilarity() = default;
  // Initializes the codebook on the GPU device.
  void initialize(TensorConstView2f codebook_host);

  CudaTensor2f codebook_;
  CudaTensor1f temp_cuda_vector_;
  Tensor1f temp_host_vector_;
};

}  // namespace ml
}  // namespace isaac
