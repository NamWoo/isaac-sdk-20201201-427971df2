/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/tensor/tensor.hpp"
#include "packages/engine_gems/state/state.hpp"

namespace isaac {

// Copies TensorView1 into State
template<typename K, int N>
bool TensorView1ToState(TensorView<K, 1> tensor, state::State<K, N>& state) {
  if (static_cast<int>(tensor.dimensions()[0]) != state.elements.size()) {
    // If size of tensor and state do not match
    LOG_ERROR("Size mismatch between Tensor and State variables !");
    return false;
  }
  for (int i = 0; i < tensor.dimensions()[0]; i++) {
    state.elements[i] = tensor(i);
  }
  return true;
}

// Copies TensorConstView1 into State
template<typename K, int N>
bool TensorConstView1ToState(TensorConstView<K, 1> tensor, state::State<K, N>& state) {
  if (static_cast<int>(tensor.dimensions()[0]) != state.elements.size()) {
    // If size of tensor and state do not match
    LOG_ERROR("Size mismatch between Tensor and State variables !");
    return false;
  }
  for (int i = 0; i < tensor.dimensions()[0]; i++) {
    state.elements[i] = tensor(i);
  }
  return true;
}

}  // namespace isaac
