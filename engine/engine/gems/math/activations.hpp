/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>

namespace isaac {

// Standard Logistic sigmoid function given by the equation
// f(x) = L / (1 + exp(-k*(x-x0)))
// where k = 1, x0 = 0 and L = 1
// input x is in the range (-inf, inf)
template <typename K>
K Sigmoid(const K x) {
  return K(1) / (K(1) + std::exp(-x));
}

}  // namespace isaac
