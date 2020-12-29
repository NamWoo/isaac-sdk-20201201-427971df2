/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/tensor/utils.hpp"

#include "gtest/gtest.h"

namespace isaac {

template <typename K>
void TestTensorCopyImpl(const Vector3<int>& dimensions) {
  Tensor3<K> host_1(dimensions);
  ASSERT_EQ(host_1.dimensions(), dimensions);

  for (int i = 0; i < dimensions[0]; i++) {
    for (int j = 0; j < dimensions[1]; j++) {
      for (int k = 0; k < dimensions[2]; k++) {
        host_1(i, j, k) = static_cast<K>(i * 1'000'000 + j * 1'000 + k);
      }
    }
  }

  CudaTensor3<K> cuda_1(dimensions);
  ASSERT_EQ(cuda_1.dimensions(), dimensions);
  Copy(host_1, cuda_1);

  CudaTensor3<K> cuda_2(dimensions);
  ASSERT_EQ(cuda_2.dimensions(), dimensions);
  Copy(cuda_1, cuda_2);

  Tensor3<K> host_2(dimensions);
  ASSERT_EQ(host_2.dimensions(), dimensions);
  Copy(cuda_2, host_2);

  for (int i = 0; i < dimensions[0]; i++) {
    for (int j = 0; j < dimensions[1]; j++) {
      for (int k = 0; k < dimensions[2]; k++) {
        ASSERT_EQ(host_2(i, j, k), host_1(i, j, k));
      }
    }
  }
}

TEST(Tensor, Copy) {
  // Create a dummy 3D tensor on host. Then: 1) copy from host to device, 2) copy from device to
  // device, 3) copy back to host, 4) compare that the data is identical to the original data.

  std::vector<Vector3<int>> dimensions_array{
    {3, 720, 1280},
    {720, 1280, 3},
    {640, 139, 5},
    {17, 19, 13},
    {1, 1, 1},
    {1000, 100, 1000}
  };

  for (const auto& dimensions : dimensions_array) {
    TestTensorCopyImpl<unsigned char>(dimensions);
    TestTensorCopyImpl<int>(dimensions);
    TestTensorCopyImpl<float>(dimensions);
    TestTensorCopyImpl<double>(dimensions);
  }
}

}  // namespace isaac
