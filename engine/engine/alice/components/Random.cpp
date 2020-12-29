/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Random.hpp"

namespace isaac {
namespace alice {

void Random::start() {
  if (get_use_random_seed()) {
    rng_.seed(std::random_device()());
  } else {
    rng_.seed(get_seed());
  }
  reportSuccess();
}

}  // namespace alice
}  // namespace isaac
