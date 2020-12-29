/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sop_transform.hpp"

#include <utility>

namespace isaac {
namespace sight {

const Json& ToJson(const SopTransform& transform) {
  return transform.json_;
}

Json ToJson(SopTransform&& transform) {
  return std::move(transform.json_);
}

}  // namespace sight
}  // namespace isaac
