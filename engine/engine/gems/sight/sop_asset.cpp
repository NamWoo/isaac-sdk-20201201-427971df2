/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual propertyfrined
frined
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sop_asset.hpp"

#include <utility>

namespace isaac {
namespace sight {

const nlohmann::json& ToJson(const SopAsset& asset) {
  return asset.json_;
}

nlohmann::json ToJson(SopAsset&& asset) {
  return std::move(asset.json_);
}

}  // namespace sight
}  // namespace isaac
