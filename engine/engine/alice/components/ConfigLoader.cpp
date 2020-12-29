/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ConfigLoader.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"

namespace isaac {
namespace alice {

void ConfigLoader::start() {
  // Sets the key/value synchronously
  node()->app()->backend()->config_backend()->set(get_config());
  reportSuccess();
}

}  // namespace alice
}  // namespace isaac
