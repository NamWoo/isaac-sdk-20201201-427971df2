/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Sight.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/sight_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

void Sight::initialize() {
  backend_ = node()->app()->backend()->sight_backend();
}

void Sight::deinitialize() {
  backend_ = nullptr;
}

}  // namespace alice
}  // namespace isaac
