/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "component.hpp"

#include <string>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/event_manager.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

Component::~Component() {
  ASSERT(hooks_.empty(), "Hooks should be removed before destructing a component");
}

std::string Component::full_name() const {
  return node()->name() + "/" + name();
}

void Component::removeHook(Hook* hook) {
  const auto position = std::find(hooks_.begin(), hooks_.end(), hook);
  ASSERT(position != hooks_.end(), "Could not find the hook to remove");
  hooks_.erase(position);
}

void Component::connectHooks() {
  for (Hook* hook : hooks_) {
    hook->connect();
  }
}

}  // namespace alice
}  // namespace isaac
