/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "hook.hpp"

#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

Hook::Hook(Component* component)
: component_(component) {
  component_->addHook(this);
}

Hook::~Hook() {
  component_->removeHook(this);
}

}  // namespace alice
}  // namespace isaac
