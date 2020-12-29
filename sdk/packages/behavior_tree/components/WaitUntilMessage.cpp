/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/behavior_tree/components/WaitUntilMessage.hpp"

#include <memory>

namespace isaac {
namespace behavior_tree {

void WaitUntilMessage::start() {
  // Accept any type of message
  hook_ = std::make_unique<alice::RxMessageHook>(this, get_channel_name());
  connectHooks();
  tickOnMessage(*hook_);
}

void WaitUntilMessage::tick() {
  reportSuccess();
}

}  // namespace behavior_tree
}  // namespace isaac
