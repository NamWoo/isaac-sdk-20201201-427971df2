/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PyCodelet.hpp"

#include <algorithm>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/component.hpp"
#include "engine/alice/message.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

using byte = uint8_t;

void PyCodelet::start() {
  pycodelet_flow_control_.start();
  pycodelet_flow_control_.cppDelegateJob("py_start");
}

void PyCodelet::tick() {
  pycodelet_flow_control_.cppDelegateJob("py_tick");
}

void PyCodelet::stop() {
  pycodelet_flow_control_.stop();
}

void PyCodelet::addRxHook(const std::string& rx_hook) {
  addRxMessageHook(rx_hook);
}

void PyCodelet::synchronizeWithTags(const std::string& tag1, const std::string& tag2) {
  synchronize(*getRxMessageHook(tag1), *getRxMessageHook(tag2));
}

void PyCodelet::tickOnMessageWithTag(const std::string& tag) {
  tickOnMessage(*getRxMessageHook(tag));
}

ConstMessageBasePtr PyCodelet::receiveMessage(const std::string& tag) {
  auto* hook = getRxMessageHook(tag);
  ASSERT(hook != nullptr, "Hook for %s does not exist", tag.c_str());
  if (!hook->available()) {
    return nullptr;
  }
  return getRxMessageHook(tag)->message();
}

std::string PyCodelet::pythonWaitForJob() {
  auto job = pycodelet_flow_control_.pythonWaitForJob();
  if (job) return *job;
  return std::string("");  // we use empty string to represent the stopping signal
}

void PyCodelet::pythonJobFinished() {
  pycodelet_flow_control_.pythonJobFinished();
}

void PyCodelet::show(const std::string& sop_json) {
  node()->sight().show(this, Json::parse(sop_json));
}

}  // namespace alice
}  // namespace isaac
