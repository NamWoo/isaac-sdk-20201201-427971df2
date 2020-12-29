/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pose_hook.hpp"

#include <string>

#include "engine/alice/components/Pose.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {
namespace pose_hook_details {

PoseHookBase::PoseHookBase(Component* component, const std::string& lhs, const std::string& rhs)
: Hook(component) {
  setLhsName(lhs);
  setRhsName(rhs);
}

Pose3d PoseHookBase::getImpl(double time) const {
  auto maybe = pose_->tryGet(lhs(), rhs(), time);
  ASSERT(maybe, "Could not get the transformation %s_T_%s.", lhs().c_str(), rhs().c_str());
  return *maybe;
}

Pose3d PoseHookBase::getImpl(double time, bool& ok) const {
  auto maybe = pose_->tryGet(lhs(), rhs(), time);
  if (maybe) {
    ok = true;
    return *maybe;
  } else {
    ok = false;
    return Pose3d::Identity();
  }
}

void PoseHookBase::setImpl(const Pose3d& lhs_T_rhs, double time) {
  pose_->set(lhs(), rhs(), lhs_T_rhs, time);
}

void PoseHookBase::setImpl(const Pose3d& lhs_T_rhs, double time, bool& ok) {
  ok = pose_->trySet(lhs(), rhs(), lhs_T_rhs, time);
}

void PoseHookBase::connect() {
  pose_ = component()->node()->getComponent<Pose>();
}

}  // namespace pose_hook_details
}  // namespace alice
}  // namespace isaac
