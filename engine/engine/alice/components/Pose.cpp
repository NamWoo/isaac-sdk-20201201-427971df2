/*
Copyright (c) 2018, 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Pose.hpp"

#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

Pose3d Pose::get(const FrameId& lhs, const FrameId& rhs, double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  ASSERT(maybe, "Could not get the transformation %s_T_%s.", lhs.c_str(), rhs.c_str());
  return *maybe;
}

Pose2d Pose::getPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const {
  return get(lhs, rhs, time).toPose2XY();
}

std::pair<Pose3d, double> Pose::getLatest(const FrameId& lhs, const FrameId& rhs) const {
  return pose_tree()->getLatest(lhs, rhs);
}

std::pair<Pose2d, double> Pose::getLatestPose2XY(const FrameId& lhs, const FrameId& rhs) const {
  return pose_tree()->getLatestPose2XY(lhs, rhs);
}

std::optional<Pose3d> Pose::tryGet(const FrameId& lhs, const FrameId& rhs, double time) const {
  return pose_tree()->tryGet(lhs, rhs, time);
}

std::optional<Pose2d> Pose::tryGetPose2XY(const FrameId& lhs, const FrameId& rhs,
                                          double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  if (maybe) {
    return maybe->toPose2XY();
  } else {
    return std::nullopt;
  }
}

std::optional<std::pair<Pose3d, double>> Pose::tryGetLatest(const FrameId& lhs,
                                                            const FrameId& rhs) const {
  return pose_tree()->tryGetLatest(lhs, rhs);
}

std::optional<std::pair<Pose2d, double>> Pose::tryGetLatestPose2XY(const FrameId& lhs,
                                                                   const FrameId& rhs) const {
  return pose_tree()->tryGetLatestPose2XY(lhs, rhs);
}

bool Pose::trySet(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time) {
  return pose_tree()->trySet(lhs, rhs, lhs_T_rhs, time);
}

bool Pose::trySet(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time) {
  return trySet(lhs, rhs, Pose3d::FromPose2XY(lhs_T_rhs), time);
}

void Pose::set(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time) {
  pose_tree()->set(lhs, rhs, lhs_T_rhs, time);
}

void Pose::set(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time) {
  set(lhs, rhs, Pose3d::FromPose2XY(lhs_T_rhs), time);
}

bool Pose::removeFrame(const FrameId& frame, double stamp) {
  return pose_tree()->removeFrame(frame, stamp);
}

bool Pose::removeEdge(const FrameId& lhs, const FrameId& rhs, double stamp) {
  return pose_tree()->removeEdge(lhs, rhs, stamp);
}

PoseTree* Pose::pose_tree() const {
  // FIXME(dweikersdorf) We need to get the pointer of the pose tree lazily as it is stored in a
  //                     node which also has a Pose component which is initialized before the pose
  //                     tree. This will be adressed in the future by separating the construction
  //                     from the initialization phase.
  if (pose_tree_ == nullptr) {
    pose_tree_ = node()->app()->backend()->pose_tree();
    ASSERT(pose_tree_ != nullptr, "pose tree not set");
  }
  return pose_tree_;
}

}  // namespace alice
}  // namespace isaac
