/*
Copyright (c) 2018, 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "PoseTree.hpp"

#include <string>
#include <utility>

namespace isaac {
namespace alice {

Pose3d PoseTree::get(const FrameId& lhs, const FrameId& rhs, double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  ASSERT(maybe, "Could not get the transformation %s_T_%s.", lhs.c_str(), rhs.c_str());
  return *maybe;
}

Pose2d PoseTree::getPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const {
  return get(lhs, rhs, time).toPose2XY();
}

std::pair<Pose3d, double> PoseTree::getLatest(const FrameId& lhs, const FrameId& rhs) const {
  auto maybe = tryGetLatest(lhs, rhs);
  ASSERT(maybe, "Could not get the latest transformation %s_T_%s.", lhs.c_str(), rhs.c_str());
  return *maybe;
}

std::pair<Pose2d, double> PoseTree::getLatestPose2XY(const FrameId& lhs, const FrameId& rhs) const {
  const auto ret_3d = getLatest(lhs, rhs);
  return {ret_3d.first.toPose2XY(), ret_3d.second};
}

std::optional<Pose3d> PoseTree::tryGet(const FrameId& lhs, const FrameId& rhs, double time) const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.get(lhs, rhs, time);
}

std::optional<Pose2d> PoseTree::tryGetPose2XY(const FrameId& lhs, const FrameId& rhs,
                                              double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  if (maybe) {
    return maybe->toPose2XY();
  } else {
    return std::nullopt;
  }
}

std::optional<std::pair<Pose3d, double>> PoseTree::tryGetLatest(const FrameId& lhs,
                                                                const FrameId& rhs) const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.getLatest(lhs, rhs);
}

std::optional<std::pair<Pose2d, double>> PoseTree::tryGetLatestPose2XY(const FrameId& lhs,
                                                                       const FrameId& rhs) const {
  auto maybe = tryGetLatest(lhs, rhs);
  if (!maybe) return std::nullopt;
  return std::make_pair(maybe->first.toPose2XY(), maybe->second);
}

bool PoseTree::removeFrame(const FrameId& frame, double stamp) {
  std::unique_lock<std::shared_timed_mutex> pose_tree_lock(pose_tree_mutex_);
  return pose_tree_.removeFrame(frame, stamp);
}

bool PoseTree::removeEdge(const FrameId& lhs, const FrameId& rhs, double stamp) {
  std::unique_lock<std::shared_timed_mutex> pose_tree_lock(pose_tree_mutex_);
  return pose_tree_.removeEdge(lhs, rhs, stamp);
}

void PoseTree::set(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time) {
  ASSERT(trySet(lhs, rhs, lhs_T_rhs, time), "Cannot set the transformation %s_T_%s at time %lf",
         lhs.c_str(), rhs.c_str(), time);
}

void PoseTree::set(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time) {
  ASSERT(trySet(lhs, rhs, lhs_T_rhs, time), "Cannot set the transformation %s_T_%s at time %lf",
         lhs.c_str(), rhs.c_str(), time);
}

bool PoseTree::trySet(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs,
                      double time) {
  return trySet(lhs, rhs, Pose3d::FromPose2XY(lhs_T_rhs), time);
}

bool PoseTree::trySet(const std::string& lhs, const std::string& rhs, const Pose3d& lhs_T_rhs,
                      double time) {
  std::unique_lock<std::shared_timed_mutex> pose_tree_lock(pose_tree_mutex_);
  const bool result = pose_tree_.set(lhs, rhs, time, lhs_T_rhs);
  pose_tree_lock.unlock();

  std::shared_lock<std::shared_timed_mutex> callbacks_lock(callbacks_mutex_);
  for (auto& kvp : callbacks_) {
    kvp.second(lhs, rhs, time, lhs_T_rhs);
  }
  callbacks_lock.unlock();

  return result;
}

pose_tree::PoseTree PoseTree::clonePoseTree() const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_;
}

pose_tree::PoseTree PoseTree::cloneLatestPoseTree() const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.latest();
}

void PoseTree::registerForUpdates(const Component* source, UpdateFunction callback) {
  std::unique_lock<std::shared_timed_mutex> lock(callbacks_mutex_);
  callbacks_[source] = callback;
}

void PoseTree::deregisterForUpdates(const Component* source) {
  std::unique_lock<std::shared_timed_mutex> lock(callbacks_mutex_);
  callbacks_.erase(source);
}

}  // namespace alice
}  // namespace isaac
