/*
Copyright (c) 2018, 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "engine/alice/component.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"

namespace isaac {
namespace alice {

// Provides convenience functions to access 3D transformations from the application wide pose tree.
//
// This component is added to every node by default and does not have to be added manually.
//
// Poses use 64-bit floating point types and are 3-dimensional. All coordinate frames for the whole
// application are stored in a single central pose tree.
//
// All functions below accept two coordinate frames: `lhs` and `rhs`. This refers to the pose
// lhs_T_rhs which is the relative transformations between these two coordinate frames. In
// particular the following equations hold:
//   p_lhs = lhs_T_rhs * p_rhs
//   a_T_c = a_T_b * b_T_c
//
// Not all coordinate frames are connected. If this is the case or either of the two coordinate
// frames does not exist the pose is said to be "invalid".
class PoseTree : public Component {
 public:
  using FrameId = std::string;
  using UpdateFunction =
      std::function<void(const std::string& lhs, const std::string& rhs,
                         double time, const Pose3d& lhs_T_rhs)>;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs`. This function will assert
  // if the pose is invalid.
  Pose3d get(const FrameId& lhs, const FrameId& rhs, double time) const;
  // Similar to `get`, but also converts the 3D pose to a 2D pose relative to the plane Z = 0.
  Pose2d getPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs` using the latest pose on each
  // edge. This function will assert if no path exist between lhs and rhs.
  std::pair<Pose3d, double> getLatest(const FrameId& lhs, const FrameId& rhs) const;
  // Similar to `getLatest`, but also converts the 3D pose to a 2D pose relative to the plane Z = 0.
  std::pair<Pose2d, double> getLatestPose2XY(const FrameId& lhs, const FrameId& rhs) const;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs`. This function will return
  // nullopt if the pose is invalid.
  std::optional<Pose3d> tryGet(const FrameId& lhs, const FrameId& rhs, double time) const;
  // Similar to `tryGet`, but also converts the 3D pose to a 2D pose relative to the plane Z = 0.
  std::optional<Pose2d> tryGetPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs` using the latest pose on each
  // edge. Returns nullopt if no path exist between lhs and rhs.
  std::optional<std::pair<Pose3d, double>> tryGetLatest(const FrameId& lhs,
                                                        const FrameId& rhs) const;
  // Similar to tryGetLatest, but also converts the 3D pose to a 2D pose relative to the plane Z = 0
  std::optional<std::pair<Pose2d, double>> tryGetLatestPose2XY(const FrameId& lhs,
                                                               const FrameId& rhs) const;

  // Sets the relative pose between two coordinate frames.  If the parameter is not specified and
  // the pose could not be set the function will assert.
  void set(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time);
  // Similar to `set` but for setting a pose in the Z = 0 plane.
  void set(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time);

  // Sets the relative pose between two coordinate frames.  If the parameter is not specified and
  // the pose could not be set the function will return false.
  bool trySet(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time);
  // Similar to `trySet` but for setting a pose in the Z = 0 plane.
  bool trySet(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time);

  // Disconnects a frame from the PoseTree and all the edges connected to it.
  bool removeFrame(const FrameId& frame, double stamp);

  // Removes an edge from the PoseTree starting at a given time.
  bool removeEdge(const FrameId& lhs, const FrameId& rhs, double stamp);

  // Creates a full copy of the pose tree.
  pose_tree::PoseTree clonePoseTree() const;

  // Creates a copy of the pose tree where each edge contains only the latest pose.
  pose_tree::PoseTree cloneLatestPoseTree() const;

  // Receive a callback every time a new pose is set.
  void registerForUpdates(const Component* callback_owner, UpdateFunction callback);

  // Stop receiving callbacks
  void deregisterForUpdates(const Component* callback_owner);

 private:
  mutable std::shared_timed_mutex pose_tree_mutex_;
  pose_tree::PoseTree pose_tree_;

  mutable std::shared_timed_mutex callbacks_mutex_;
  std::unordered_map<const Component*, UpdateFunction> callbacks_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::PoseTree)
