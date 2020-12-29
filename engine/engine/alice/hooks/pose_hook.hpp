/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/hooks/hook.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"

namespace isaac {
namespace alice {

class Pose;

namespace pose_hook_details {

// Base class for pose hooks used to access the pose tree from a codelet
class PoseHookBase : public Hook {
 public:
  // Sets the name of the "left" frame
  void setLhsName(const std::string& lhs) { lhs_ = lhs; }
  // Sets the name of the "right" frame
  void setRhsName(const std::string& rhs) { rhs_ = rhs; }
  // Gets the name of the "left" frame
  std::string getLhsName() const { return lhs_; }
  // Gets the name of the "right" frame
  std::string getRhsName() const { return rhs_; }

 protected:
  PoseHookBase(Component* component, const std::string& lhs, const std::string& rhs);
  const std::string& lhs() const { return lhs_; }
  const std::string& rhs() const { return rhs_; }
  Pose3d getImpl(double time) const;
  Pose3d getImpl(double time, bool& ok) const;
  void setImpl(const Pose3d& lhs_T_rhs, double time);
  void setImpl(const Pose3d& lhs_T_rhs, double time, bool& ok);

  void connect() override;

 private:
  Pose* pose_;
  std::string lhs_, rhs_;
};

// Converts poses between different dimensions
template<int N>
struct PoseHookTraits;

template<>
struct PoseHookTraits<2> {
  using PoseType = Pose2d;
  static PoseType Convert(const Pose3d& pose) { return pose.toPose2XY(); }
  static Pose3d Convert(const PoseType& pose) { return Pose3d::FromPose2XY(pose); }
};

template<>
struct PoseHookTraits<3> {
  using PoseType = Pose3d;
  static const PoseType& Convert(const PoseType& pose) { return pose; }
};

}  // namespace pose_hook_details

// A hook to access a pose in the pose graph
// Usage example:
//  class MyCodelet : isaac::alice::Codelet {
//   public:
//    ...
//    ISAAC_POSE3(world, robot);
//    ...
//    void tick() {
//      // get the pose when know that the pose exists (will assert otherwise)
//      const Pose3d robot_T_lidar = get_robot_T_lidar();
//      ...
//      // set the pose (will assert if this would form a cycle)
//      set_robot_T_lidar(robot_T_lidar);
//      ...
//      // get the pose when we don't know if the pose exists
//      bool ok;
//      const Pose3d world_T_robot = get_world_T_robot(ok);
//      if (!ok) {
//        // error handling
//        return;
//      }
//      ...
//    }
//  };
template <int N>
class PoseHook : public pose_hook_details::PoseHookBase {
 public:
  using PoseType = typename pose_hook_details::PoseHookTraits<N>::PoseType;
  // Creates a hook for a pose between `lhs` and `rhs` and links it to a component
  PoseHook(Component* component, const std::string& lhs, const std::string& rhs)
  : pose_hook_details::PoseHookBase(component, lhs, rhs) {}
  // Gets the pose at the given time, or assert if the pose does not exist
  PoseType get(double time) const {
    return pose_hook_details::PoseHookTraits<N>::Convert(getImpl(time));
  }
  // Gets the pose at the given time, or sets `ok` to false if the pose does not exist
  PoseType get(double time, bool& ok) const {
    return pose_hook_details::PoseHookTraits<N>::Convert(getImpl(time, ok));
  }
  // Sets the pose for the given time, or assert if this would form a cycle
  void set(const PoseType& pose, double time) {
    setImpl(pose_hook_details::PoseHookTraits<N>::Convert(pose), time);
  }
  // Sets the pose for the given time, or sets `ok` to false if the pose does not exist
  void set(const PoseType& pose, double time, bool& ok) {
    setImpl(pose_hook_details::PoseHookTraits<N>::Convert(pose), time, ok);
  }
};

// Helper macros for pose hooks
#define _ISAAC_POSE_IMPL_UNI(N, A, B) \
 private: \
  ::isaac::alice::PoseHook<N> A##_T_##B##_{this, #A, #B}; \
 public: \
  ::isaac::alice::PoseHook<N>::PoseType get_##A##_T_##B(double time) const { \
    return A##_T_##B##_.get(time); \
  } \
  ::isaac::alice::PoseHook<N>::PoseType get_##A##_T_##B(double time, bool& ok) const { \
    return A##_T_##B##_.get(time, ok); \
  } \
  void set_##A##_T_##B(const ::isaac::alice::PoseHook<N>::PoseType& A##_T_##B, double time) { \
    A##_T_##B##_.set(A##_T_##B, time); \
  }
#define _ISAAC_POSE_IMPL_BI(N, A, B) \
  _ISAAC_POSE_IMPL_UNI(N, A, B) \
  _ISAAC_POSE_IMPL_UNI(N, B, A)

// Gives access to a 2D pose
#define ISAAC_POSE2(A, B) _ISAAC_POSE_IMPL_BI(2, A, B)
// Gives access to a 3D pose
#define ISAAC_POSE3(A, B) _ISAAC_POSE_IMPL_BI(3, A, B)

}  // namespace alice
}  // namespace isaac
