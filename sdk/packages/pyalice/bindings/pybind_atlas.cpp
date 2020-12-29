/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_atlas.hpp"

#include <array>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/component_impl.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace isaac {
namespace alice {

PybindAtlas::PybindAtlas(const PybindApplication& app) {
  pose_tree_ = app.app().backend()->pose_tree();
}

PybindAtlas::~PybindAtlas() {}

pybind11::object PybindAtlas::getPose(const std::string& lhs, const std::string& rhs,
                                      const double time) {
  if (pose_tree_ == nullptr) {
    return pybind11::none();
  }
  const auto pose = pose_tree_->tryGet(lhs, rhs, time);
  if (!pose) {
    return pybind11::none();
  }
  const auto q = pose->rotation.quaternion();
  const auto t = pose->translation;
  return pybind11::cast(std::array<double, 7>{q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z()});
}

bool PybindAtlas::setPose(const std::string& lhs, const std::string& rhs, const double time,
                          const std::array<double, 7>& pose) {
  if (pose_tree_ == nullptr) {
    return false;
  }
  const auto rotation = SO3d::FromQuaternion(Quaterniond(pose[0], pose[1], pose[2], pose[3]));
  const auto translation = Vector3d(pose[4], pose[5], pose[6]);
  pose_tree_->set(lhs, rhs, Pose3d{rotation, translation}, time);
  return true;
}

void InitPybindAtlas(pybind11::module& m) {
  pybind11::class_<PybindAtlas>(m, "PybindAtlas")
      .def(pybind11::init<const PybindApplication&>())
      .def("get_pose", &PybindAtlas::getPose)
      .def("set_pose", &PybindAtlas::setPose);
}

}  // namespace alice
}  // namespace isaac
