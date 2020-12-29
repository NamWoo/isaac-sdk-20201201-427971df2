/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "kinematic_tree.hpp"

#include <memory>
#include <string>
#include <utility>

#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace kinematic_tree {

const KinematicTree::Link* KinematicTree::tryGetLink(const std::string& name) const {
  auto it = links_map_.find(name);
  return it == links_map_.end() ? nullptr : it->second;
}

const KinematicTree::Link* KinematicTree::tryGetLink(LinkId link_id) const {
  if (link_id < 0 || link_id >= getNumberOfLinks()) return nullptr;
  return &links_[link_id];
}

const KinematicTree::Link* KinematicTree::addLink(const std::string& name,
                                                  const std::string& parent,
                                                  std::unique_ptr<Motor> motor) {
  return addLink(name, tryGetLink(parent), std::move(motor));
}

const KinematicTree::Link* KinematicTree::addLink(const std::string& name, const Link* parent,
                                                  std::unique_ptr<Motor> motor) {
  state_offset_.push_back(next_offset_);
  links_.push_back(
      {name, parent, /* children = */ {}, std::move(motor), static_cast<int>(links_.size())});
  const Link* link = &links_.back();
  if (link->motor && link->motor->getNumberOfArguments() > 0) {
    next_offset_ += link->motor->getNumberOfArguments();
    active_links_.push_back(link);
  }
  // Add root link
  if (parent == nullptr) {
    ASSERT(root_link_ == nullptr, "Root link already exist. Cannot add a second root link.");
    root_link_ = link;
  }
  links_map_[name] = link;
  if (parent != nullptr) {
    links_[parent->id].children.push_back(link);
  }
  return link;
}

int KinematicTree::getNumberOfLinks() const {
  return static_cast<int>(links_.size());
}

int KinematicTree::getMotorStateDimensions() const {
  return next_offset_;
}

EigenVectorConstView<double> KinematicTree::getMotorStateView(const VectorXd& state,
                                                              LinkId link_id) const {
  return EigenVectorConstView<double>(state.data() + getLinkOffset(link_id),
                                      links_[link_id].motor->getNumberOfArguments());
}

int KinematicTree::getLinkOffset(LinkId link_id) const {
  ASSERT(0 <= link_id && link_id < getNumberOfLinks(), "Invalid link_id: %d", link_id);
  ASSERT(links_[link_id].motor.get() != nullptr, "This link does not have a motor attached to it.");
  return state_offset_[link_id];
}

}  // namespace kinematic_tree
}  // namespace isaac
