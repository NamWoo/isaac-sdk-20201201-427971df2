/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/core/optional.hpp"
#include "packages/math/gems/kinematic_tree/motor.hpp"

namespace isaac {
namespace kinematic_tree {

// Class holding the representation of a kinematic tree constitued of links. The links form a tree
// structure with the root being the first link (link_[0]), and each successive link will have a
// single parent.
class KinematicTree {
 public:
  using LinkId = int;
  // Structure holding all the information about a given link.
  struct Link {
    // Name of the current link
    std::string name;
    // pointer to the parent link. (nullptr if this link is the root)
    const Link* parent;
    // List of links attached to this one.
    std::vector<const Link*> children;
    // Pointer to a motor object that represent the transformation applied from this link to its
    // children.
    std::unique_ptr<Motor> motor;
    // TODO(Ben): Add collider

    // Id of the link in Vector state.
    LinkId id;
  };

  // Returns the link with the given name if it exists. Returns nullptr otherwise.
  const Link* tryGetLink(const std::string& name) const;
  // Returns the link with the given id if it exists. Returns nullptr otherwise.
  const Link* tryGetLink(LinkId link_id) const;
  // Returns root link if it exists. Returns nullptr otherwise.
  const Link* tryGetRootLink() const { return root_link_; }

  // Add a link with a given parent and with a given pose.
  // If the link is the root, parent need to be nullptr
  const Link* addLink(const std::string& name, const Link* parent,
                      std::unique_ptr<Motor> motor = std::unique_ptr<Motor>());
  // Add a link with a given parent and with a given pose.
  // If the link is the root, provide an empty name for parent.
  const Link* addLink(const std::string& name, const std::string& parent,
                      std::unique_ptr<Motor> motor = std::unique_ptr<Motor>());

  // Returns the number of links of this tree.
  int getNumberOfLinks() const;

  // Returns the number of dimensions (free parameters)
  int getMotorStateDimensions() const;

  // Returns the state view for a given link.
  EigenVectorConstView<double> getMotorStateView(const VectorXd& state, LinkId link_id) const;

  // Returns the offset of a link in the state.
  int getLinkOffset(LinkId link_id) const;

  // Returns links with non-constant motor
  const std::vector<const Link*>& getActiveLinks() const { return active_links_; }

 private:
  std::deque<Link> links_;
  const Link* root_link_ = nullptr;
  std::vector<int> state_offset_;
  int next_offset_ = 0;
  std::vector<const Link*> active_links_;
  std::unordered_map<std::string, const Link*> links_map_;
};

}  // namespace kinematic_tree
}  // namespace isaac
