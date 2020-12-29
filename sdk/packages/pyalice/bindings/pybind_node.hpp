/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "packages/pyalice/bindings/pybind_component.hpp"

#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

class Node;
enum class Status;

// A wrapper around an alice node to make it accessible to Python via pybind
class PybindNode {
 public:
  PybindNode();
  PybindNode(alice::Node* node);
  ~PybindNode();

  // Returns a pointer to the underlying alice node
  alice::Node* handle() const { return node_; }

  // Returns true if handle is nullptr
  bool isNull() const { return node_ == nullptr; }

  // Calls Node::name
  const std::string& name() const;
  // Calls Node::getStatus
  alice::Status getStatus() const;
  // Calls Node::addComponent
  PybindComponent addComponent(const std::string& type, const std::string& name);
  // Calls Node::getComponent
  pybind11::object getComponent(const std::string& name);
  // Returns names of all components
  std::vector<std::string> getAllComponentNames();
  // Sets Node::disable_automatic_start variable
  void disableAutomaticStart(bool flag);
  // Sets Node::start_order variable
  void setStartOrder(const int start_order);

  // Starts the node
  void start();
  // Stops the node
  void stop();

 private:
  alice::Node* node_;
};

// Initializes the python module
void InitPybindNode(pybind11::module& m);

}  // namespace alice
}  // namespace isaac
