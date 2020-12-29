/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_node.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/node.hpp"
#include "pybind11/stl.h"

namespace isaac {
namespace alice {

PybindNode::PybindNode() : node_(nullptr) {}

PybindNode::PybindNode(alice::Node* node) : node_(node) {}

PybindNode::~PybindNode() {}

const std::string& PybindNode::name() const {
  return node_->name();
}

alice::Status PybindNode::getStatus() const {
  return (node_ ? node_->getStatus() : Status::INVALID);
}

PybindComponent PybindNode::addComponent(const std::string& type, const std::string& name) {
  return PybindComponent(node_->addComponent(type, name));
}

pybind11::object PybindNode::getComponent(const std::string& name) {
  auto* component_ptr = node_->findComponentByName(name);
  if (component_ptr == nullptr) {
    return pybind11::object(pybind11::cast(nullptr));
  }
  return pybind11::cast(PybindComponent(component_ptr));
}

std::vector<std::string> PybindNode::getAllComponentNames() {
  std::vector<std::string> result;
  if (node_ == nullptr) return result;

  const std::vector<Component*> components = node_->getComponents();
  result.reserve(components.size());
  for (const Component* c : components) {
    result.push_back(c->name());
  }

  return result;
}

void PybindNode::disableAutomaticStart(bool flag) {
  node_->disable_automatic_start = flag;
}

void PybindNode::setStartOrder(const int order) {
  node_->start_order = order;
}

void PybindNode::start() {
  LOG_ERROR("starting %s", node_->name().c_str());
  node_->app()->backend()->node_backend()->startNode(node_);
}
void PybindNode::stop() {
  node_->app()->backend()->node_backend()->stopNode(node_);
}

void InitPybindNode(pybind11::module& m) {
  pybind11::enum_<Status>(m, "Status")
      .value("Success", Status::SUCCESS)
      .value("Failure", Status::FAILURE)
      .value("Running", Status::RUNNING)
      .value("Invalid", Status::INVALID)
      .export_values();
  pybind11::class_<PybindNode>(m, "PybindNode")
      .def(pybind11::init<>())
      .def("add_component", &isaac::alice::PybindNode::addComponent)
      .def("get_all_component_names", &isaac::alice::PybindNode::getAllComponentNames)
      .def("get_component", &isaac::alice::PybindNode::getComponent)
      .def("disable_automatic_start", &isaac::alice::PybindNode::disableAutomaticStart)
      .def("get_status", &isaac::alice::PybindNode::getStatus)
      .def("is_null", &isaac::alice::PybindNode::isNull)
      .def("name", &isaac::alice::PybindNode::name)
      .def("set_start_order", &isaac::alice::PybindNode::setStartOrder)
      .def("start", &isaac::alice::PybindNode::start)
      .def("stop", &isaac::alice::PybindNode::stop);
}

}  // namespace alice
}  // namespace isaac
