/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "pybind11/pybind11.h"

#include "packages/pyalice/bindings/pybind_component.hpp"
#include "packages/pyalice/bindings/pybind_node.hpp"

namespace isaac {
namespace alice {

class PyCodelet;

// Provides access to alice pycodelet in Python
class PybindPyCodelet {
 public:
  PybindPyCodelet(alice::PybindComponent& component);

  void tickPeriodically(pybind11::float_ interval);
  void tickBlocking();
  // set tick on message beahviour by the channel's tag
  void tickOnMessage(pybind11::str tag);
  // set synchronization behaviour by the channel's tags
  void synchronize(pybind11::str tag1, pybind11::str tag2);

  pybind11::float_ getTickTime();
  pybind11::float_ getTickDt();
  pybind11::bool_ isFirstTick();
  pybind11::int_ getTickCount();

  // adds a new rx message hook by its tag
  void addRxHook(pybind11::str rx_hook);
  // adds a new tx message hook by its tag
  void addTxHook(pybind11::str tx_hook);

  // a blocking function that waits until the C++ side notifies it to start a particular job
  pybind11::str pythonWaitForJob();
  // a non-blocking function that notifies the C++ side that it finishes the job
  void pythonJobFinished();

  // publish on sight interface
  void show(pybind11::str sop_json);

  // Update status to success or failure
  bool updateStatus(Status status, const std::string& msg);

  // gets the component name of the corresponding PyCodelet component
  pybind11::str getComponentName() const;
  // gets the node name of the corresponding PyCodelet component
  pybind11::str getNodeName() const;

  // gets the new message
  pybind11::object getMessage(const std::string& tag);

 private:
  PyCodelet* pycodelet_node_;  // the actual C++ component that is registered as a isaac codelet
};

// Initializes the python module
void InitPybindPyCodelet(pybind11::module& m);

}  // namespace alice
}  // namespace isaac
