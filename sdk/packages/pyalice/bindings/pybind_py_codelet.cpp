/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_py_codelet.hpp"

#include "pybind11/pybind11.h"

#include "engine/alice/components/PyCodelet.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"
#include "packages/pyalice/bindings/pybind_message.hpp"

namespace isaac {
namespace alice {

PybindPyCodelet::PybindPyCodelet(alice::PybindComponent& component) {
  ASSERT(component.component_, "null argument");
  pycodelet_node_ = dynamic_cast<PyCodelet*>(component.component_);
  ASSERT(pycodelet_node_, "Component %s/%s is not PyCodelet",
         component.component()->node()->name().c_str(), component.name().c_str());
}

void PybindPyCodelet::tickPeriodically(pybind11::float_ interval) {
  const double interval_val = interval;
  {
    pybind11::gil_scoped_release release_gil;
    pycodelet_node_->tickPeriodically(interval_val);
  }
}

void PybindPyCodelet::tickBlocking() {
  pycodelet_node_->tickBlocking();
}

void PybindPyCodelet::tickOnMessage(pybind11::str tag) {
  const std::string tag_val = std::string(tag);
  {
    pybind11::gil_scoped_release release_gil;
    pycodelet_node_->tickOnMessageWithTag(tag);
  }
}

void PybindPyCodelet::synchronize(pybind11::str py_tag1, pybind11::str py_tag2) {
  const std::string tag1 = py_tag1;
  const std::string tag2 = py_tag2;
  {
    pybind11::gil_scoped_release release_gil;
    pycodelet_node_->synchronizeWithTags(tag1, tag2);
  }
}

pybind11::float_ PybindPyCodelet::getTickTime() {
  double val = 0.0;
  {
    pybind11::gil_scoped_release release_gil;
    val = pycodelet_node_->getTickTime();
  }
  return val;
}

pybind11::float_ PybindPyCodelet::getTickDt() {
  double val = 0.0;
  {
    pybind11::gil_scoped_release release_gil;
    val = pycodelet_node_->getTickDt();
  }
  return val;
}

pybind11::bool_ PybindPyCodelet::isFirstTick() {
  bool result = false;
  {
    pybind11::gil_scoped_release release_gil;
    result = pycodelet_node_->isFirstTick();
  }
  return result;
}

pybind11::int_ PybindPyCodelet::getTickCount() {
  int val = 0;
  {
    pybind11::gil_scoped_release release_gil;
    val = pycodelet_node_->getTickCount();
  }
  return val;
}

void PybindPyCodelet::addRxHook(pybind11::str rx_hook) {
  std::string hook = rx_hook;
  {
    pybind11::gil_scoped_release release_gil;
    pycodelet_node_->addRxHook(hook);
  }
}

void PybindPyCodelet::show(pybind11::str sop_json) {
  std::string sop_string = sop_json;
  {
    pybind11::gil_scoped_release release_gil;
    pycodelet_node_->show(sop_string);
  }
}

pybind11::str PybindPyCodelet::pythonWaitForJob() {
  std::string job;
  {
    pybind11::gil_scoped_release release_gil;
    job = pycodelet_node_->pythonWaitForJob();
  }
  return pybind11::str(job);
}

void PybindPyCodelet::pythonJobFinished() {
  pycodelet_node_->pythonJobFinished();
}

bool PybindPyCodelet::updateStatus(Status status, const std::string& msg) {
  pycodelet_node_->updateStatus(status, msg.c_str());
  return true;
}

pybind11::str PybindPyCodelet::getComponentName() const {
  return pycodelet_node_->name();
}

pybind11::str PybindPyCodelet::getNodeName() const {
  return pycodelet_node_->node()->name();
}

pybind11::object PybindPyCodelet::getMessage(const std::string& tag) {
  MessageBasePtr message_ptr =
      std::const_pointer_cast<isaac::alice::MessageBase>(pycodelet_node_->receiveMessage(tag));
  if (message_ptr == nullptr) {
    return pybind11::none();
  }
  return pybind11::cast(PybindMessage(message_ptr));
}

/*
 * As read from pybind11 code, pybind11 type wrappers access Python c-api for accessing
 * corresponding Py-instances, as shown below:
 * m_ptr = PyLong_FromLong((long) value);
 * These c-api talks to Python runtime.
 * As documented in Python c-api: https://docs.python.org/3/c-api/init.html
 * "The Python interpreter is not fully thread-safe. In order to support multi-threaded Python
 * programs, there’s a global lock, called the global interpreter lock or GIL, that must be held by
 * the current thread before it can safely access Python objects."
 * So the pybind11 code that manipulates python objects has to be protected by GIL.
 */

void InitPybindPyCodelet(pybind11::module& m) {
  pybind11::class_<isaac::alice::PybindPyCodelet>(m, "PybindPyCodelet")
      .def(pybind11::init<isaac::alice::PybindComponent&>())
      .def("add_rx_hook", &PybindPyCodelet::addRxHook)
      .def("get_component_name", &PybindPyCodelet::getComponentName)
      .def("get_message", &PybindPyCodelet::getMessage)
      .def("get_node_name", &PybindPyCodelet::getNodeName)
      .def("get_tick_count", &PybindPyCodelet::getTickCount)
      .def("get_tick_dt", &PybindPyCodelet::getTickDt)
      .def("get_tick_time", &PybindPyCodelet::getTickTime)
      .def("is_first_tick", &PybindPyCodelet::isFirstTick)
      .def("python_job_finished", &isaac::alice::PybindPyCodelet::pythonJobFinished,
           pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("python_wait_for_job", &PybindPyCodelet::pythonWaitForJob)
      .def("show", &PybindPyCodelet::show)
      .def("synchronize", &PybindPyCodelet::synchronize)
      .def("tick_blocking", &PybindPyCodelet::tickBlocking,
           pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("tick_on_message", &PybindPyCodelet::tickOnMessage)
      .def("tick_periodically", &PybindPyCodelet::tickPeriodically)
      .def("update_status", &PybindPyCodelet::updateStatus);
}

}  // namespace alice
}  // namespace isaac
