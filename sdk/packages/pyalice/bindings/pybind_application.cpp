/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_application.hpp"

#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/clock.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/modules.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "engine/alice/utils/path_utils.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "engine/gems/serialization/json.hpp"
#include "messages/utils/utils.hpp"
#include "pybind11/stl.h"

namespace isaac {
namespace alice {

PybindApplication::PybindApplication(const std::string& app_json_str, const std::string& more_jsons,
                                     const std::string& asset_path) {
  ApplicationJsonLoader loader(asset_path);

  // Load app
  const auto json = serialization::ParseJson(app_json_str);
  ASSERT(json, "Could not parse JSON from input string '%s'", app_json_str.c_str());
  loader.loadApp(*json);
  home_workspace_name_ = loader.getHomeWorkspaceName();

  // Load additional files
  if (!more_jsons.empty()) {
    for (const auto& filename : SplitString(more_jsons, ',')) {
      if (!filename.empty()) {
        const auto more_json = serialization::TryLoadJsonFromFile(filename);
        ASSERT(more_json, "Could not load additional file '%s'", filename.c_str());
        loader.loadMore(*more_json);
      }
    }
  }

  app_.reset(new Application(loader));
}

PybindApplication::~PybindApplication() {}

std::set<std::string> PybindApplication::getLoadedComponents() const {
  return app_->backend()->module_manager()->getComponentNames();
}

void PybindApplication::start() {
  app_->disableStopOnCtrlC();
  app_->runAsync();
}

void PybindApplication::stop() {
  app_->stopBlocking();
}

void PybindApplication::start_wait_stop() {
  app_->runBlocking();
}

void PybindApplication::run(pybind11::object max_duration, pybind11::object node_name) {
  pybind11::gil_scoped_release release_gil;
  if (!max_duration.is_none()) {
    app_->enableStopOnTimeout(max_duration.cast<double>());
  }
  if (!node_name.is_none()) {
    app_->enableStopOnNode(node_name.cast<std::string>());
  }
  app_->runBlocking();
}

pybind11::str PybindApplication::uuid() const {
  std::string uuid;
  {
    pybind11::gil_scoped_release release_gil;
    uuid = app_->uuid().str();
  }
  return pybind11::str(uuid);
}

PybindNode PybindApplication::createNode(const std::string& name) {
  return PybindNode(app_->createNode(name));
}

void PybindApplication::connect(const std::string& source, const std::string& target) {
  app_->connect(source, target);
}

void PybindApplication::connect(const PybindComponent& tx_component, const std::string& tx_tag,
                                const PybindComponent& rx_component, const std::string& rx_tag) {
  Connect(tx_component.component(), tx_tag, rx_component.component(), rx_tag);
}

PybindNode PybindApplication::findNodeByName(const std::string& name) {
  return PybindNode(app_->findNodeByName(name));
}

std::string PybindApplication::getConfig(const std::string& node, const std::string& component,
                                         const std::string& parameter) const {
  // Get the parameter as a JSON string
  auto* maybe_json = app_->backend()->config_backend()->tryGetJson(node, component, parameter);
  if (maybe_json == nullptr) {
    return "{}";
  }

  return maybe_json->dump();
}

bool PybindApplication::eraseConfig(const std::string& node_name, const std::string& component_name,
                                    const std::string& parameter_name) {
  if (node_name.empty() || component_name.empty() || parameter_name.empty()) {
    return false;
  }
  return app_->backend()->config_backend()->erase(node_name, component_name, parameter_name);
}

void PybindApplication::setConfig(const std::string& node, const std::string& component,
                                  const std::string& key, const std::string& value) {
  auto maybe_json = serialization::ParseJson(value);
  if (maybe_json) {
    app_->backend()->config_backend()->setJson(node, component, key, std::move(*maybe_json));
  }
}

PybindMessage PybindApplication::receive(const std::string& node_name,
                                         const std::string& component_name,
                                         const std::string& channel_name) {
  Node* node = app_->findNodeByName(node_name);
  if (node == nullptr) {
    return PybindMessage();
  }

  MessageLedger* ledger = node->getComponentOrNull<alice::MessageLedger>();
  if (ledger == nullptr) {
    return PybindMessage();
  }

  Component* component = node->findComponentByName(component_name);

  MessageLedger::Endpoint endpoint;
  if (component) {
    endpoint.component = component;
  } else {
    endpoint.component = ledger;
  }
  endpoint.tag = channel_name;

  MessageBasePtr message_ptr;

  if (auto* queue = ledger->tryGetQueue(endpoint)) {
    queue->sync();
    if (!queue->empty()) {
      message_ptr = std::const_pointer_cast<isaac::alice::MessageBase>(queue->latest());
      queue->popAll();  // FIXME(yangl) Should not only return latest message but give queue access.
    }
  }

  return PybindMessage(message_ptr);
}

bool PybindApplication::publish(const std::string& node_name, const std::string& component_name,
                                const std::string& channel_name, const std::string& uuid_str,
                                uint64_t type_id, const std::string proto_bytes,
                                std::vector<pybind11::buffer>& buffers, int64_t acqtime) {
  Node* node = app_->findNodeByName(node_name);
  if (node == nullptr) {
    // Fails on missing node
    return false;
  }
  Component* component = node->findComponentByName(component_name);
  MessageLedger* ledger = node->getComponentOrNull<alice::MessageLedger>();
  if (component == nullptr || ledger == nullptr) {
    // Fails on missing component or ledger
    return false;
  }
  const Uuid uuid = uuid_str.empty() ? Uuid::Generate() : Uuid::FromString(uuid_str);

  // Populates buffers
  std::vector<CpuBuffer> cpu_buffers;
  cpu_buffers.reserve(buffers.size());
  for (auto& buffer : buffers) {
    pybind11::buffer_info info = buffer.request();
    const size_t buffer_size = info.itemsize * info.size;
    CpuBuffer cpuBuffer(buffer_size);
    std::copy(reinterpret_cast<byte*>(info.ptr), reinterpret_cast<byte*>(info.ptr) + buffer_size,
              cpuBuffer.begin());
    cpu_buffers.emplace_back(std::move(cpuBuffer));
  }

  // Creates ProtoMessage
  CpuBuffer proto_buffer;
  proto_buffer.resize(proto_bytes.size());
  std::memcpy(reinterpret_cast<char*>(proto_buffer.begin()), proto_bytes.data(),
              proto_bytes.size());

  MessageBasePtr message_base =
      utils::CreateProtoMessage(uuid, type_id, std::move(proto_buffer), std::move(cpu_buffers));
  if (message_base == nullptr) {
    return false;
  }

  // Populates acq and pub time
  const int64_t now = app_->backend()->clock()->timestamp();
  message_base->pubtime = now;
  message_base->acqtime = acqtime == 0 ? now : acqtime;

  // Publishes message
  MessageLedger::Endpoint endpoint{component, channel_name};
  ledger->provide(endpoint, std::move(message_base));
  ledger->notifyScheduler(endpoint, now);

  return true;
}

void PybindApplication::setHomeWorkspaceName(const std::string& home_workspace_name) {
  home_workspace_name_ = home_workspace_name;
}

const std::string& PybindApplication::getHomeWorkspaceName() {
  return home_workspace_name_;
}

std::pair<std::string, std::string> PybindApplication::expandAssetPath(
    const std::string& asset_path) {
  return TranslateAssetPath(asset_path, home_workspace_name_, std::string(""));
}

bool PybindApplication::load(const std::string& filename, const std::string& prefix,
                             const std::string& context_workspace) {
  ApplicationJsonLoader loader;
  loader.setHomeWorkspaceName(home_workspace_name_);

  const auto json = serialization::TryLoadJsonFromFile(filename);
  if (!json) {
    return false;
  }
  loader.loadMore(*json, prefix, context_workspace);

  app_->createMore(loader);

  return true;
}

bool PybindApplication::loadModule(const std::string& module_name) {
  if (module_name.empty()) {
    return false;
  }

  ApplicationJsonLoader loader;
  loader.setHomeWorkspaceName(home_workspace_name_);
  loader.loadModule(module_name);
  app_->createMore(loader);

  return true;
}

std::vector<std::string> PybindApplication::getAllNodeNames() {
  std::vector<std::string> result;
  if (app_ == nullptr) return result;

  const std::vector<Node*> nodes = app_->nodes();
  result.reserve(nodes.size());
  for (const Node* n : nodes) {
    result.push_back(n->name());
  }
  return result;
}

void InitPybindApplication(pybind11::module& m) {
  pybind11::class_<PybindApplication>(m, "PybindApplication")
      .def(pybind11::init<const std::string&, const std::string&, const std::string&>())
      .def("connect", pybind11::overload_cast<const PybindComponent&, const std::string&,
                                              const PybindComponent&, const std::string&>(
                          &PybindApplication::connect))
      .def("connect", pybind11::overload_cast<const std::string&, const std::string&>(
                          &PybindApplication::connect))
      .def("create_node", &PybindApplication::createNode)
      .def("erase_config", &PybindApplication::eraseConfig)
      .def("expand_asset_path", &PybindApplication::expandAssetPath)
      .def("find_node_by_name", &PybindApplication::findNodeByName)
      .def("get_config", &PybindApplication::getConfig)
      .def("get_home_workspace_name", &PybindApplication::getHomeWorkspaceName)
      .def("get_loaded_components", &PybindApplication::getLoadedComponents)
      .def("get_all_node_names", &PybindApplication::getAllNodeNames)
      .def("load", &PybindApplication::load)
      .def("load_module", &PybindApplication::loadModule)
      .def("publish", &PybindApplication::publish)
      .def("receive", &PybindApplication::receive)
      .def("run", &PybindApplication::run)
      .def("set_config", &PybindApplication::setConfig)
      .def("set_home_workspace_name", &PybindApplication::setHomeWorkspaceName)
      .def("start", &PybindApplication::start, pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("start_wait_stop", &PybindApplication::start_wait_stop,
           pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("stop", &PybindApplication::stop, pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("uuid", &PybindApplication::uuid);
}

}  // namespace alice
}  // namespace isaac
