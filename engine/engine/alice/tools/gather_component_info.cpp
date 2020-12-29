/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/tools/gather_component_info.hpp"

#include <string>
#include <utility>

#include "engine/alice/backend/component_registry.hpp"
#include "engine/alice/component.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

Json GatherComponentInfo() {
  Json json;

  // Iterate over all registered components
  auto& registry = Singleton<ComponentRegistry>::Get();
  for (const std::string& name : registry.getNames()) {
    // Create the component of the given type name. We get a pointer to the base class component,
    // but it will actually have the type we requested.
    Component* component = registry.create(name);
    ASSERT(component, "could not create component '%s'", name.c_str());

    // Prepare a JSON object with information about all hooks
    Json hooks_json = Json::array();
    for (const Hook* hook_base : component->hooks()) {
      if (const MessageHook* message_hook = dynamic_cast<const MessageHook*>(hook_base)) {
        Json hook_json;
        hook_json["type"] = "message";
        hook_json["tag"] = message_hook->tag();
        if (dynamic_cast<const RxMessageHook*>(message_hook)) {
          hook_json["direction"] = "rx";
        } else if (dynamic_cast<const TxMessageHook*>(message_hook)) {
          hook_json["direction"] = "tx";
        } else {
          LOG_WARNING("Unknown message hook: '%s'", message_hook->tag().c_str());
        }
        hooks_json.emplace_back(std::move(hook_json));
      } else if (const ConfigHook* config_hook = dynamic_cast<const ConfigHook*>(hook_base)) {
        Json hook_json;
        hook_json["type"] = "config";
        hook_json["key"] = config_hook->key();
        hook_json["type_name"] = config_hook->type_name();
        hook_json["default"] = config_hook->getDefault();
        hooks_json.emplace_back(std::move(hook_json));
      } else if (const pose_hook_details::PoseHookBase* pose_hook =
          dynamic_cast<const pose_hook_details::PoseHookBase*>(hook_base)) {
        Json hook_json;
        hook_json["type"] = "pose";
        hook_json["lhs"] = pose_hook->getLhsName();
        hook_json["rhs"] = pose_hook->getRhsName();
        hooks_json.emplace_back(std::move(hook_json));
      } else {
        LOG_WARNING("Unknown hook in component '%s'", component->name().c_str());
      }
    }

    // Prepare a JSON object with information about the component
    Json node_json;
    node_json["base_type_name"] = component->base_type_name();
    node_json["type_name"] = component->type_name();
    node_json["hooks"] = std::move(hooks_json);
    json[name] = std::move(node_json);

    delete component;
  }

  return json;
}

}  // namespace alice
}  // namespace isaac
