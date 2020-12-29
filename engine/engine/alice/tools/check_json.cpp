/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <memory>
#include <set>
#include <string>

#include "engine/alice/components/alice_all_components.hpp"
#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/modules.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {

// Load a json file, load the modules specified, and go through all the components and try to
// build them.
void Main(int argc, char* argv[]) {
  ASSERT(argc > 1, "Please provide a json file as argument");

  alice::ApplicationJsonLoader loader;
  Json json = serialization::LoadJsonFromFile(argv[1]);
  loader.loadApp(json);

  // Load modules
  alice::ModuleManager module_manage;
  module_manage.loadStaticallyLinked();
  module_manage.loadModules(loader.getModuleNames());

  // Extract components
  const auto components = loader.getComponentNames();
  // Check we can build each component.
  for (const auto& comp : components) {
    alice::Component* component = module_manage.createComponent(comp);
    ASSERT(component != nullptr, "Cannot create the component:", comp.c_str());
    delete component;
  }
}

}  // namespace isaac

int main(int argc, char* argv[]) {
  isaac::Main(argc, argv);
  return 0;
}
