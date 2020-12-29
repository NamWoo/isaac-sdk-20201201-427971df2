/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "modules.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "dlfcn.h"
#include "engine/alice/application.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

namespace {

constexpr const char kIsaacGatherComponentInfo[] = "IsaacGatherComponentInfo";
constexpr const char kIsaacCreateComponent[] = "IsaacCreateComponent";
constexpr const char kIsaacModuleStart[] = "IsaacModuleStart";
constexpr const char kIsaacModuleStop[] = "IsaacModuleStop";
constexpr const char kIsaacModuleInitialize[] = "IsaacModuleInitialize";

// Path to module libraries from external workspaces
constexpr const char kExternalPath[] = "external/";
constexpr size_t kExternalPathLength = sizeof(kExternalPath) - 1;  // '\0' does not count
// Expected path to external Isaac workspace from external one
constexpr const char kExternalIsaacPath[] = "external/com_nvidia_isaac/";

constexpr const char kPackages[] = "packages";
constexpr size_t kPackagesLength = sizeof(kPackages) - 1;  // '\0' does not count

// prefix path with working_path if working_path is not empty and path is relative.
// otherwise return path as is.
std::string ExpandPath(const std::string& path, const std::string& working_path) {
  // Do nothing about absolute path
  if (working_path.empty() || (!path.empty() && path.at(0) == '/')) {
    return path;
  }
  return working_path + "/" + path;
}

// Loads a package and makes sure it is a valid package.
void* LoadPackageImpl(const std::string& filename, const std::string& working_path) {
  // We need to be able to successfully load it
  void* handle = dlopen(ExpandPath(filename, working_path).c_str(), RTLD_LAZY);
  if (handle == nullptr) return nullptr;
  // We need to have the IsaacGatherComponentInfo function
  void* pointer = dlsym(handle, kIsaacGatherComponentInfo);
  if (pointer == nullptr) return nullptr;
  return handle;
}

// Loads a shared library. In case the shared library comes from a package an we fail to load it we
// are also trying to load it from platform-specific locations. This is due to the current method
// of creating the binary release.
void* LoadPackage(const std::vector<std::string>& paths, const std::string& filename,
                  const std::string& working_path) {
  void* handle = nullptr;
  // In case this is a package try to load it from platform specific directories. For example from
  // packages_nano/viewers/libviewers_module.so instead of packages/viewers/libviewers_module.so
  bool match_packages = filename.substr(0, kPackagesLength).compare(kPackages) == 0;
  bool match_external = filename.substr(0, kExternalPathLength).compare(kExternalPath) == 0;
  if (!match_packages && !match_external) {
    return LoadPackageImpl(filename, working_path);
  }
  std::string prefix = "";
  std::string remaining = filename;
  if (match_external) {
    prefix = kExternalPath;
    remaining = filename.substr(kExternalPathLength);
    const size_t pos = remaining.find("/");
    if (pos != std::string::npos) {
      // Grab repo folder to prefix
      absl::StrAppend(&prefix, remaining.substr(0, pos + 1));
      remaining = remaining.substr(pos + 1);
    }
  }
  if (remaining.substr(0, kPackagesLength).compare(kPackages) == 0) {
    const std::string sans_packages = remaining.substr(kPackagesLength);
    for (const auto& path : paths) {
      std::string alternative_filename;
      absl::StrAppend(&alternative_filename, prefix, path, sans_packages);
      handle = LoadPackageImpl(alternative_filename, working_path);
      if (handle != nullptr) return handle;
    }
  }
  // In case we failed to load from platform specific locations we load one last time to get the
  // original error message into dlerror.
  return LoadPackageImpl(filename, working_path);
}

}  // namespace

ModuleManager::Module::~Module() {
  if (handle_ != nullptr) close();
}

void ModuleManager::Module::open(const std::vector<std::string>& paths, const std::string& filename,
                                 const std::string& working_path) {
  ASSERT(handle_ == nullptr, "Module already loaded");

  // Open shared object
  dlerror();
  handle_ = LoadPackage(paths, filename, working_path);
  if (handle_ == nullptr) {
    error_message_ = dlerror();
    return;
  } else {
    filename_ = filename;
  }

  // Load component info function
  gather_component_info_function_ =
      reinterpret_cast<GatherComponentInfoFunction*>(loadSymbol(kIsaacGatherComponentInfo));
  if (gather_component_info_function_ == nullptr) {
    LOG_ERROR("Could not load symbol '%s' from module in file '%s'", kIsaacGatherComponentInfo,
              filename.c_str());
    close();
    return;
  }

  // Load create component function
  create_component_function_ =
      reinterpret_cast<CreateComponentFunction*>(loadSymbol(kIsaacCreateComponent));
  if (create_component_function_ == nullptr) {
    LOG_ERROR("Could not load symbol '%s' from module in file '%s'", kIsaacCreateComponent,
              filename.c_str());
    close();
    return;
  }

  // Get component info
  const int buffer_size = gather_component_info_function_(nullptr, 0);
  char* buffer = new char[buffer_size];
  const int buffer_size_now = gather_component_info_function_(buffer, buffer_size);
  ASSERT(buffer_size_now == buffer_size, "not supported");
  const std::string buffer_str = buffer;
  delete[] buffer;
  auto maybe_component_info = serialization::ParseJson(buffer_str);
  if (!maybe_component_info) {
    LOG_ERROR("Could not load module '%s': %s did not return a valid JSON object", filename.c_str(),
              kIsaacGatherComponentInfo);
    close();
    return;
  }
  component_info_ = *maybe_component_info;

  // Collect component info
  components_.clear();
  for (auto it : component_info_.items()) {
    components_.insert(it.key());
  }

  // Execute the module load function
  start_function_ = reinterpret_cast<IsaacModuleStartFunction*>(loadSymbol(kIsaacModuleStart));
  stop_function_ = reinterpret_cast<IsaacModuleStopFunction*>(loadSymbol(kIsaacModuleStop));
  initialize_function_ =
      reinterpret_cast<IsaacModuleInitializeFunction*>(loadSymbol(kIsaacModuleInitialize));

  name_ = filename;  // TODO Use a better name
}

void ModuleManager::Module::close() {
  ASSERT(handle_ != nullptr, "Module not loaded");
  dlerror();
  const int result = dlclose(handle_);
  if (result != 0) {
    LOG_ERROR("dlclose failed: %s", dlerror());
  } else {
    handle_ = nullptr;
  }
}

void ModuleManager::Module::start(Application* app) {
  if (start_function_) {
    start_function_(static_cast<void*>(app));
  }
}

void ModuleManager::Module::initialize(Application* app) {
  if (initialize_function_ && !initialized_) {
    initialized_ = true;
    initialize_function_(static_cast<void*>(app));
  }
}

void ModuleManager::Module::stop(Application* app) {
  if (stop_function_) {
    stop_function_(static_cast<void*>(app));
  }
}

void* ModuleManager::Module::loadSymbol(const std::string& name) {
  ASSERT(handle_ != nullptr, "Module not loaded");
  dlerror();
  return dlsym(handle_, name.c_str());
}

std::optional<std::string> ModuleManager::Module::findComponentName(
    const std::string& component_name) const {
  // Exact match
  if (components_.count(component_name) != 0) {
    return component_name;
  }
  // Try to find a non-namespace component name. For example Foo would match against bar::zar::Foo.
  for (const auto& x : components_) {
    const auto it = x.find(component_name);
    if (it == std::string::npos) continue;
    if (it > 2 && x.size() - it == component_name.size() && x[it - 1] == ':' && x[it - 2] == ':') {
      return x;
    }
  }
  return std::nullopt;
}

Component* ModuleManager::Module::createComponent(const std::string& type_name) const {
  const auto actual_name = findComponentName(type_name);
  if (!actual_name) {
    return nullptr;
  }
  if (actual_name != type_name) {
    LOG_WARNING("Matching component name '%s' to '%s'", type_name.c_str(), actual_name->c_str());
  }
  return static_cast<Component*>(create_component_function_(actual_name->c_str()));
}

void ModuleManager::loadStaticallyLinked() {
  for (const std::string& name : Singleton<ComponentRegistry>::Get().getNames()) {
    self_components_.insert(name);
  }
}

std::optional<std::string> ModuleManager::loadModule(const std::string& name) {
  std::unique_lock<std::mutex> lock(mutex_);

  auto module = std::make_unique<Module>();

  // Try to load the library
  module->open(module_paths_, name, working_path_);
  if (!module->isOpen()) {
    if (name.compare("packages/sight/libsight_module.so") == 0) {
      // FIXME Sight module needs to be loaded with awareness of workspace
      module->open(module_paths_, std::string(kExternalIsaacPath) + name, working_path_);
    }
    if (!module->isOpen()) {
      return module->getErrorMessage();
    }
  }

  // Check the name
  if (modules_.find(module->name()) != modules_.end()) {
    LOG_WARNING("Module with name '%s' already loaded", module->name().c_str());
    return std::nullopt;
  }

  LOG_INFO("Loaded module '%s': Now has %zd components total", name.c_str(),
           module->components().size());

  modules_[module->name()] = std::move(module);
  return std::nullopt;
}

void ModuleManager::loadModules(const std::vector<std::string>& filenames) {
  // We are trying to load modules iteratively until we suceed. This is a workaround because some
  // modules might depend on other modules and we need to load them in the right order.
  std::set<std::string> libraries_to_load(filenames.begin(), filenames.end());
  std::map<std::string, std::string> errors;
  while (!libraries_to_load.empty()) {
    std::set<std::string> failed;
    // Try to load modules
    for (const auto& name : libraries_to_load) {
      const auto maybe_error = loadModule(name);
      if (maybe_error) {
        failed.insert(name);
        errors[name] = *maybe_error;
      }
    }
    // Check if we made progress
    if (failed.size() == libraries_to_load.size()) {
      for (const auto& module : libraries_to_load) {
        LOG_ERROR("%s: %s", module.c_str(), errors[module].c_str());
      }
      PANIC("Could not load all required modules for application");
    }
    libraries_to_load = failed;
  }
}

void ModuleManager::unloadModule(const std::string& name) {
  std::unique_lock<std::mutex> lock(mutex_);
  // FIXME Currently not supported as we are actually using the static component registry
  auto it = modules_.find(name);
  if (it == modules_.end()) {
    LOG_ERROR("Module with name '%s' not loaded", name.c_str());
    return;
  }
  modules_.erase(it);
}

Component* ModuleManager::createComponent(const std::string& type_name) const {
  std::unique_lock<std::mutex> lock(mutex_);
  // First try statically linked components
  if (self_components_.count(type_name) != 0) {
    return Singleton<ComponentRegistry>::Get().create(type_name);
  }
  // Try components loaded via modules. This is currently in alphabetically order.
  for (const auto& kvp : modules_) {
    Component* component = kvp.second->createComponent(type_name);
    if (component) return component;
  }
  PANIC("Could not load component '%s'", type_name.c_str());
}

std::set<std::string> ModuleManager::getModuleNames() const {
  std::unique_lock<std::mutex> lock(mutex_);
  std::set<std::string> module_names;
  for (const auto& kvp : modules_) {
    module_names.insert(kvp.first);
  }
  return module_names;
}

std::set<std::string> ModuleManager::getComponentNames() const {
  std::unique_lock<std::mutex> lock(mutex_);
  std::set<std::string> names;
  names.insert(self_components_.begin(), self_components_.end());
  for (const auto& kvp : modules_) {
    const auto components = kvp.second->components();
    names.insert(components.begin(), components.end());
  }
  return names;
}

void ModuleManager::setModuleWorkingPath(const std::string& working_path) {
  working_path_ = working_path;
}

void ModuleManager::appendModulePaths(const std::vector<std::string>& paths) {
  module_paths_.insert(module_paths_.end(), paths.begin(), paths.end());
}

void ModuleManager::start(Application* app) {
  for (auto& kvp : modules_) {
    kvp.second->start(app);
  }
}

void ModuleManager::initialize(Application* app) {
  for (auto& kvp : modules_) {
    kvp.second->initialize(app);
  }
}

void ModuleManager::stop(Application* app) {
  for (auto& kvp : modules_) {
    kvp.second->stop(app);
  }
}

}  // namespace alice
}  // namespace isaac
