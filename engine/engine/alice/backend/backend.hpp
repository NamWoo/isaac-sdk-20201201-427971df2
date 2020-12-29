/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <memory>
#include <utility>
#include <vector>

#include "engine/alice/backend/component_backend.hpp"
#include "engine/gems/scheduler/execution_group_descriptor.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace scheduler {
class Scheduler;
}
}  // namespace isaac

namespace isaac {
namespace alice {

class AllocatorBackend;
class AnyStorage;
class Application;
class Clock;
class ConfigBackend;
class ErrorHandler;
class EventManager;
class MessageLedgerBackend;
class ModuleManager;
class NodeBackend;
class PoseTree;
class SightBackend;
class RedisBackend;

// Manages all backends
class Backend {
 public:
  Backend(Application* app, const nlohmann::json& backend_json);
  ~Backend();

  AllocatorBackend* allocator_backend() const { return allocator_backend_.get(); }
  AnyStorage* any_storage() const { return any_storage_.get(); }
  Clock* clock() const { return clock_.get(); }
  ConfigBackend* config_backend() const { return config_backend_; }
  ErrorHandler* error_handler() const { return error_handler_.get(); }
  EventManager* event_manager() const { return event_manager_.get(); }
  MessageLedgerBackend* message_ledger_backend() const { return message_ledger_backend_; }
  ModuleManager* module_manager() const { return module_manager_.get(); }
  NodeBackend* node_backend() const { return node_backend_.get(); }
  PoseTree* pose_tree() const { return pose_tree_; }
  scheduler::Scheduler* scheduler() const { return scheduler_.get(); }
  SightBackend* sight_backend() const { return sight_backend_.get(); }
  RedisBackend* redis_backend() const { return redis_backend_.get(); }

  // Adds a component backend (generic version)
  template <typename T, typename... Args>
  T* addBackend(Args&&... args) {
    auto uptr = std::make_unique<T>(std::forward<Args>(args)...);
    auto* ptr = uptr.get();
    addBackend(std::move(uptr));
    return ptr;
  }
  void addBackend(std::unique_ptr<ComponentBackendBase> backend);

  // Gets a backend of given type. Will panic if no backend with that type exists.
  template <typename T>
  T* getBackend() {
    for (const auto& uptr : backends_) {
      T* ptr = dynamic_cast<T*>(uptr.get());
      if (ptr) {
        return ptr;
      }
    }
    PANIC("No backend of desired type");
  }

  // Prepares the backend. Needs to be called before start, but before any other nodes are added.
  void prepare();

  // Starts the backend
  void start();
  // Stops the backend
  void stop();

  // Gets statistics for all backend
  nlohmann::json getStatistics() const;

 private:
  class SchedulerClock;

  // Parse the scheduler configuration portion of the JSON app configuration.
  std::vector<scheduler::ExecutionGroupDescriptor> parseExecutionGroupConfig(
      const nlohmann::json& scheduler_json);
  // Configure the clock to use a time scale or the scheduler to use the time machine
  void configureSchedulerClock(const nlohmann::json& scheduler_json, Clock* clock,
                               scheduler::Scheduler* scheduler);

  Application* app_;

  std::unique_ptr<AllocatorBackend> allocator_backend_;
  std::unique_ptr<ErrorHandler> error_handler_;
  std::unique_ptr<ModuleManager> module_manager_;

  std::unique_ptr<AnyStorage> any_storage_;
  std::unique_ptr<EventManager> event_manager_;
  std::unique_ptr<NodeBackend> node_backend_;
  std::unique_ptr<SightBackend> sight_backend_;
  std::unique_ptr<Clock> clock_;

  std::unique_ptr<scheduler::Scheduler> scheduler_;
  std::unique_ptr<SchedulerClock> scheduler_clock_;

  std::unique_ptr<RedisBackend> redis_backend_;

  ConfigBackend* config_backend_;
  PoseTree* pose_tree_;
  MessageLedgerBackend* message_ledger_backend_;

  std::vector<std::unique_ptr<ComponentBackendBase>> backends_;

  bool needs_stop_;
};

}  // namespace alice
}  // namespace isaac
