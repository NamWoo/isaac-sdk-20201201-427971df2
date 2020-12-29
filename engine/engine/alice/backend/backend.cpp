/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "backend.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/allocator_backend.hpp"
#include "engine/alice/backend/any_storage.hpp"
#include "engine/alice/backend/clock.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/error_handler.hpp"
#include "engine/alice/backend/event_manager.hpp"
#include "engine/alice/backend/failsafe_backend.hpp"
#include "engine/alice/backend/message_ledger_backend.hpp"
#include "engine/alice/backend/modules.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/backend/post_execution_report.hpp"
#include "engine/alice/backend/redis_backend.hpp"
#include "engine/alice/backend/sight_backend.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "engine/core/optional.hpp"
#include "engine/core/singleton.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

class Backend::SchedulerClock : public scheduler::Clock {
 public:
  SchedulerClock(alice::Clock* clock) : clock_(clock) {}
  int64_t now() override { return clock_->timestamp(); }
  void advance(int64_t dt) override { clock_->advance(dt); }

 private:
  alice::Clock* clock_;
};

Backend::Backend(Application* app, const nlohmann::json& backend_json) : app_(app) {
  redis_backend_ = std::make_unique<RedisBackend>(app);

  allocator_backend_ = std::make_unique<AllocatorBackend>(app);

  error_handler_ = std::make_unique<ErrorHandler>();
  module_manager_ = std::make_unique<ModuleManager>();

  any_storage_ = std::make_unique<AnyStorage>();

  // Manually bring up the clock backend.
  clock_ = std::make_unique<Clock>();
  // Bring up the scheduler first and bring it down last.
  nlohmann::json scheduler_json = {};
  const auto it_scheduler = backend_json.find("scheduler");
  if (it_scheduler != backend_json.end()) {
    scheduler_json = *it_scheduler;
  }
  scheduler_clock_ = std::make_unique<SchedulerClock>(clock_.get());
  scheduler_ = std::make_unique<scheduler::Scheduler>(parseExecutionGroupConfig(scheduler_json),
                                                      scheduler_clock_.get());
  configureSchedulerClock(scheduler_json, clock_.get(), scheduler_.get());

  event_manager_ = std::make_unique<EventManager>(app);

  config_backend_ = addBackend<ConfigBackend>();

  pose_tree_ = nullptr;

  sight_backend_ = std::make_unique<SightBackend>();

  message_ledger_backend_ = addBackend<MessageLedgerBackend>();

  auto* failsafe_backend = addBackend<FailsafeBackend>();
  addBackend<FailsafeHeartbeatBackend>(failsafe_backend);

  node_backend_ = std::make_unique<NodeBackend>(app_);

  needs_stop_ = false;
}

Backend::~Backend() {
  stop();
  node_backend_->destroy();
}

void Backend::addBackend(std::unique_ptr<ComponentBackendBase> backend) {
  backend->app_ = app_;
  backends_.emplace_back(std::move(backend));
}

void Backend::prepare() {
  // create a node to run backend components.
  // FIXME(dweikersdorf) The timing of this function is a problem and needs further consideration.
  Node* backend_node = node_backend_->createMessageNode("backend");
  pose_tree_ = backend_node->addComponent<isaac::alice::PoseTree>("pose_tree");
}

void Backend::start() {
  clock_->start();
  scheduler_->start();
  allocator_backend_->start();  // needs scheduler

  module_manager_->start(app_);

  for (auto& ptr : backends_) {
    ptr->start();
  }

  redis_backend_->start();

  node_backend_->start();

  needs_stop_ = true;
}

void Backend::stop() {
  if (!needs_stop_) {
    return;
  }
  LOG_INFO("Backend is shutting down...");
  // FIXME check that we started?
  node_backend_->stop();
  for (auto& ptr : backends_) {
    ptr->stop();
  }

  redis_backend_->stop();

  module_manager_->stop(app_);

  scheduler_->stop();
  clock_->stop();
  allocator_backend_->stop();

  needs_stop_ = false;
  LOG_INFO("Backend is shutting down... DONE");

  // Print statistics
  for (const auto& kvp : StringifyBackendStatistics(getStatistics())) {
    LOG_INFO("%s:\n%s\n", kvp.first.c_str(), kvp.second.c_str());
  }
}

nlohmann::json Backend::getStatistics() const {
  nlohmann::json result;
  result["codelets"] = node_backend()->getCodeletStatistics();
  return result;
}

std::vector<scheduler::ExecutionGroupDescriptor> Backend::parseExecutionGroupConfig(
    const nlohmann::json& scheduler_json) {
  // The working set for the scheduler execution groups.
  std::vector<scheduler::ExecutionGroupDescriptor> execution_groups;

  // Extracts the default group configuration if one exists. This configutaion
  // will be used to override any attempt and auto generating a default configation.
  auto default_execution_group_config = scheduler_json.find("default_execution_group_config");
  if (default_execution_group_config != scheduler_json.end()) {
    ASSERT(default_execution_group_config->is_array(), "Must be an array: %s",
           default_execution_group_config->dump(2).c_str());
    for (auto it = default_execution_group_config->begin();
         it != default_execution_group_config->end(); ++it) {
      auto worker_cores = serialization::TryGetFromMap<std::vector<int>>(*it, "worker_cores");
      auto blocker_cores = serialization::TryGetFromMap<std::vector<int>>(*it, "blocker_cores");
      if (worker_cores) {
        scheduler::ExecutionGroupDescriptor group;
        group.name = scheduler::Scheduler::kDefaultWorkerGroup;
        group.cores = *worker_cores;
        group.has_workers = true;
        execution_groups.push_back(group);
      }
      if (blocker_cores) {
        scheduler::ExecutionGroupDescriptor group;
        group.name = scheduler::Scheduler::kDefaultBlockerGroup;
        group.cores = *blocker_cores;
        group.has_workers = false;
        execution_groups.push_back(group);
      }
    }
  }

  // Extract specific execution group configurations.
  auto execution_group_config = scheduler_json.find("execution_group_config");
  if (execution_group_config == scheduler_json.end()) {
    if (execution_groups.empty()) {
      LOG_WARNING(
          "This application does not have an execution group configuration. "
          "One will be autogenerated to the best of the systems abilities if possible.");
    }
    return execution_groups;
  }

  ASSERT(execution_group_config->is_array(), "Must be an array: %s",
         execution_group_config->dump(2).c_str());
  for (auto it = execution_group_config->begin(); it != execution_group_config->end(); ++it) {
    auto& execution_group = *it;

    scheduler::ExecutionGroupDescriptor group;
    group.name = execution_group["name"];
    group.has_workers = execution_group["workers"];
    auto group_cores = serialization::TryGetFromMap<std::vector<int>>(execution_group, "cores");
    if (group_cores) {
      group.cores = *group_cores;
    }
    execution_groups.push_back(group);
  }

  return execution_groups;
}

void Backend::configureSchedulerClock(const nlohmann::json& scheduler_json, Clock* clock,
                                      scheduler::Scheduler* scheduler) {
  if (scheduler_json.is_null()) return;
  auto use_time_machine = scheduler_json.value("use_time_machine", false);
  auto clock_scale = scheduler_json.value("clock_scale", 1.0);
  if (use_time_machine) {
    scheduler->enableTimeMachine();
  }
  clock->setTimeScale(clock_scale);
}

}  // namespace alice
}  // namespace isaac
