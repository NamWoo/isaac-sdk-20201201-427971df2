/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/backend/codelet_canister.hpp"

#include <map>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <string>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/component_backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/backend/node_canister.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/alice/components/Scheduling.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

namespace {

// Rounds a number to at most two digits after the point
double Round2(double x) {
  return std::round(x * 100.0) / 100.0;
}

}  // namespace

CodeletCanister::CodeletCanister(NodeCanister* node_canister, Codelet* codelet)
    : node_canister_(node_canister), codelet_(codelet), is_ready_to_start_(false),
      is_started_(false), ledger_(nullptr), scheduler_(nullptr) {
  ASSERT(codelet_ != nullptr, "Codelet must not be null");
  ASSERT(codelet_->canister_ == nullptr, "Codelet must not already have a canister");
  codelet_->canister_ = this;
  codelet_full_name_ = codelet_->full_name();

  tick_data_.count = 0;
  tick_data_.timestamp = 0;
  tick_data_.last_timestamp = 0;

  codelet_->initialize();
}

CodeletCanister::~CodeletCanister() {
  ASSERT(!is_started_, "Codelet '%s' was not stopped after it was started",
         codelet_full_name_.c_str());
  codelet_->deinitialize();
  codelet_->canister_ = nullptr;
}

void CodeletCanister::prepareToStart(scheduler::Scheduler* scheduler) {
  ASSERT(scheduler != nullptr, "Scheduler must not be null");
  ASSERT(!is_started_, "Codelet '%s' already started", codelet_full_name_.c_str());
  ASSERT(!is_ready_to_start_, "Codelet '%s' already prepared", codelet_full_name_.c_str());

  codelet()->node()->app()->backend()->node_backend()->reportStatusUpdate({
    NowCount(), codelet()->node()->name(), codelet()->name(), LifecycleStage::kConstructed,
    LifecycleStage::kPreStart
  });

  scheduler_ = scheduler;
  ledger_ = codelet_->node()->getComponentOrNull<MessageLedger>();

  // Reset the status to clear information from previous runs
  codelet_->status_ = Status::RUNNING;

  // Prepare for start
  codelet_->node()->config().updateHooks(codelet_);

  tick_data_.count = 0;
  tick_data_.timestamp = codelet_->node()->clock()->timestamp();
  tick_data_.last_timestamp = tick_data_.timestamp;

  is_ready_to_start_ = true;
}

void CodeletCanister::start() {
  ASSERT(!is_started_, "Codelet '%s' already started", codelet_full_name_.c_str());
  ASSERT(is_ready_to_start_, "Codelet '%s' not yet prepared", codelet_full_name_.c_str());

  // Remove ready flag
  is_ready_to_start_ = false;

  // Start codelet
  codelet_->start();
  is_started_ = true;
  codelet()->node()->app()->backend()->node_backend()->reportStatusUpdate({
    NowCount(), codelet()->node()->name(), codelet()->name(), LifecycleStage::kPreStart,
    LifecycleStage::kStarted
  });

  // Schedule codelet for ticking. This must happen last and especially after starting the codelet.
  // Once the codelet is scheduled it may tick anytime.
  addToScheduler();
}

void CodeletCanister::stop() {
  ASSERT(is_started_, "Codelet '%s' was not started", codelet_full_name_.c_str());
  codelet()->node()->app()->backend()->node_backend()->reportStatusUpdate({
    NowCount(), codelet()->node()->name(), codelet()->name(), LifecycleStage::kStarted,
    LifecycleStage::kPreStopped
  });

  // Unschedule the job and wait until it is unscheduled. This will guarantee that the codelet does
  // not tick anymore afterwards.
  if (job_handle_) {
    scheduler_->destroyJobAndWait(*job_handle_);
    preserved_statistics_ = getStatistics();
    job_handle_ = std::nullopt;
  }
  scheduler_ = nullptr;

  // Prepare for stop
  codelet_->node()->config().updateDirtyHooks(codelet_);

  // TODO(dweikersdorf) Probably should not update tick timestamp
  tick_data_.last_timestamp = tick_data_.timestamp;
  tick_data_.timestamp = codelet_->node()->clock()->timestamp();

  is_started_ = false;  // Called before stop so that potential calls to onTickParametersChanged
                        // do not trigger a call to addToScheduler.
  codelet_->stop();

  codelet()->node()->app()->backend()->node_backend()->reportStatusUpdate({
    NowCount(), codelet()->node()->name(), codelet()->name(), LifecycleStage::kPreStopped,
    LifecycleStage::kStopped
  });
}

void CodeletCanister::onTickParametersChanged() {
  // If the codelet was already scheduled the corresponding job is destroyed.
  if (job_handle_) {
    scheduler_->destroyJobAndWait(*job_handle_);
    preserved_statistics_ = getStatistics();
    job_handle_ = std::nullopt;
  }
  // In case the codelet was already started we re-schedule. Otherwise scheduling will happen at
  // a later point in time.
  if (is_started_) {
    addToScheduler();
  }
}

nlohmann::json CodeletCanister::getStatistics() const {
  if (!job_handle_) {
    return preserved_statistics_;
  }
  auto stats = scheduler_->getJobStatistics(*job_handle_);
  // Update stats to the current time to get better statistics
  const double time = ToSeconds(codelet_->node()->clock()->timestamp());
  stats.current_load.updateTime(time);
  stats.current_rate.updateTime(time);
  // Get statistics as JSON
  nlohmann::json json;
  json["average_exec_dt"] = Round2(1000.0 * stats.exec_dt.value());
  json["average_late_dt"] = Round2(1000.0 * stats.getAverageOverrunTime());
  json["late_p"] = Round2(100.0 * stats.getOverrunPercentage());
  json["load"] = Round2(100.0 * stats.current_load.rate());
  json["frequency"] = Round2(stats.current_rate.rate());
  json["dt"] = Round2(1000.0 * codelet_->getTickDt());
  json["num_ticks"] = codelet_->getTickCount();
  json["total_time"] = ToSeconds(stats.total_time);
  json["median"] = Round2(1000.0 * stats.execution_time_median.median());
  json["p90"] = Round2(1000.0 * stats.execution_time_median.percentile(0.9));
  json["max"] = Round2(1000.0 * stats.execution_time_median.max());
  return json;
}

void CodeletCanister::setCodeletStatus(Status status) {
  codelet_->status_ = status;
}

void CodeletCanister::addToScheduler() {
  ASSERT(job_handle_ == std::nullopt, "Codelet already scheduled");
  ASSERT(scheduler_ != nullptr, "Scheduler not set");

  scheduler::JobDescriptor job_descriptor;
  job_descriptor.name = codelet_full_name_;
  job_descriptor.groups.push_back(codelet_->node()->name());

  // Set the scheduling parameters
  Scheduling* scheduling = codelet_->node()->getComponentOrNull<Scheduling>();
  if (scheduling) {
    job_descriptor.slack = SecondsToNano(scheduling->get_slack());
    if (auto deadline = scheduling->try_get_deadline()) {
      job_descriptor.deadline = SecondsToNano(*deadline);
    }
    job_descriptor.priority = scheduling->get_priority();
    job_descriptor.execution_group = scheduling->get_execution_group();
  } else {
    job_descriptor.slack = 0;
    job_descriptor.priority = 0;
    job_descriptor.execution_group = "";
  }

  // Execute the tick of the codelet as an action
  job_descriptor.action = [this] { tick(); };

  if (!codelet_->triggers_.empty()) {
    job_descriptor.execution_mode = scheduler::ExecutionMode::kEventTask;
    // Always allow one extra event to trigger so we don't miss work.
    job_descriptor.event_trigger_limit = codelet_->get_execution_queue_limit();
  } else if (codelet_->tick_period_ == 0) {
    job_descriptor.execution_mode = scheduler::ExecutionMode::kBlocking;
  } else if (codelet_->tick_period_ > 0) {
    job_descriptor.execution_mode = scheduler::ExecutionMode::kPeriodicTask;
    job_descriptor.period = codelet_->tick_period_;
    if (!job_descriptor.deadline) {
      job_descriptor.deadline = codelet_->tick_period_;
    } else if (job_descriptor.deadline && job_descriptor.deadline > codelet_->tick_period_) {
      LOG_WARNING("Deadline parameter set greater than tick period. Tick period used instead.");
      job_descriptor.deadline = codelet_->tick_period_;
    }
  } else {
    LOG_WARNING("Codelet '%s' was not added to scheduler because no tick method is specified.",
                codelet_full_name_.c_str());
    return;
  }
  job_handle_ = scheduler_->createJob(job_descriptor);
  if (job_handle_) {
    if (!codelet_->triggers_.empty()) {
      scheduler_->registerEvents(*job_handle_, codelet_->triggers_);
    }
    scheduler_->startJob(*job_handle_);
  } else {
    LOG_ERROR("Could node schedule codelet");
  }
}

void CodeletCanister::tick() {
  ASSERT(is_started_, "Codelet '%s' was not started", codelet_full_name_.c_str());

  // Do not tick codelets which are not running anymore.
  if (codelet_->getStatus() != Status::RUNNING) return;

  // Update messages for receiving hooks and only tick if there is something to tick off.
  if (!updateRx()) {
    return;
  }

  // Prepare for tick
  codelet_->node()->config().updateDirtyHooks(codelet_);

  tick_data_.count++;
  tick_data_.last_timestamp = tick_data_.timestamp;
  tick_data_.timestamp = codelet_->node()->clock()->timestamp();

  // Tick the codelet
  codelet_->tick();
}

bool CodeletCanister::updateRx() {
  ASSERT(codelet_ != nullptr, "Codelet pointer must not be null");

  for (auto& kvp : codelet_->rx_hook_trackers_) {
    kvp.second.rx->queue().sync();
  }

  // A codelet is considered to be ready for execution either when it ticks periodically and
  // does not have any triggers, or if the trigger check was disabled explicitly.
  bool is_triggered = codelet_->triggers_.empty() || codelet_->non_rx_triggered_;
  if (ledger_ == nullptr) {
    return is_triggered;
  }
  for (auto& kvp : codelet_->rx_hook_trackers_) {
    auto& tracker = kvp.second;
    ASSERT(tracker.rx, "Encountered null pointer");
    ASSERT(tracker.rx->component() == codelet_, "RX component is not identical to this codelet");
    // find all new messages and pass them to all synchronizers
    // if there is no synchronizer, then just put the message directly into the hook
    auto& queue = tracker.rx->queue();
    for (size_t i = 0; i < queue.size(); i++) {
      const auto& message = queue.peek(i);
      ASSERT(message, "Encountered null pointer");
      if (!tracker.last_timestamp || message->pubtime > *tracker.last_timestamp) {
        tracker.last_timestamp = message->pubtime;
        // give message to synchronizer
        bool found_sync = false;
        for (auto& sync : codelet_->synchronizers_) {
          if (sync->contains(tracker.rx->tag())) {
            sync->push(tracker.rx->tag(), message);
            found_sync = true;
          }
        }
        if (!found_sync) {
          // set message directly
          tracker.rx->setMessage(message);
          is_triggered = true;
        }
      }
    }
  }
  // check synchronizers for new message
  std::map<std::string, alice::ConstMessageBasePtr> messages;
  for (const auto& sync : codelet_->synchronizers_) {
    while (sync->sync_pop(messages)) {
      for (const auto& kvp : messages) {
        codelet_->rx_hook_trackers_[kvp.first].rx->setMessage(kvp.second);
        is_triggered = true;
      }
    }
  }
  return is_triggered;
}

}  // namespace alice
}  // namespace isaac
