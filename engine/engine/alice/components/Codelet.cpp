/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Codelet.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/any_storage.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/codelet_canister.hpp"
#include "engine/alice/backend/event_manager.hpp"
#include "engine/alice/backend/synchronization.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/time.hpp"

namespace isaac {
namespace alice {

void Codelet::tickBlocking() {
  tick_period_ = 0;
  triggers_.clear();
  non_rx_triggered_ = false;
  onTickParametersChanged();
}

void Codelet::tickPeriodically() {
  auto period = getTickPeriodAsSeconds();
  if (!period) {
    reportFailure("The tick_period of codelet '%s' must be specified to use tickPeriodically()",
                  full_name().c_str());
    return;
  }
  tick_period_ = SecondsToNano(*period);
  triggers_.clear();
  non_rx_triggered_ = false;
  onTickParametersChanged();
}

void Codelet::tickPeriodically(double interval) {
  LOG_WARNING("Function deprecated. Set tick_period to the desired tick parameter");
  set_tick_period(std::to_string(interval));
  tickPeriodically();
}

std::optional<double> Codelet::getTickPeriodAsSeconds() {
  std::string period_str = get_tick_period();

  // Convert string to standard lower form.
  std::transform(period_str.begin(), period_str.end(), period_str.begin(), ::tolower);

  // Parse the first string
  char* suffix;
  errno = 0;
  const double value = std::strtod(period_str.c_str(), &suffix);
  if (errno != 0 || !std::isfinite(value)) {
    LOG_ERROR("Tick period '%s' for codelet '%s' is not a number", period_str.c_str(),
              full_name().c_str());
    return std::nullopt;
  }
  errno = 0;

  // Value must be positive
  if (value <= 0.0) {
    LOG_ERROR("Tick period '%s' for codelet '%s' must be positive", period_str.c_str(),
              full_name().c_str());
    return std::nullopt;
  }

  // Check for unit type.
  const size_t idx_hz = period_str.find("hz", suffix - period_str.c_str());
  const size_t idx_ms = period_str.find("ms", suffix - period_str.c_str());
  if (idx_hz != std::string::npos && idx_ms != std::string::npos) {
    LOG_ERROR("Invalid tick period '%s' for codelet '%s'", period_str.c_str(), full_name().c_str());
    return std::nullopt;
  } else if (idx_hz != std::string::npos) {
    // If unit is Hz need to convert final result to seconds by inverting the value.
    return 1.0 / value;
  } else if (idx_ms != std::string::npos) {
    // If unit is ms the final result must be divided by 1000
    return 0.001 * value;
  } else {
    // If no unit specified the value is interpret as a duration in seconds
    return value;
  }
}

double Codelet::getTickDt() const {
  return ToSeconds(canister_->getTickTimestampDelta());
}

int64_t Codelet::getTickTimestamp() const {
  return canister_->getTickTimestamp();
}

int64_t Codelet::getTickCount() const {
  return canister_->getTickCount();
}

void Codelet::tickOnMessage(const RxMessageHook& rx) {
  ASSERT(rx.component() == this, "Can not tick on a message hook from another codelet");
  triggers_.insert(rx.channel_id());
  tick_period_ = -1;
  onTickParametersChanged();
}

void Codelet::tickOnEvents(const std::unordered_set<std::string>& events) {
  tick_period_ = -1;
  triggers_ = events;
  non_rx_triggered_ = true;
  onTickParametersChanged();
}

void Codelet::synchronize(const RxMessageHook& rx1, const RxMessageHook& rx2) {
  ASSERT(rx1.component() == this, "Can not synchronize with a message hook from another codelet");
  ASSERT(rx2.component() == this, "Can not synchronize with a message hook from another codelet");
  ASSERT(rx1.tag() != rx2.tag(), "Can not synchronize a channel with itself");
  auto sync_rx1 = synchronizers_.end();
  auto sync_rx2 = synchronizers_.end();
  // Check if rx1 or rx2 belong to a group already
  for (auto it = synchronizers_.begin(); it != synchronizers_.end(); ++it) {
    if ((*it)->contains(rx1.tag())) {
      sync_rx1 = it;
    }
    if ((*it)->contains(rx2.tag())) {
      sync_rx2 = it;
    }
  }
  if (sync_rx1 == synchronizers_.end() && sync_rx2 == synchronizers_.end()) {
    // If none of them belong to a group add a new one
    auto uptr = std::make_unique<ChannelSynchronizer>();
    uptr->mark(rx1.tag());
    uptr->mark(rx2.tag());
    synchronizers_.insert(std::move(uptr));
  } else if (sync_rx1 == synchronizers_.end()) {
    // If rx1 doesn't belong to a group, add it to the group for rx2
    (*sync_rx2)->mark(rx1.tag());
  } else if (sync_rx2 == synchronizers_.end()) {
    // If rx2 doesn't belong to a group, add it to the group for rx1
    (*sync_rx1)->mark(rx2.tag());
  } else if (sync_rx1 != sync_rx2) {
    // If they don't belong to the same group, we merge the group of rx2 in rx1 and erase it.
    (*sync_rx1)->merge(std::move(*(*sync_rx2).get()));
    synchronizers_.erase(sync_rx2);
  }
}

void Codelet::onTickParametersChanged() {
  ASSERT(canister_ != nullptr,
         "Codelet is not setup properly. Did you called tickBlocking, tickPeriodically, or "
         "tickOnMessage in the constructor?");
  canister_->onTickParametersChanged();
}

Stopwatch& Codelet::stopwatch(const std::string& name) {
  Stopwatch& stopwatch = stopwatches_[name];
  if (!stopwatch.valid()) {
    stopwatch.setClock(node()->clock());
  }
  return stopwatch;
}

void Codelet::setVariable(const std::string& tag, int64_t timestamp, double value) const {
  // FIXME use timestamp
  node()->app()->backend()->any_storage()->set(full_name() + "/" + tag, value);  // NOLINT
  show(tag, ToSeconds(timestamp), value);
}

std::optional<double> Codelet::getVariable(const std::string& link, int64_t timestamp) const {
  // FIXME use timestamp
  return node()->app()->backend()->any_storage()->tryGet(link);
}

void Codelet::reportSuccess(const char* message, ...) {
  va_list args;
  va_start(args, message);
  updateStatusImpl(Status::SUCCESS, message, args);
  va_end(args);
}

void Codelet::reportFailure(const char* message, ...) {
  va_list args;
  va_start(args, message);
  updateStatusImpl(Status::FAILURE, message, args);
  va_end(args);
}

void Codelet::updateStatus(Status new_status, const char* message, ...) {
  ASSERT(new_status == Status::SUCCESS || new_status == Status::FAILURE,
         "updateStatus can only be called with SUCCESS or FAILURE");
  va_list args;
  va_start(args, message);
  updateStatusImpl(new_status, message, args);
  va_end(args);
}

void Codelet::updateStatusImpl(Status new_status, const char* message, va_list args) {
  // Check that the status is only updated when the component is alive
  const auto stage = node()->getStage();
  ASSERT(stage == LifecycleStage::kPreStart || stage == LifecycleStage::kStarted
         || stage == LifecycleStage::kPreStopped , "reportSuccess or reportFailure can only be "
         "called in start, stop, or tick. component: %s, message: %s", full_name().c_str(),
         message);
  status_ = new_status;

  // Create the message
  va_list args2;
  va_copy(args2, args);
  std::vector<char> buffer(1 + std::vsnprintf(NULL, 0, message, args2), '\0');
  va_end(args2);
  std::vsnprintf(buffer.data(), buffer.size(), message, args);
  status_message_ = std::string(buffer.data(), buffer.size());

  // Print error messages to the console. We might want to revisit this later.
  if (new_status == Status::FAILURE) {
    LOG_ERROR("Component '%s' of type '%s' reported FAILURE:\n\n"
              "    %s\n", full_name().c_str(), type_name().c_str(), status_message_.c_str());
  }

  node()->app()->backend()->event_manager()->onStatusUpdate(this);
}

}  // namespace alice
}  // namespace isaac
