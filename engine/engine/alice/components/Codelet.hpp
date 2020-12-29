/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "engine/alice/backend/stopwatch.hpp"
#include "engine/alice/backend/synchronization.hpp"
#include "engine/alice/component.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

class CodeletCanister;
class ChannelSynchronizer;

// Codelets are special components which allow the periodic execution of custom code. The user can
// create her own codelets by deriving from this class and overriding the functions initialize,
// start, tick, stop, and deinitialize.
class Codelet : public Component {
 public:
  virtual ~Codelet() = default;

  // This function is called during the start phase of the codelet. It allows derived classes to
  // execute custom code during the start phase. This is a good place to obtain resources which
  // are necessary for ticking the codelet. This function is guaranteed to be called before the
  // first call to tick.
  virtual void start() {}

  // This function is called whenever the codelet is expected to do work, e.g. when an event was
  // received or periodically. The tick method can be specified with various other member functions.
  // This function is the main work horse of the codelet.
  virtual void tick() {}

  // This function is called during the stop phase of the codelet. It allows derived classes to
  // execute custom code during the stop phase. This is a good place to clean up any resources which
  // where obtained during `start`. After the codelet is stopped it should be in the same state as
  // it was before `start` was called. Be careful to not leave any unintended left overs as `start`
  // might be called again afterwards. It is guaranteed that stop is called after the last
  // call to tick. When start was called stop will be called, too.
  virtual void stop() {}

  // Tick as fast as possible (assuming that the tick uses blocking calls)
  void tickBlocking();
  // Tick periodically with given interval read from config. See tick_period
  void tickPeriodically();
  // Legacy tickPeriodically interface.  Deprecated.
  void tickPeriodically(double interval);
  // Tick each time a new messages is received by the given receiver
  void tickOnMessage(const RxMessageHook& rx);
  // Ticks when an event happens
  void tickOnEvents(const std::unordered_set<std::string>& events);
  // Synchronizes channels so that messages are only received when timestamps match. This function
  // can be called multiple times to synchronize multiple channels.
  void synchronize(const RxMessageHook& rx1, const RxMessageHook& rx2);
  // Synchronizes the list of channels. It relies on the function above and is more a convenience
  // function.
  template <class ... T>
  void synchronize(const RxMessageHook& rx1, const RxMessageHook& rx2, const T&... rxs) {
    synchronize(rx1, rx2);
    synchronize(rx1, rxs...);
  }
  // Converts the tick unit from string to seconds
  std::optional<double> getTickPeriodAsSeconds();

  // Time at which the current tick started
  double getTickTime() const { return ToSeconds(getTickTimestamp()); }
  // Time duration between the start of the current and the previous tick
  double getTickDt() const;
  // Tick at which current tick started
  int64_t getTickTimestamp() const;

  // Returns true if this is the first tick after start
  bool isFirstTick() const { return getTickCount() == 1; }
  // Returns the number of times a codelet ticked
  int64_t getTickCount() const;
  // Returns a stopwatch for the given nameSleep tag.
  Stopwatch& stopwatch(const std::string& clock_name = "");

  // Helper function to show a variable with sight
  template <typename T, std::enable_if_t<std::is_arithmetic<std::decay_t<T>>::value, int> = 0>
  void show(const std::string& tag, T value) const {
    node()->sight().show(this, tag, getTickTime(), value);
  }
  // Helper function to show a variable with sight
  template <typename T, std::enable_if_t<std::is_arithmetic<std::decay_t<T>>::value, int> = 0>
  void show(const std::string& tag, double time, T value) const {
    node()->sight().show(this, tag, time, value);
  }
  // Helper function to show everything except a variable with sight
  template <typename T, std::enable_if_t<!std::is_arithmetic<std::decay_t<T>>::value, int> = 0>
  void show(const std::string& tag, T&& arg) const {
    node()->sight().show(this, tag, getTickTime(), std::forward<T>(arg));
  }
  // Helper function to show everything except a variable with sight
  template <typename T, std::enable_if_t<!std::is_arithmetic<std::decay_t<T>>::value, int> = 0>
  void show(const std::string& tag, double time, T&& arg) const {
    node()->sight().show(this, tag, time, std::forward<T>(arg));
  }

  // Helper function to set a variable
  void setVariable(const std::string& tag, double value) const {
    setVariable(tag, getTickTimestamp(), value);
  }
  void setVariable(const std::string& tag, int64_t timestamp, double value) const;
  // Helper function to get a variable
  std::optional<double> getVariable(const std::string& link) const {
    return getVariable(link, getTickTimestamp());
  }
  std::optional<double> getVariable(const std::string& link, int64_t timestamp) const;

  // Marks this codelet as successful. The codelet will stop, but might start again afterwards.
  void reportSuccess(const char* message, ...);
  // Special version of `reportSuccess` which does not require a message.
  void reportSuccess() { reportSuccess(""); }
  // Marks this codelet as failed. The codelet will stop, but might start again afterwards.
  void reportFailure(const char* message, ...);
  // Special version of `reportFailure` which does not require a message.
  void reportFailure() { reportFailure(""); }
  // Same as `reportSuccess` or `reportFailure` depending on the value of `new_status`.
  void updateStatus(Status new_status, const char* message, ...);
  // Special version of `updateStatus` which does not require a message.
  void updateStatus(Status new_status) { updateStatus(new_status, ""); }
  // Gets the current status of the comopnent
  Status getStatus() const { return status_; }
  // Gets the message associated with the last status update
  std::string getStatusMessage() const { return status_message_; }

  // Internal usage // TODO: protect this function better
  void addRx(RxMessageHook* rx) {
    rx_hook_trackers_[rx->tag()] = {rx};
  }

  // Config parameter for setting tick period.  Units support are s, ms, and hz
  // If no unit is specified seconds are assumed.
  ISAAC_PARAM(std::string, tick_period);

  // In case the codelet is triggered while already running it is queued at most the given number
  // of times. If set to -1 no limit will be applied. Using no limit is discouraged as it can lead
  // to queues growing without bound and thus unbounded memory usage. By default the codelet
  // can be queued at most once. This allows it to fire again immediately after it is done with
  // execution to react to data which arrived while it was processing previous data.
  ISAAC_PARAM(int, execution_queue_limit, 1);

 private:
  friend class CodeletCanister;

  void onTickParametersChanged();

  // Implementation of updateStatus
  void updateStatusImpl(Status new_status, const char* message, va_list args);

  CodeletCanister* canister_ = nullptr;

  int64_t tick_period_ = -1;
  std::unordered_set<std::string> triggers_;
  bool non_rx_triggered_ = false;

  std::unordered_map<std::string, Stopwatch> stopwatches_;

  struct RxHookTracker {
    RxMessageHook* rx = nullptr;
    std::optional<int64_t> last_timestamp;
  };

  std::map<std::string, RxHookTracker> rx_hook_trackers_;
  std::set<std::unique_ptr<ChannelSynchronizer>> synchronizers_;

  // Run status
  Status status_ = Status::RUNNING;
  std::string status_message_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::alice::Codelet)
