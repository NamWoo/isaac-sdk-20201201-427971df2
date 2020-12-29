/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/components/Codelet.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

class NodeCanister;

// A helper class which handles the life cylce of a codelet.
//
// Every codelet is wrapped into a canister after it is constructed. The canister takes care of
// calling the life cycle functions like initialize, start, tick, stop and deinitialize. It sets up
// scheduling for the codelet and prepares the codelet accordingly before calling the start, tick,
// or stop functions. It currently also handles message synchronization. The canister does not
// construct or delete the codelet.
class CodeletCanister {
 public:
  // Creates a new canister for the given codelet. At this pointer the codelet should only be
  // constructed. The canister will handle all further life cycle functions.
  CodeletCanister(NodeCanister* node_canister, Codelet* codelet);

  // Destroys the canister. The canister will stop and deinitialize the codelet. It will however not
  // deconstruct it, i.e. it will not call delete on the pointer.
  ~CodeletCanister();

  // Returns a pointer to the codelet inside the canister
  Codelet* codelet() const { return codelet_; }

  // This function must be called whenever the tick parameters of the codelet have changed. This
  // function will notify the scheduler accordingly.
  void onTickParametersChanged();

  // Prepares the underlying codelet for starting. Must be called before start(). This sets the
  // scheduler of this canister, resets the codelet's status to Status::RUNNING, hooks up the
  // codelet to the governing node instance and resets the internal tick data. No communication with
  // the node backend is happening here to make sure this canister is in a defined and ready to
  // start state before interactive with asynchronous effects outside of this instance.
  void prepareToStart(scheduler::Scheduler* scheduler);

  // Starts the underlying codelet. Must be called before stop(). prepareToStart() must be called
  // prior to calling this. This method starts the contained codelet, reports the updated status to
  // the node backend, and actually adds the codelet to the scheduler set by prepareToStart().
  void start();

  // Timestamp at which current tick started
  int64_t getTickTimestamp() const { return tick_data_.timestamp; }
  // Time duration between the start of the current and the previous tick
  int64_t getTickTimestampDelta() const { return tick_data_.timestamp - tick_data_.last_timestamp; }
  // Returns the number of times a codelet ticked in the current start/stop cycle
  size_t getTickCount() const { return tick_data_.count; }

  // Stops the underlying codelet. Must be called after start.
  void stop();

  // Gets statistics as JSON
  nlohmann::json getStatistics() const;

  // Sets the status of the contained codelet to `status`.
  void setCodeletStatus(Status status);

 private:
  // Struct for trackining timing information for the codelet
  struct TickData {
    // Number of times the codelet has ticked. This will be 0 during start and 1 during the first
    // call in this tick cycle.
    size_t count;
    // Timestamp of the current tick in ticks
    int64_t timestamp;
    // Time of the previous tick in seconds
    int64_t last_timestamp;
  };

  // Schedules the codelet for execution in the backend scheduler.
  void addToScheduler();

  // Ticks the underlying codelet once. May not tick if the codelet is ticking on synchronized
  // messages and not set of synchronized messages is available.
  void tick();

  // Checks message channels for new messages, synchronizes messages, populates RX message channels.
  // Returns true if the codelet was triggered and should tick.
  bool updateRx();

  NodeCanister* node_canister_;
  Codelet* codelet_;
  std::string codelet_full_name_;

  bool is_ready_to_start_;
  bool is_started_;

  MessageLedger* ledger_;

  scheduler::Scheduler* scheduler_;
  std::optional<scheduler::JobHandle> job_handle_;

  TickData tick_data_;

  nlohmann::json preserved_statistics_;
};

}  // namespace alice
}  // namespace isaac
