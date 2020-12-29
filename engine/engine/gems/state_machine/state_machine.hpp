/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace state_machine {

// A basic state machine.
//
// The state machine is always in exactly one out of a set of user-define states. States are
// enumerated by a template type chosen by the user.
//
// For each state the user can specify three callback: entry, stay and exit. The entry function
// is called whenever the state machine enters the state. The stay function is called each time
// `tick` is called and the state machine stays in that state. The exit function is called whenever
// the state machine exits the state.
//
// In addition the user can add possible transitions between states. For each transition he can
// specify a condition and a transition function. During each `tick` the condition functions of the
// current state are evaluated and the first condition which is true will trigger a transition
// into a new state.
template <typename StateId>
class StateMachine {
 public:
  using Condition = std::function<bool()>;
  using Action = std::function<void()>;

  // Sets a function to convert a state identifier to a string
  void setToString(std::function<std::string(const StateId&)> to_string) {
    to_string_ = to_string;
  }

  // Adds a new state and specifies the entry, stay and exit callbacks
  void addState(const StateId& state, Action&& on_entry_action, Action&& on_stay_action,
                Action&& on_exit_action) {
    ASSERT(states_.find(state) == states_.end(), "State '%s' already added",
           toString(state).c_str());
    states_[state] = State{
      std::forward<Action>(on_entry_action),
      std::forward<Action>(on_stay_action),
      std::forward<Action>(on_exit_action),
      {}
    };
  }
  // Adds a new state with empty entry, stay and exit callbacks
  void addState(const StateId& state) {
    addState(state, [] {}, [] {}, [] {});
  }

  // Adds a transition from `state_` to `state_2` and specifies the condition and transition
  // callback functions.
  void addTransition(const StateId& state_1, const StateId& state_2, Condition&& condition,
                     Action&& on_transition_action) {
    findState(state_1).transitions.emplace_back(Transition{
      state_2,
      std::forward<Condition>(condition),
      std::forward<Action>(on_transition_action)
    });
  }

  // Initializes the state machine and enters the given state
  void start(const StateId& state) {
    enterState(state);
  }

  // Stops the state machine (and leaves the current state; if any)
  void stop() {
    exitState();
  }

  // Ticks the state machine possibly calling callbacks
  void tick() {
    ASSERT(current_state_, "State machine not started or already stopped.");
    auto& state = findState(*current_state_);
    for (auto& transition : state.transitions) {
      if (transition.condition()) {
        LOG_DEBUG("Transition '%s' -> '%s'", toString(*current_state_).c_str(),
                  toString(transition.target_state).c_str());
        exitState();
        transition.on_transition_action();
        enterState(transition.target_state);
        return;
      }
    }
    // No state transition thus we execute the stay action
    state.on_stay_action();
  }

  // Gets the current state of the state machine
  std::optional<StateId> current_state() const { return current_state_; }

  // Converts a state to a descriptive string
  std::string toString(const StateId& state_id) const {
    if (to_string_) {
      return to_string_(state_id);
    } else {
      return std::string("N/A");
    }
  }

 private:
  // A transition between two states
  struct Transition {
    StateId target_state;
    Condition condition;
    Action on_transition_action;
  };

  // A state in which the state machine can be
  struct State {
    Action on_entry_action;
    Action on_stay_action;
    Action on_exit_action;
    std::vector<Transition> transitions;
  };

  // Finds a state by identifier
  State& findState(const StateId& state_id) {
    auto it = states_.find(state_id);
    ASSERT(it != states_.end(), "Unknown state '%s'", toString(state_id).c_str());
    return it->second;
  }

  // Enters the given state assuming we are currently not in any state
  void enterState(const StateId& state_id) {
    ASSERT(!current_state_, "State machine can not enter state '%s' because it has not exited from "
           "state '%s'", toString(state_id).c_str(), toString(*current_state_).c_str());
    findState(state_id).on_entry_action();
    current_state_ = state_id;
  }

  // Exits the current state
  void exitState() {
    ASSERT(current_state_, "State machine can not exit from state because it is not in a state");
    findState(*current_state_).on_exit_action();
    current_state_ = std::nullopt;
  }

  // Function to convert a state identifier to a string
  std::function<std::string(const StateId&)> to_string_;

  // Dictionary with all states. Transitions are stored as part of the source state.
  std::map<StateId, State> states_;

  // The identifier of the current state of the state machine
  std::optional<StateId> current_state_;
};

}  // namespace state_machine
}  // namespace isaac
