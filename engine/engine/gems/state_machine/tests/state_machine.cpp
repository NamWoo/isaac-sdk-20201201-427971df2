/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/state_machine/state_machine.hpp"

#include <string>

#include "gtest/gtest.h"

namespace isaac {
namespace state_machine {

TEST(StateMachine, Test1) {
  // A basic state machine with two states which automatically transitions from the first to
  // the second state.
  state_machine::StateMachine<int> sm;
  sm.setToString([](int state_id) { return std::to_string(state_id); });
  sm.addState(0);
  sm.addState(1);
  sm.addTransition(0, 1, [] { return true; }, [] {});
  EXPECT_EQ(sm.current_state(), std::nullopt);
  sm.start(0);
  EXPECT_EQ(sm.current_state(), 0);
  sm.tick();
  EXPECT_EQ(sm.current_state(), 1);
  sm.stop();
  EXPECT_EQ(sm.current_state(), std::nullopt);
}

TEST(StateMachine, Test2) {
  // A basic state machine which starts in state 0 and could transition to state 1 or 2, but
  // the condition to state 1 is always blocked and thus it transitions to state 2.
  state_machine::StateMachine<int> sm;
  sm.setToString([](int state_id) { return std::to_string(state_id); });
  sm.addState(0);
  sm.addState(1);
  sm.addState(2);
  sm.addTransition(0, 1, [] { return false; }, [] {});
  sm.addTransition(0, 2, [] { return true; }, [] {});
  EXPECT_EQ(sm.current_state(), std::nullopt);
  sm.start(0);
  EXPECT_EQ(sm.current_state(), 0);
  sm.tick();
  EXPECT_EQ(sm.current_state(), 2);
  sm.stop();
  EXPECT_EQ(sm.current_state(), std::nullopt);
}

TEST(StateMachine, CountTo10) {
  // A simple state machine which counts to 10.
  enum class State { kInit, kAdd, kDone };
  int value = -1;
  state_machine::StateMachine<State> sm;
  sm.setToString([](State state_id) { return std::to_string(static_cast<int>(state_id)); });
  sm.addState(State::kInit, [&] { value = 0; }, [] {}, [] {});
  sm.addState(State::kAdd);
  sm.addState(State::kDone);
  sm.addTransition(State::kInit, State::kAdd, [&] { return true; }, [] {});
  sm.addTransition(State::kAdd, State::kAdd, [&] { return value < 10; }, [&] { value++; });
  sm.addTransition(State::kAdd, State::kDone, [&] { return value == 10; }, [] {});
  sm.start(State::kInit);
  for (int i = 0; i <= 10; i++) {
    sm.tick();
    EXPECT_EQ(sm.current_state(), State::kAdd);
    EXPECT_EQ(value, i);
  }
  for (int i = 0; i < 5; i++) {
    sm.tick();
    EXPECT_EQ(sm.current_state(), State::kDone);
    EXPECT_EQ(value, 10);
  }
  sm.stop();
}

}  // namespace state_machine
}  // namespace isaac
