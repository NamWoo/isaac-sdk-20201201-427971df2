/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"
#include "packages/behavior_tree/components/MemorySequenceBehavior.hpp"
#include "packages/behavior_tree/components/ParallelBehavior.hpp"
#include "packages/behavior_tree/components/RepeatBehavior.hpp"
#include "packages/behavior_tree/components/TimerBehavior.hpp"
#include "packages/behavior_tree/tests/utils.hpp"

namespace isaac {
namespace behavior_tree {

namespace {
// Global variable to test whether nodes start and stop in expected order
int step = 0;
// Name to be used for stopwatches
constexpr char kStopwatchName[] = "stopwatch";
}  // namespace

class OrderChecker : public alice::Codelet {
 public:
  void start() override {
    ASSERT_EQ(step, 0);
    step = 1;
    tickPeriodically();
  }
  void tick() override {
    ASSERT_EQ(step, 1);
    step = 2;
    reportSuccess();
  }
  void stop() override {
    ASSERT_EQ(step, 2);
    step = 0;
  }
};

// Checks time between stop() calls
class TimeChecker : public alice::Codelet {
 public:
  void stop() override {
    if (stopwatch(kStopwatchName).running()) {
      stopwatch(kStopwatchName).stop();
      EXPECT_GE(stopwatch(kStopwatchName).read(), get_min_time_before_next_stop());
    }
    stopwatch(kStopwatchName).start();
  }

  // Time between two stop() calls need to be at list this many seconds apart
  ISAAC_PARAM(double, min_time_before_next_stop, 0.5);
};

// A test behavior that delays its start by a defined duration, in seconds.
class DelayedStart : public alice::Codelet {
 public:
  void start() override {
    node()->clock()->sleep(get_duration());
  }

  // The amount of time to defer start() by, in seconds.
  ISAAC_PARAM(double, duration);
};

// Run this test with:
//        bazel test packages/behavior_tree/tests:combined --test_output=all
//              --test_arg=--gtest_filter=*StartStopOrder
TEST(CombinedBehaviors, StartStopOrder) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"sequence"});
  repeat->async_set_wait_duration(0.1);

  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "sequence", {"first", "second"}, true);

  alice::Node* first = app.createNode("first");
  first->disable_automatic_start = true;
  first->addComponent<OrderChecker>()->async_set_tick_period("10Hz");

  alice::Node* second = app.createNode("second");
  second->disable_automatic_start = true;
  second->addComponent<OrderChecker>()->async_set_tick_period("10Hz");

  app.enableStopOnTimeout(0.95);
  app.runBlocking();
}

// Run this test with:
//        bazel test packages/behavior_tree/tests:combined --test_output=all
//              --test_arg=--gtest_filter=*RepeatSequenceOfParallels
TEST(CombinedBehaviors, RepeatSequenceOfParallels) {
  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"sequence"});
  repeat->async_set_wait_duration(0.1);

  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "sequence", {"parallel_1", "parallel_2"},
                                                      true);

  for (int idx = 1; idx <= 2; ++idx) {
    // Names of nodes
    const std::string node_name_postfix = std::to_string(idx);
    const std::string node_name_parallel = "parallel_" + node_name_postfix;
    const std::string node_name_timer = "timer_" + node_name_postfix;
    const std::string node_name_constant = "constant_" + node_name_postfix;
    const std::string node_name_checker = "checker_" + node_name_postfix;

    auto* parallel = CreateCompositeBehaviorNode<ParallelBehavior>(
        app, node_name_parallel, {node_name_timer, node_name_constant, node_name_checker}, true);
    parallel->async_set_success_threshold(1);

    alice::Node* timer = app.createNode(node_name_timer);
    timer->disable_automatic_start = true;
    timer->addComponent<TimerBehavior>();

    alice::Node* constant = app.createNode(node_name_constant);
    constant->disable_automatic_start = true;
    constant->addComponent<ConstantBehavior>();

    alice::Node* checker = app.createNode(node_name_checker);
    checker->disable_automatic_start = true;
    checker->addComponent<TimeChecker>();
  }

  app.enableStopOnTimeout(6.0);
  app.runBlocking();
}

// Run this test with:
//      bazel test packages/behavior_tree/tests:combined --runs_per_test=10 --jobs 1
//          --test_arg=--gtest_filter=*RepeatSuccess
TEST(CombinedBehaviors, RepeatSuccess) {
  constexpr double kWaitBeforeRepeat = 1e-11;

  alice::Application app;
  auto* repeat = CreateCompositeBehaviorNode<RepeatBehavior>(app, "repeat", {"constant"});
  repeat->async_set_wait_duration(kWaitBeforeRepeat);

  alice::Node* constant = app.createNode("constant");
  constant->disable_automatic_start = true;
  constant->addComponent<ConstantBehavior>()->async_set_status(alice::Status::SUCCESS);

  app.enableStopOnTimeout(1.0);
  app.runBlocking();
  EXPECT_GE(repeat->getTickCount(), 3);
}

// This test checks for the correct behavior of ParallelBehavior when one of its child nodes takes
// longer to start() than expected. Specifically what is reproduced here is:
// 1. A number of nodes is executed, and they succeed. In this case, three TimerBehavior nodes.
// 2. The same TimerBehavior nodes are run again inside ParallelBehavior, but with a
//    DelayedStart node after the first one.
// 3. Since the success threshold of the ParallelBehavior is 2, and the child nodes are started
//    in sequence, no child should tick the ParallelBehavior before all child nodes are initialized
//    properly. Should this not be the case, the dangling `SUCCESS` status of `timer_2` and
//    `timer_3` will already satisfy `parallel`'s `success_threshold` when `timer_1` transitions
//    from its former `SUCCESS` state to `RUNNING`.
// 4. The test checks if either `timer_2` or `timer_3` ticked twice, respectively. For
//    TimerBehavior, the first tick is ignored, and the second tick (after the timer's delay)
//    reports success. So for ParallelBehavior to report `SUCCESS`, at least one of the two needs
//    to tick twice to report success as `delayed_start` doesn't succeed. If this is not the case,
//    and ParallelBehavior succeeded due to a dangling `SUCCESS` flag in one of the timers.
TEST(CombinedBehaviors, ParallelFailsWhenDelayedChildAreadySucceeded) {
  alice::Application app;
  CreateCompositeBehaviorNode<MemorySequenceBehavior>(app, "sequence",
                                                      {"timer_1",
                                                       "timer_2",
                                                       "timer_3",
                                                       "parallel"});
  auto* parallel = CreateCompositeBehaviorNode<ParallelBehavior>(app, "parallel",
                                                                 {"timer_1",
                                                                  "delayed_start",
                                                                  "timer_2",
                                                                  "timer_3"});
  parallel->async_set_success_threshold(2);
  parallel->node()->disable_automatic_start = true;

  auto* timer_1 = CreateSubBehaviorNode<TimerBehavior>(app, "timer_1");
  timer_1->async_set_delay(0.1);
  timer_1->node()->disable_automatic_start = true;
  auto* timer_2 = CreateSubBehaviorNode<TimerBehavior>(app, "timer_2");
  timer_2->async_set_delay(0.15);
  timer_2->node()->disable_automatic_start = true;
  auto* timer_3 = CreateSubBehaviorNode<TimerBehavior>(app, "timer_3");
  timer_3->async_set_delay(0.2);
  timer_3->node()->disable_automatic_start = true;
  auto* delayed_start = CreateSubBehaviorNode<DelayedStart>(app, "delayed_start");
  delayed_start->async_set_duration(0.5);
  delayed_start->node()->disable_automatic_start = true;

  app.enableStopOnTimeout(2.0);
  app.runBlocking();

  // Either of `timer_2` or `timer_3` needs to have ticked twice because they are the only nodes in
  // `parallel` that can cause it to succeed, together with `timer_1`. If neither of `timer_2` or
  // `timer_3` ticked twice (i.e. reported success as TimerBehavior dictates) and `parallel` still
  // succeeded, the `SUCCESS` state was entered without justified satisfaction of `parallel`'s
  // `success_threshold`. That's why it is enough for one of the two to tick twice.
  EXPECT_TRUE(timer_2->getTickCount() == 2 || timer_3->getTickCount() == 2);
}

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::DelayedStart);
ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::OrderChecker);
ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::TimeChecker);
