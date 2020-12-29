/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "gtest/gtest.h"
#include "packages/behavior_tree/components/SwitchBehavior.hpp"
#include "packages/behavior_tree/tests/utils.hpp"

namespace isaac {
namespace behavior_tree {

class ExecutionMonitor : public alice::Codelet {
 public:
  void start() override {
    was_started_ = true;
    tickPeriodically();
  }

  bool getWasStarted() { return was_started_; }

 private:
  bool was_started_;
};

TEST(SwitchBehavior, ResolveNodeNameCorrectly) {
  alice::Application app;
  CreateCompositeBehaviorNode<SwitchBehavior>(app, "parent", {"normal_node", "aliased_node"});
  auto normal_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "normal_node");
  normal_node->async_set_tick_period("10Hz");
  auto aliased_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "aliased_node");
  aliased_node->async_set_tick_period("10Hz");

  static const auto config = R"(
    {
      "parent": {
        "isaac.behavior_tree.SwitchBehavior": {
          "node_alias_map": {
            "alias": "aliased_node"
          },
          "desired_behavior": "normal_node"
        }
      }
    }
  )"_json;

  app.backend()->config_backend()->set(config);

  app.enableStopOnTimeout(0.5);
  app.runBlocking();

  EXPECT_TRUE(normal_node->getWasStarted());
  EXPECT_FALSE(aliased_node->getWasStarted());
}

TEST(SwitchBehavior, ResolveAliasCorrectly) {
  alice::Application app;
  CreateCompositeBehaviorNode<SwitchBehavior>(app, "parent", {"normal_node", "aliased_node"});
  auto normal_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "normal_node");
  normal_node->async_set_tick_period("10Hz");
  auto aliased_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "aliased_node");
  aliased_node->async_set_tick_period("10Hz");

  static const auto config = R"(
    {
      "parent": {
        "isaac.behavior_tree.SwitchBehavior": {
          "node_alias_map": {
            "alias": "aliased_node"
          },
          "desired_behavior": "alias"
        }
      }
    }
  )"_json;

  app.backend()->config_backend()->set(config);

  app.enableStopOnTimeout(0.5);
  app.runBlocking();

  EXPECT_FALSE(normal_node->getWasStarted());
  EXPECT_TRUE(aliased_node->getWasStarted());
}

TEST(SwitchBehavior, AliasesTakePrecedenceOverNodeNames) {
  alice::Application app;
  CreateCompositeBehaviorNode<SwitchBehavior>(app, "parent", {"node", "aliased_node"});
  auto normal_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "node");
  normal_node->async_set_tick_period("10Hz");
  auto aliased_node = CreateSubBehaviorNode<ExecutionMonitor>(app, "aliased_node");
  aliased_node->async_set_tick_period("10Hz");

  static const auto config = R"(
    {
      "parent": {
        "isaac.behavior_tree.SwitchBehavior": {
          "node_alias_map": {
            "node": "aliased_node"
          },
          "desired_behavior": "node"
        }
      }
    }
  )"_json;

  app.backend()->config_backend()->set(config);

  app.enableStopOnTimeout(0.5);
  app.runBlocking();

  EXPECT_FALSE(normal_node->getWasStarted());
  EXPECT_TRUE(aliased_node->getWasStarted());
}

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::ExecutionMonitor);
