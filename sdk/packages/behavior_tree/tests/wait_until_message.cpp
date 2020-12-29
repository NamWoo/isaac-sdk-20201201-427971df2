/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "gtest/gtest.h"
#include "packages/behavior_tree/components/WaitUntilMessage.hpp"

namespace isaac {
namespace behavior_tree {

// Data structure for the message
struct FooClass {
  int foo_value;
};

// Codelet to publish message
class MyPublisher : public alice::Codelet {
 public:
  void publish(bool first_channel) {
    FooClass foo{42};
    if (first_channel) {
      tx_foo_message1().publish(foo);
    } else {
      tx_foo_message2().publish(foo);
    }
  }

  ISAAC_RAW_TX(FooClass, foo_message1)
  ISAAC_RAW_TX(FooClass, foo_message2)
};

TEST(MessageBehavior, WaitUntilMessage) {
  alice::Application app;

  alice::Node* node_behavior = app.createNode("behavior");
  node_behavior->addComponent<alice::MessageLedger>();
  auto* behavior = node_behavior->addComponent<WaitUntilMessage>();
  behavior->async_set_channel_name("channel_name");

  alice::Node* node_publisher = app.createNode("publisher");
  node_publisher->addComponent<alice::MessageLedger>();
  auto* publisher = node_publisher->addComponent<MyPublisher>();

  Connect(publisher->tx_foo_message1(), behavior, "wrong_name");
  Connect(publisher->tx_foo_message2(), behavior, "channel_name");

  app.runAsync();
  EXPECT_EQ(behavior->getStatus(), alice::Status::RUNNING);
  Sleep(SecondsToNano(0.10));
  // This publish does not result in success because foo_message1 is connected to the wrong channel
  // name
  publisher->publish(true);
  Sleep(SecondsToNano(0.10));
  EXPECT_EQ(behavior->getStatus(), alice::Status::RUNNING);
  Sleep(SecondsToNano(0.10));
  publisher->publish(false);
  Sleep(SecondsToNano(0.10));
  EXPECT_EQ(behavior->getStatus(), alice::Status::SUCCESS);
  app.stopBlocking();
}

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::MyPublisher);
