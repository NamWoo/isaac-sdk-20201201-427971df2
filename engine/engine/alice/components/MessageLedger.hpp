/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>  // NOLINT
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/component_registry.hpp"
#include "engine/alice/component_impl.hpp"
#include "engine/alice/hooks/config_hook.hpp"
#include "engine/alice/message.hpp"
#include "engine/gems/math/exponential_moving_average.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "gxf/gems/staging_queue/staging_queue.hpp"

namespace isaac {
namespace alice {

class MessageLedgerBackend;

// A queue with alice messages
using ConstMessageQueue = gxf::staging_queue::StagingQueue<ConstMessageBasePtr>;

// Stores time histories of messages for various channels of this node and distributes messages
// between various systems. Every node which engages in message passing must have a MessageLedger
// component.
class MessageLedger : public Component {
 public:
  void initialize() override;
  void deinitialize() override;

  // An endpoint (tx or rx) of a connection
  struct Endpoint {
    // Component which transmits or receives the message
    const Component* component;
    // A name under which the message is transmitted or received
    std::string tag;

    // Computes a name based on the node, component, and tag
    std::string name() const;

    // Computes a name based on the app, node, component, and tag
    std::string nameWithApp() const;

    // Comparison operator to enable this type as a key in std::map
    friend bool operator<(const Endpoint& lhs, const Endpoint& rhs) {
      return lhs.component < rhs.component || (lhs.component == rhs.component && lhs.tag < rhs.tag);
    }
  };

  // Statistics about a channel
  struct ChannelStatistics {
    // The total number of messages received on this channel
    size_t count;
    // A running average of the number of messages received per second
    math::ExponentialMovingAverageRate<double> rate;
  };

  // Type of callback to notify about new connections
  using OnConnectCallback = std::function<void(const Endpoint& tx, const Endpoint& rx)>;
  // Type of callback to notify about new messages
  using OnMessageCallback = std::function<void(ConstMessageBasePtr)>;
  // Type of callback to notify about new messages on a channel
  using OnChannelMessageCallback =
      std::function<void(const Endpoint&, const ConstMessageBasePtr& message)>;
  // Callback type for a list of messages
  using OnMessageListCallback = std::function<void(const ConstMessageQueue&)>;

  // Adds a callback to be called when this message ledger is connected as a transmitter
  void addOnConnectAsTxCallback(OnConnectCallback callback);
  // Adds a callback to be called when this message ledger is connected as a receiver
  void addOnConnectAsRxCallback(OnConnectCallback callback);

  // Adds a new message to the given endpoint
  void provide(const Endpoint& endpoint, ConstMessageBasePtr message);

  // Notifies the scheduler about messages received on an endpoint for example to tick codelets
  void notifyScheduler(const Endpoint& endpoint, int64_t target_time) const;

  // Adds a callback to be called when a new message is added into the `tx` endpoint which shall
  // be delivered to the `rx` endpoint in a different message ledger.
  void addOnMessageCallback(const Endpoint& endpoint, const Component* source,
                            OnMessageCallback callback);

  // Adds a callback which is called for every new message added to the message ledger.
  void addOnMessageCallback(OnChannelMessageCallback callback);

  // Disconnects all message callbacks connections for a component
  void removeCustomer(const Component* source);
  // Disconnects this ledger so that it does not receive messages anymore
  void disconnect();
  // Adds a new source.
  void addSource(MessageLedger* source, const Component* customer);

  // Iterates overall endpoints and gives access to their message queues.
  void iterateAllQueues(
      std::function<void(const Endpoint&, ConstMessageQueue&)> callback) const;

  // Creates a channel and gets its queue
  ConstMessageQueue& getOrCreateQueue(const Endpoint& endpoint);

  // Accesses the underlying message queue
  ConstMessageQueue* tryGetQueue(const Endpoint& endpoint);

  // return the number of channels connected to this component
  size_t numSourceChannels() const;

  // Returns statistics about all channels
  std::map<std::string, ChannelStatistics> getStatistics() const;

  // The maximum number of messages to hold in the history
  ISAAC_PARAM(int, history, 10);

 private:
  friend class MessageLedgerBackend;

  // Information about a channel, i.e. data coming from a specific source under a given tag
  class Channel {
   public:
    Channel(int capacity);
    // Adds a callback to be called whenever a new message is added to the channel
    void addCallback(const Component* source, OnMessageCallback callback);
    // Removes all callback for a given source
    void removeSource(const Component* source);
    // Adds a message but keeps history length at desired size `max`
    void addMessage(ConstMessageBasePtr message);
    // Gets statistics for this channel
    ChannelStatistics getStatistics() const;

    const ConstMessageQueue& messages() const { return messages_; }
    ConstMessageQueue& messages() { return messages_; }

   private:
    mutable std::mutex messages_mutex_;
    mutable std::mutex channels_mutex_;

    // The message queue for the channel
    ConstMessageQueue messages_;
    // List of all callbacks to call when a message arrives on the channel
    std::map<const Component*, std::vector<OnMessageCallback>> callbacks_;
    // Statistics for this channel
    ChannelStatistics statistics_;
  };

  // Gets the channel for the given endpoints or returns nullptr if it does not exist
  std::shared_ptr<Channel> findChannel(const Endpoint& endpoint) const;
  // Gets the channel for the given endpoints potentially creating it in case it does not exist yet
  std::shared_ptr<Channel> getOrCreateChannel(const Endpoint& endpoint);

  // Called when a connection between two message ledgers is created
  void connectAsTx(const Endpoint& tx, const Endpoint& rx);
  void connectAsRx(const Endpoint& tx, const Endpoint& rx);

  std::vector<OnConnectCallback> on_connect_as_tx_callbacks_;
  std::vector<OnConnectCallback> on_connect_as_rx_callbacks_;

  std::mutex callbacks_mutex_;
  std::vector<OnChannelMessageCallback> callbacks_;

  mutable std::shared_timed_mutex channels_mutex_;
  std::map<Endpoint, std::shared_ptr<Channel>> channels_;

  std::mutex sources_mutex_;
  std::set<std::pair<MessageLedger*, const Component*>> sources_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::MessageLedger)
