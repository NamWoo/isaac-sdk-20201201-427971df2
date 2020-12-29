/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/backend/component_backend.hpp"
#include "engine/alice/components/MessageLedger.hpp"

namespace isaac {
namespace scheduler {
class Scheduler;
}
}  // namespace isaac

namespace isaac {
namespace alice {

// Backend for the MessageLedger component
class MessageLedgerBackend : public ComponentBackend<MessageLedger> {
 public:
  // A message connection between two components
  struct Connection {
    // The source endpoint in the format "nodename/compname/channelname"
    std::string source;
    // The target endpoint (same format as source)
    std::string target;
  };

  // Connects two endpoints together
  void connect(const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx);

  // Gets a list of all connections
  const std::vector<Connection>& connections() const { return connections_; }

 private:
  std::mutex connections_mutex_;
  std::vector<Connection> connections_;
};

}  // namespace alice
}  // namespace isaac
