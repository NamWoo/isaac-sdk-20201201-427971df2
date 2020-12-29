/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/backend/py_codelet_flow_control.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace alice {

// PyCodelet is a C++ Codelet instance for Python codelets that synchronizes with a Python codelet
// to mimic the effect of embedding Python scripts into the C++ codelet.
class PyCodelet : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // These update the hooks that are required in a Python Codelet. The hook update can be called
  // from Python at any start/tick/stop body.
  void addRxHook(const std::string& rx_hook);
  // Gets the message
  ConstMessageBasePtr receiveMessage(const std::string& tag);

  // Synchronizes two channels by their tags
  void synchronizeWithTags(const std::string& tag1, const std::string& tag2);
  // Changes tick on message behaviour by the channel's tag
  void tickOnMessageWithTag(const std::string& tag);

  std::string pythonWaitForJob();
  void pythonJobFinished();

  // Send messages to sight for visualization
  void show(const std::string& sop_json);

  // Parameter for getting Isaac parameters to pyCodelets. For details, see PybindPyCodelet.
  ISAAC_PARAM(nlohmann::json, config, nlohmann::json({}));

 private:
  // Gets the rx message hook ptr of the pycodelet given the tag. It asserts when the hook is not
  // found when assert is set to true. Otherwise it return std::nullopt when the hook is not found.
  std::optional<RxMessageHook*> findRxMessageHook(const std::string& tag) {
    auto it = customized_rx_message_hooks_.find(tag);
    if (it != customized_rx_message_hooks_.end())
      return dynamic_cast<RxMessageHook*>(it->second.get());
    return std::nullopt;
  }

  RxMessageHook* getRxMessageHook(const std::string& tag) {
    auto hook = findRxMessageHook(tag);
    ASSERT(hook, "RxMessageHook [%s] not found in PyCodelet [%s]", tag.c_str(),
           full_name().c_str());
    return *hook;
  }

  // Finds the tx message hook ptr of the pycodelet given the tag. It returns an optional hook if it
  // is not found.
  std::optional<TxMessageHook*> findTxMessageHook(const std::string& tag) {
    auto it = customized_tx_message_hooks_.find(tag);
    if (it != customized_tx_message_hooks_.end())
      return dynamic_cast<TxMessageHook*>(it->second.get());
    return std::nullopt;
  }

  TxMessageHook* getTxMessageHook(const std::string& tag) {
    auto hook = findTxMessageHook(tag);
    ASSERT(hook, "TxMessageHook [%s] not found in PyCodelet [%s]", tag.c_str(),
           full_name().c_str());
    return *hook;
  }

  // Adds a rx message hook in the pycodelet. If it already exists, nothing happens.
  void addRxMessageHook(const std::string& tag) {
    if (findRxMessageHook(tag)) {
      return;
    }
    customized_rx_message_hooks_.emplace(tag, std::make_unique<RxMessageHook>(this, tag));
    static_cast<Hook*>(customized_rx_message_hooks_[tag].get())->connect();
  }

  // Adds a tx message hook in the pycodelet. If it already exists, nothing happens.
  void addTxMessageHook(const std::string& tag) {
    if (findTxMessageHook(tag)) {
      return;
    }
    customized_tx_message_hooks_.emplace(tag, std::make_unique<TxMessageHook>(this, tag));
    static_cast<Hook*>(customized_tx_message_hooks_[tag].get())->connect();
  }

  // Gets the cap'n'proto message in the rx hook by tag
  const MessageBase* getMessage(const std::string& tag) {
    return getRxMessageHook(tag)->getMessage();
  }

  // Publishes a cap'n'proto message through the tx hook by tag
  void publishMessage(const std::string& tag, MessageBasePtr message,
                      std::optional<int64_t> acqtime) {
    getTxMessageHook(tag)->publishImpl(message, acqtime);
  }

  PyCodeletFlowControl pycodelet_flow_control_;  // the delegator that manages the python thread
  // maps from tag to tx/rx hooks
  std::map<std::string, std::unique_ptr<RxMessageHook>> customized_rx_message_hooks_;
  std::map<std::string, std::unique_ptr<TxMessageHook>> customized_tx_message_hooks_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::PyCodelet);
