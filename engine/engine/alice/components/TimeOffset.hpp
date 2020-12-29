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

#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

class MessageLedger;

// Adds a time offset to a message stream. The current implementation will create copies of
// incoming messages.
class TimeOffset : public Component {
 public:
  void initialize() override;

  // The name of message channel which will have it's time stamps changed.
  ISAAC_PARAM(std::string, input_channel, "input");
  // The name of message channel with changed timestamps.
  ISAAC_PARAM(std::string, output_channel, "output");
  // A time offset in nanoseconds which will be added to the acquisition time of incoming messages.
  ISAAC_PARAM(int64_t, acqtime_offset, 0);

 private:
  MessageLedger* ledger_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::TimeOffset)
