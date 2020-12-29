/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/components/Codelet.hpp"

namespace isaac {
namespace alice {

// Periodically writes statistic about all message passing channels to the redis metadata server.
// The message passing report will be stored in a hashset under the following keys:
//   isaac:AUUID:msgs:SRC_i:DST_i
//     count: total number of messages passed on the channel
//     rate: running average of number of messages per second
// where
//   AUUID: UUID of the current ISAAC application
//   (SRC_i, DST_i): names of the source and destination channels each using the format
//                   node_name/component_name/tag
class MessagePassingReport : public Codelet {
 public:
  void start() override;
  void tick() override;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::MessagePassingReport)
