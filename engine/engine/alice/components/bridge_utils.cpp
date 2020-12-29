/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "bridge_utils.hpp"

#include <string>
#include <utility>

namespace isaac {
namespace alice {

void SendMsgToWebsightServer(RawTx<nlohmann::json>& tx, std::string cmd, Json cmd_data) {
  Json reply;
  reply["cmd"] = std::move(cmd);
  reply["cmd-data"] = std::move(cmd_data);
  tx.publish(std::move(reply));
}

}  // namespace alice
}  // namespace isaac

