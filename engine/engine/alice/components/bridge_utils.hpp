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
#include <utility>

#include "engine/alice/hooks/message_hook.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

// sends a json to WebsightServer for front-end
void SendMsgToWebsightServer(RawTx<nlohmann::json>& tx, std::string cmd, Json cmd_data = Json());

}  // namespace alice
}  // namespace isaac
