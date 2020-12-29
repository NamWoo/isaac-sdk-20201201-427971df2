/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "InteractiveMarkersBridge.hpp"

#include <errno.h>
#include <unistd.h>

#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/components/Pose.hpp"
#include "engine/alice/components/PoseInitializer.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

namespace {
// Commands received from the Front end
constexpr char kCmdSynch[] = "edit";
constexpr char kCmdQuery[] = "query";
// Commands send to the front end
constexpr char kCmdPopulate[] = "populate";
}  // namespace

// Auxiliary function to encode a single edge as a JSON
void WriteEdgeToJson(const std::string& a, const std::string& b, const Pose3d& a_T_b,
                     nlohmann::json& edge_json) {
  edge_json.push_back(a);
  edge_json.push_back(b);
  nlohmann::json pose_json;
  serialization::Set(pose_json, a_T_b);
  edge_json.emplace_back(std::move(pose_json));
}

void InteractiveMarkersBridge::start() {
  tickPeriodically();
  // Populate the list of editable edges
  registerEdges();
}

void InteractiveMarkersBridge::tick() {
  // Process all messages from sight since last tick function call
  // TODO It is unclear why `this->` is required explicitly by the compiler in the lambda function
  rx_request().processAllNewMessages([this](auto json, int64_t pubtime, int64_t acqtime) {
    if (this->isMessageValid(json, kCmdSynch, "edge")) {
      const auto lhs = serialization::TryGetFromMap<std::string>(json, "lhs");
      const auto rhs = serialization::TryGetFromMap<std::string>(json, "rhs");
      if (!lhs || !rhs) {
        LOG_ERROR("Message does not contain an edge: %s", json.dump(2).c_str());
        return;
      }
      if (this->isEdgeEditable(*lhs, *rhs)) {
        const auto translation =
            serialization::TryGetFromMap<std::vector<double>>(json, "position");
        const auto rotation = serialization::TryGetFromMap<std::vector<double>>(json, "rotation");
        if (!translation || !rotation) {
          LOG_ERROR("Message does not contain a valid pose: %s", json.dump(2).c_str());
          return;
        }
        // Get new value for the edge and edit it
        const Pose3d lhs_T_rhs{
            SO3d::FromQuaternion({(*rotation)[0], (*rotation)[1], (*rotation)[2], (*rotation)[3]}),
            Vector3d((*translation)[0], (*translation)[1], (*translation)[2])
        };
        // See if we have a timestamp and use it
        const auto maybe_timestamp = serialization::TryGetFromMap<double>(json, "timestamp");
        if (maybe_timestamp) {
          this->node()->pose().set(*lhs, *rhs, lhs_T_rhs, *maybe_timestamp);
        } else {
          this->node()->pose().set(*lhs, *rhs, lhs_T_rhs, this->getTickTime());
        }
      } else {
        LOG_ERROR("Tried to edit a non-editable edge: [%s, %s]", (*lhs).c_str(), (*rhs).c_str());
      }
    } else if (this->isMessageValid(json, kCmdQuery, "edges")) {
      this->sendEditableList();
    }
  });
}

bool InteractiveMarkersBridge::isMessageValid(const Json& json, const std::string& expected_cmd,
    const std::string& expected_param) {
  const auto maybe_cmd = serialization::TryGetFromMap<std::string>(json, "cmd");
  if (!maybe_cmd) {
    LOG_ERROR("Received message does not contain 'cmd': %s", json.dump(2).c_str());
    return false;
  }
  if (*maybe_cmd != expected_cmd) {
    // Silent return, message is OK its just not for us
    return false;
  }
  const auto param = serialization::TryGetFromMap<std::string>(json, "cmdparams");
  if (!param) {
    LOG_ERROR("Received cmd: %s does not contain cmdparams", maybe_cmd->c_str());
    return false;
  }
  if (*param != expected_param) {
    // Silent return, message's param is OK it's just not what we expected
    return false;
  }
  return true;
}

void InteractiveMarkersBridge::registerEdges() {
  for (auto* marker : node()->app()->findComponents<PoseInitializer>()) {
    if (marker->get_attach_interactive_marker()) {
      edges_.insert(std::make_pair(marker->get_lhs_frame(), marker->get_rhs_frame()));
    }
  }
}

void InteractiveMarkersBridge::sendEditableList() {
  Json msg;
  msg["cmd"] = kCmdPopulate;
  Json edges_json = Json::array();
  for (const auto& edge : edges_) {
    Json json_edge;
    json_edge.push_back(edge.first);
    json_edge.push_back(edge.second);
    edges_json.push_back(json_edge);
  }
  msg["edges"] = std::move(edges_json);
  msg["startTime"] = getTickTime();
  tx_reply().publish(std::move(msg));
}

bool InteractiveMarkersBridge::isEdgeEditable(const std::string& lhs,
    const std::string& rhs) const {
  return (edges_.count({lhs, rhs}) > 0) || (edges_.count({rhs, lhs}) > 0);
}

}  // namespace alice
}  // namespace isaac
