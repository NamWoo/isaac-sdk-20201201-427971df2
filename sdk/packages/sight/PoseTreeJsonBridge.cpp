/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sight/PoseTreeJsonBridge.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace sight {

namespace {

// Writes information about a pose tree edge to json
void WriteEdgeToJson(size_t a, size_t b, const Pose3d& a_T_b, double timestamp,
                     nlohmann::json& edge_json) {
  edge_json.push_back(a);
  edge_json.push_back(b);
  edge_json.push_back(timestamp);
  nlohmann::json pose_json;
  serialization::Set(pose_json, a_T_b);
  edge_json.emplace_back(std::move(pose_json));
}

// A helper type to create a lookup table of names
struct StringLookupTable {
 public:
  // If necessary adds the given value to our lookup table, and then returns the index in the table
  size_t getOrInsert(const std::string& value) {
    // If we already have the value in our table we can just return the index
    auto it = lookup_.find(value);
    if (it != lookup_.end()) {
      return it->second;
    }
    // Otherwise add the value and return the new index
    const size_t value_index = values_.size();
    lookup_[value] = value_index;
    values_.push_back(value);
    return value_index;
  }

  // The list of values in the table
  const std::vector<std::string>& values() const { return values_; }

 private:
  std::map<std::string, size_t> lookup_;
  std::vector<std::string> values_;
};

}  // namespace

void PoseTreeJsonBridge::start() {
  tickPeriodically();
}

void PoseTreeJsonBridge::tick() {
  // Get a consistent snapshot of the pose tree
  // TODO We should add a special function to only copy the last n seconds to avoid copying large
  //      timeseries of historic data which we won't actually need.
  const pose_tree::PoseTree pose_tree =
      node()->app()->backend()->pose_tree()->cloneLatestPoseTree();

  // Create a lookup map from node UUID to index
  StringLookupTable node_names;

  Json edges_json;

  // Iterate overa all edges in the pose tree
  for (const auto& kvp : pose_tree.edges()) {
    const auto& range = kvp.second.back();
    // Check the edge is still at the current time.
    if (range.end <= getTickTime()) continue;
    // Make sure that we have the corresponding nodes in our lookup table
    const size_t index_a = node_names.getOrInsert(kvp.first.first);
    const size_t index_b = node_names.getOrInsert(kvp.first.second);
    // Get the latest pose
    // TODO Do some more sophisticated interpolation or extrapolation
    const auto& edge = range.history.youngest();
    // Add edge to json
    Json edge_json;
    WriteEdgeToJson(index_a, index_b, edge.state, edge.stamp, edge_json);
    edges_json.push_back(std::move(edge_json));
  }

  // Create and publish the JSON for the whole pose tree
  Json pose_tree_json;
  pose_tree_json["time"] = getTickTime();
  pose_tree_json["nodes"] = node_names.values();
  pose_tree_json["edges"] = std::move(edges_json);
  tx_pose_tree().publish(std::move(pose_tree_json));
}

}  // namespace sight
}  // namespace isaac
