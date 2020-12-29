/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pose_tree.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/gems/interpolation/poses.hpp"

namespace isaac {
namespace pose_tree {

namespace {

// The maximum of history ranges we keep per edges.
constexpr const size_t kHistoryMaxRanges = 50;
// For a given range, what is the maximum poses we keep.
constexpr const size_t kHistoryMaxSize = 1000;
// How much history stamp do we keep track of.
constexpr const size_t kTopologyChangesMaxSize = 10000;

// Interpolates a pose in a time series at given time.
std::optional<Pose3d> Interpolate(const PoseTree::PoseHistory& history, double stamp) {
  return history.interpolate_2_p(stamp,
      [](double p, const Pose3d& a, const Pose3d& b) {
        return ::isaac::Interpolate(p, a, b);
      });
}
Pose3d Interpolate(const PoseTree::PoseHistory& history, double stamp, size_t index) {
  return history.interpolate_2_p(stamp, index,
      [](double p, const Pose3d& a, const Pose3d& b) {
        return ::isaac::Interpolate(p, a, b);
      });
}

// Returns a point to the only RangeHistory that contains stamp. If none of them includes stamp
// then nullptr is returned.
// Note that the range is semi open: [RangeHistory.start, RangeHistory.end)
const PoseTree::PoseHistory* GetRangeHistory(const std::vector<PoseTree::RangeHistory>& ranges,
                                             double stamp) {
  auto history = std::lower_bound(ranges.begin(), ranges.end(), stamp,
                                  [](const PoseTree::RangeHistory& a, double stamp) {
                                    return a.end <= stamp;
                                  });
  if (history == ranges.end() || history->start > stamp) {
    return nullptr;
  }
  return &(history->history);
}

}  // namespace

bool PoseTree::hasDirectConnection(const std::string& lhs, const std::string& rhs,
                                   double stamp) const {
  // Same node are always connected
  if (lhs == rhs) return true;
  auto ranges = edges_.find({lhs, rhs});
  // If there was never an edge, then it is obviously not connected
  if (ranges == edges_.end()) return false;
  return GetRangeHistory(ranges->second, stamp) != nullptr;
}

bool PoseTree::hasIndirectConnection(const std::string& lhs, const std::string& rhs,
                                     double stamp) const {
  return !hasDirectConnection(lhs, rhs, stamp) && !findPath(lhs, rhs, stamp).empty();
}

std::optional<std::pair<Pose3d, double>> PoseTree::getLatest(const std::string& lhs,
                                                             const std::string& rhs) const {
  // Check for identity
  if (lhs == rhs) {
    return std::make_pair(Pose3d::Identity(), 0.0);
  }
  // If we have never added any edge, there is no possible path.
  if (topology_change_stamps_.empty()) return std::nullopt;

  // try to find a path at the last known topology change.
  const std::vector<std::string> path = findPath(lhs, rhs, *topology_change_stamps_.rbegin());
  if (path.empty()) return std::nullopt;

  double latest_time = 0.0;
  Pose3d pose = Pose3d::Identity();
  for (size_t frame_id = 1; frame_id < path.size(); frame_id++) {
    auto it = edges_.find({path[frame_id-1], path[frame_id]});
    ASSERT(it != edges_.end() && !it->second.empty(), "Logic error, missing edge: %s_T_%s",
           path[frame_id-1].c_str(), path[frame_id].c_str());
    const auto& youngest = it->second.back().history.youngest();
    pose = pose * youngest.state;
    latest_time = std::max(latest_time, youngest.stamp);
  }
  return std::make_pair(pose, latest_time);
}

std::optional<Pose3d> PoseTree::get(const std::string& lhs, const std::string& rhs,
                                    double stamp) const {
  // Check for identity
  if (lhs == rhs) {
    return Pose3d::Identity();
  }
  // See if this is a direct edge
  auto ranges = edges_.find({lhs, rhs});
  if (ranges != edges_.end()) {
    const auto* history = GetRangeHistory(ranges->second, stamp);
    if (history != nullptr) {
      return Interpolate(*history, stamp);
    }
  }
  // try to find a path
  const std::vector<std::string> path = findPath(lhs, rhs, stamp);
  if (path.empty()) return std::nullopt;
  return interpolateOnPath(path, stamp);
}

bool PoseTree::set(const std::string& lhs, const std::string& rhs, double stamp,
                   const Pose3d& lhs_T_rhs) {
  if (lhs == rhs) {
    return false;
  }

  auto it_rhs_lhs = edges_.find({rhs, lhs});
  const bool need_new_edge = it_rhs_lhs == edges_.end() || it_rhs_lhs->second.empty() ||
                             it_rhs_lhs->second.back().end <= stamp;
  if (need_new_edge) {
    // Check for all the timestamp that contains a structure change in the tree.
    // TODO(ben): Implement a simpler function hasIndirectConnection that do not check for the
    // timestamp. If using all the edges that has ever existed there is no connection then we can
    // stop here.
    for (auto it = topology_change_stamps_.rbegin(); it != topology_change_stamps_.rend(); ++it) {
      if (hasIndirectConnection(lhs, rhs, *it)) {
        return false;
      }
      if (*it <= stamp) break;
    }
  }
  auto& r_lhs_rhs = edges_[{lhs, rhs}];
  auto& r_rhs_lhs = edges_[{rhs, lhs}];
  // If we are creating a new range
  if (need_new_edge) {
    topology_change_stamps_.insert(stamp);
    // Remove the first stamp if we have reached the size limit
    if (topology_change_stamps_.size() > kTopologyChangesMaxSize) {
      topology_change_stamps_.erase(topology_change_stamps_.begin());
    }
    r_lhs_rhs.push_back({stamp, std::numeric_limits<double>::max(), PoseHistory()});
    r_rhs_lhs.push_back({stamp, std::numeric_limits<double>::max(), PoseHistory()});
    r_lhs_rhs.back().history.insert(stamp, lhs_T_rhs);
    r_rhs_lhs.back().history.insert(stamp, lhs_T_rhs.inverse());
    if (r_lhs_rhs.size() > kHistoryMaxRanges) r_lhs_rhs.erase(r_lhs_rhs.end() - kHistoryMaxRanges);
    if (r_rhs_lhs.size() > kHistoryMaxRanges) r_rhs_lhs.erase(r_rhs_lhs.end() - kHistoryMaxRanges);
    outgoing_[lhs].insert(rhs);
    outgoing_[rhs].insert(lhs);
    return true;
  }
  auto& h_lhs_rhs = r_lhs_rhs.back().history;
  auto& h_rhs_lhs = r_rhs_lhs.back().history;
  if (!h_lhs_rhs.tryPush(stamp, lhs_T_rhs)) return false;
  // Push here as if the previous tryPush succeeded, this push needs to be succeed or we would leave
  // into an invalid state.
  h_rhs_lhs.push(stamp, lhs_T_rhs.inverse());
  h_lhs_rhs.forgetBySize(kHistoryMaxSize);
  h_rhs_lhs.forgetBySize(kHistoryMaxSize);
  return true;
}

PoseTree PoseTree::latest() const {
  PoseTree result;
  result.outgoing_ = outgoing_;
  for (const auto& kvp : edges_) {
    if (kvp.second.empty() || kvp.second.back().history.empty()) continue;
    const auto& latest = kvp.second.back().history.youngest();
    result.edges_[kvp.first].push_back(
        {kvp.second.back().start, kvp.second.back().end, PoseHistory()});
    result.edges_[kvp.first].back().history.push(latest.stamp, latest.state);
    result.topology_change_stamps_.insert(latest.stamp);
  }
  return result;
}

bool PoseTree::removeFrame(const std::string& frame, double stamp) {
  auto it = outgoing_.find(frame);
  if (it == outgoing_.end()) return false;
  // Check first if we can disconnect all the edges.
  for (const std::string& out : it->second) {
    auto& frame_out = edges_[{frame, out}].back();
    if (frame_out.history.youngest().stamp >= stamp) {
      return false;
    }
  }
  // Go throught all the edges and disconnect them.
  for (const std::string& out : it->second) {
    auto& frame_out = edges_[{frame, out}].back();
    // Ignore edge already disconnected.
    if (frame_out.end <= stamp) continue;
    frame_out.end = stamp;
    auto& out_frame = edges_[{out, frame}].back();
    out_frame.end = stamp;
  }
  topology_change_stamps_.insert(stamp);
  // Remove the first stamp if we have reached the size limit
  if (topology_change_stamps_.size() > kTopologyChangesMaxSize) {
    topology_change_stamps_.erase(topology_change_stamps_.begin());
  }
  return true;
}

bool PoseTree::removeEdge(const std::string& lhs, const std::string& rhs, double stamp) {
  auto lhs_rhs = edges_.find({lhs, rhs});
  // If there is no connection between lhs and rhs, then
  if (lhs_rhs == edges_.end() || lhs_rhs->second.empty()) return false;
  auto& lhs_rhs_range = lhs_rhs->second.back();

  if (lhs_rhs_range.history.youngest().stamp >= stamp || stamp >= lhs_rhs_range.end) {
    return false;
  }
  lhs_rhs_range.end = stamp;

  auto rhs_lhs = edges_.find({rhs, lhs});
  // Sanity check, these edges should be symetrical.
  ASSERT(rhs_lhs != edges_.end(), "%s_T_%s does not exist", rhs.c_str(), lhs.c_str());
  ASSERT(!rhs_lhs->second.empty(), "%s_T_%s is empty", rhs.c_str(), lhs.c_str());
  auto& rhs_lhs_range = rhs_lhs->second.back();
  ASSERT(rhs_lhs_range.history.youngest().stamp < stamp && stamp < rhs_lhs_range.end,
         "invalid range: %lf not in ]%lf, %lf]",
         stamp, rhs_lhs_range.history.youngest().stamp, rhs_lhs_range.end);
  rhs_lhs_range.end = stamp;
  topology_change_stamps_.insert(stamp);
  // Remove the first stamp if we have reached the size limit
  if (topology_change_stamps_.size() > kTopologyChangesMaxSize) {
    topology_change_stamps_.erase(topology_change_stamps_.begin());
  }
  return true;
}

std::vector<std::string> PoseTree::findPath(const std::string& lhs, const std::string& rhs,
                                            double stamp) const {
  if (outgoing_.count(lhs) == 0 || outgoing_.count(rhs) == 0) {
    return {};
  }
  std::queue<std::string> open;
  open.push(rhs);
  std::map<std::string, std::string> visited;
  visited[rhs] = "";
  while (!open.empty()) {
    std::string top = std::move(open.front());
    open.pop();
    auto it = outgoing_.find(top);
    ASSERT(it != outgoing_.end(), "Node without outgoing edges");
    for (const std::string& out : it->second) {
      // Check if already visited
      if (visited.count(out) > 0) {
        continue;
      }
      // New possible edge
      auto ranges = edges_.find({top, out});
      ASSERT(ranges != edges_.end(), "Edge without history");
      const auto* history = GetRangeHistory(ranges->second, stamp);
      if (history == nullptr) continue;
      // Check if target found
      if (out == lhs) {
        // trace back path
        std::vector<std::string> path = {out, top};
        while (top != rhs) {
          auto kt = visited.find(top);
          ASSERT(kt != visited.end(), "How did we come here?");
          top = kt->second;
          path.push_back(top);
        }
        return path;
      }
      // Add possible branch
      visited[out] = top;
      open.push(std::move(out));
    }
  }
  return {};
}

Pose3d PoseTree::interpolateOnPath(const std::vector<std::string>& path, double stamp) const {
  Pose3d pose = Pose3d::Identity();
  for (size_t frame_id = 1; frame_id < path.size(); frame_id++) {
    auto it = edges_.find({path[frame_id-1], path[frame_id]});
    ASSERT(it != edges_.end(), "Logic error, missing edge: %s_T_%s",
           path[frame_id-1].c_str(), path[frame_id].c_str());
    const auto* history = GetRangeHistory(it->second, stamp);
    ASSERT(history != nullptr, "Logic error, missing edge: %s_T_%s",
           path[frame_id-1].c_str(), path[frame_id].c_str());
    const ssize_t history_lower_index = history->interpolate_2_index(stamp);
    pose = pose * Interpolate(*history, stamp, history_lower_index);
  }
  return pose;
}

}  // namespace pose_tree
}  // namespace isaac
