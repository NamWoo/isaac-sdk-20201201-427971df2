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
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/algorithm/timeseries.hpp"

namespace isaac {
namespace pose_tree {

// A temporal pose tree to store relative coordinate system transformations over time
// This implementation does not support multiple paths between the same coordinate systems at a
// given time. It does however allow to disconnect edge and create new connection using a different
// path. It also allow for multiple "roots". In fact the transformation relationships form an
// acylic, bi-directional, not necessarily fully-connected graph.
class PoseTree {
 public:
  using PoseHistory = Timeseries<Pose3d, double>;
  // Helper structure that contains a PoseHistory and the validity range:
  // All the timestamp in PoseHistory are included in the range [start, end).
  // The range is invalid for any start < stamp or stamp >= end.
  struct RangeHistory {
    double start, end;
    PoseHistory history;
  };
  // Contains all the ranges for a given edge. It assumes range do not overlap and are sorted:
  // EdgeHistory[i].end <= EdgeHistory[i+1].start
  using EdgeHistory = std::vector<RangeHistory>;

  // Checks if there is a direct connection between two coordinate systems at a given time
  bool hasDirectConnection(const std::string& lhs, const std::string& rhs, double stamp) const;
  // Checks if there is an indirect connection between two coordinate systems at a given time.
  // This means a lhs and rhs are not connected directly, but there is a sequence of direct
  // connections to get from lhs to rhs.
  bool hasIndirectConnection(const std::string& lhs, const std::string& rhs, double stamp) const;

  // Gets the 3D pose lhs_T_rhs at given time
  // This will try to find a path between the two specified coordinate frames. If there is no path
  // between the two coordinate frames this function will return std::nullopt.
  // TODO The current implementation will be slow for big graphs
  std::optional<Pose3d> get(const std::string& lhs, const std::string& rhs, double stamp) const;

  // Gets the latest 3D pose lhs_T_rhs and its time
  // This will follow the same logic as the function above, if there is no path at the current time,
  // it will return std::nullopt, if there is no direct connection, it will use the latest pose on
  // each of the traversed edge, and will return the time of the latest update of all these edges.
  std::optional<std::pair<Pose3d, double>> getLatest(const std::string& lhs,
                                                     const std::string& rhs) const;

  // Sets the 3D pose lhs_T_rhs (and rhs_T_lhs) at given time
  // If the connection would add a cycle to the graph this function will return false.
  bool set(const std::string& lhs, const std::string& rhs, double stamp, const Pose3d& lhs_T_rhs);

  // Gets the transformation a_t1_T_a_t2 using `base` as a reference. This is computed as
  // a_t1_T_base_t1 * base_t2_T_a_t2. Here x_t indicates the pose of a frame x at time t.
  // Will return std::nullopt if `a` and `base` are not connected at either time t1 or time t2.
  std::optional<Pose3d> get(const std::string& a, double t1, double t2,
                            const std::string& base) const {
    auto a1_T_base = get(a, base, t1);
    auto base_T_a2 = get(base, a, t2);
    if (!a1_T_base || !base_T_a2) {
      return std::nullopt;
    }
    return (*a1_T_base)*(*base_T_a2);
  }

  // Disconnect a frame from the PoseTree: all the edges connected to it will be removed after
  // the given `stamp`. It can only be disconnected if none of the edge contains an update later
  // than the given `stamp`.
  // Returns true if the frame was disconnected successfully.
  bool removeFrame(const std::string& frame, double stamp);

  // Removes an edge from the PoseTree. The edge will be invalid for any time >= `stamp`.
  // An edge can not be removed if the pose has been set for a time >= `stamp`.
  // Returns true if the edge was disconnected successfully.
  bool removeEdge(const std::string& lhs, const std::string& rhs, double stamp);

  // The number of edges in the pose graph that has ever existed.
  size_t numEdges() const {
    return edges_.size();
  }

  // The total number of poses stored in the pose graph
  size_t numEntries() const {
    size_t count = 0;
    for (const auto& edge : edges_) {
      for (const RangeHistory& range : edge.second) {
        count += range.history.size();
      }
    }
    return count;
  }

  // Gets a copy of the pose tree where every edge only contains the latest pose
  PoseTree latest() const;

  // Get direct access to all edges stored in the pose tree
  const std::map<std::pair<std::string, std::string>, EdgeHistory>& edges() const { return edges_; }

 private:
  // Breadth-first search to find a path from lhs to rhs at the given timestamp. If it finds a path
  // it will return the list of the node to traverse. If there is no valid path at the given stamp
  // it returns an empty path.
  std::vector<std::string> findPath(const std::string& lhs, const std::string& rhs,
                                    double stamp) const;

  // Returns the Pose3d using the provided path. It interpolates on each edge using the provided
  // stamp.
  Pose3d interpolateOnPath(const std::vector<std::string>& path, double stamp) const;

  std::map<std::pair<std::string, std::string>, EdgeHistory> edges_;
  std::map<std::string, std::set<std::string>> outgoing_;
  // Hold all the date an edge has been added or removed.
  std::set<double> topology_change_stamps_;
};

}  // namespace pose_tree
}  // namespace isaac
