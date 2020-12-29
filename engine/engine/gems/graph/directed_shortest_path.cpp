/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "directed_shortest_path.hpp"

#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace graph {

void DirectedShortestPath::addEdge(int from, int to, double dist) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= from && from < num_nodes, "Invalid source index: 0 <= %d < %d", from, num_nodes);
  ASSERT(0 <= to && to < num_nodes, "Invalid destimation index: 0 <= %d < %d", to, num_nodes);
  ISAAC_ASSERT_LT(0.0, dist);
  nodes_[to].in.push_back({dist, from, true});
  nodes_[from].out.push_back({dist, to, true});
}

void DirectedShortestPath::blockEdge(int from, int to) {
  for (auto& node : nodes_[from].out) {
    if (node.node_id == to) {
      // Make sure the edge was not already blocked, otherwise we can return right away.
      if (!node.open) return;
      node.open = false;
      break;
    }
  }
  for (auto& node : nodes_[to].in) {
    if (node.node_id == from) {
      node.open = false;
      break;
    }
  }
  updateBlockNodesFrom(from, to);
}

void DirectedShortestPath::updateBlockNodesFrom(int from, int to) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= from && from < num_nodes, "Invalid source index: 0 <= %d < %d", from, num_nodes);
  ASSERT(0 <= to && to < num_nodes, "Invalid destimation index: 0 <= %d < %d", to, num_nodes);
  // We expand in dfs order from the given edge.
  // We mark all the node that used the blocked edge as not visited yet to explore them again.
  std::vector<std::pair<int, int>> opens = {{from, to}};
  while (!opens.empty()) {
    const int current_node = opens.back().first;
    const int current_parent = opens.back().second;
    opens.pop_back();
    if (dijkstra_visited_[current_node] != virtual_init_ ||
        dijkstra_from_[current_node] != current_parent) {
      // The edge is not part of any optimal path, we don't need to do anything
      continue;
    }
    // The node was part of an optimal path that used the blocked edge. We first mark the edge as
    // not visited (and not processed).
    dijkstra_processed_[current_node] = virtual_init_ - 1;
    dijkstra_visited_[current_node] = virtual_init_ - 1;
    // For all the nodes we could have used to reach this point, we add them back to the list of
    // node to process as we will need to explore them again.
    for (const auto& edge : nodes_[current_node].out) {
      // We only consider edge that are valid
      if (!edge.open) continue;
      // and edge that come from a node that was processed (otherwise the node will get processed at
      // some point in the future without requiring any action from us).
      if (dijkstra_processed_[edge.node_id] != virtual_init_) continue;
      dijkstra_processed_[edge.node_id] = virtual_init_ - 1;
      queue_.insert({dijkstra_distance_[edge.node_id], edge.node_id});
    }
    // Finally we look for other nodes that used this node, and we add them to the list to explore
    // them.
    for (const auto& edge : nodes_[current_node].in) {
      if (edge.open) {
        opens.push_back({edge.node_id, current_node});
      }
    }
  }
}

void DirectedShortestPath::openEdge(int from, int to) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= from && from < num_nodes, "Invalid source index: 0 <= %d < %d", from, num_nodes);
  ASSERT(0 <= to && to < num_nodes, "Invalid destimation index: 0 <= %d < %d", to, num_nodes);
  for (auto& node : nodes_[from].out) {
    if (node.node_id == to) {
      // Make sure the edge was not already open, otherwise we can return right away.
      if (node.open) return;
      node.open = true;
      break;
    }
  }
  for (auto& node : nodes_[to].in) {
    if (node.node_id == from) {
      node.open = true;
      break;
    }
  }
  if (dijkstra_processed_[to] == virtual_init_) {
    dijkstra_processed_[to] = virtual_init_ - 1;
    queue_.insert({dijkstra_distance_[to], to});
  }
}

void DirectedShortestPath::resetNumNodes(int num_nodes) {
  ISAAC_ASSERT_LE(0, num_nodes);
  nodes_.resize(0);  // First erase all the nodes to clear the edges.
  nodes_.resize(num_nodes);
  dijkstra_visited_.resize(num_nodes, 0);
  dijkstra_processed_.resize(num_nodes, 0);
  dijkstra_from_.resize(num_nodes);
  dijkstra_distance_.resize(num_nodes);
  current_target_ = -1;
}

void DirectedShortestPath::resetTarget(int target) {
  if (current_target_ == target) return;
  const int num_nodes = getNumNodes();
  ASSERT(0 <= target && target < num_nodes,
         "Invalid target index: 0 <= %d < %d", target, num_nodes);
  // Make sure the memory has been allocated properly
  if (virtual_init_ == 0) {
    // Reset which node have been visited for the first time, or in the very unlikely case we do
    // 4 Billions planning.
    std::fill(dijkstra_visited_.begin(), dijkstra_visited_.end(), 0);
    std::fill(dijkstra_processed_.begin(), dijkstra_processed_.end(), 0);
  }
  virtual_init_++;
  queue_.clear();
  // Initialize dijkstra by inserting the last node in the queue.
  queue_.insert({0.0, target});
  dijkstra_distance_[target] = 0.0;
  dijkstra_from_[target] = -1;
  dijkstra_visited_[target] = virtual_init_;
  current_target_ = target;
}

std::vector<int> DirectedShortestPath::findShortestPath(int start_id) {
  const int num_nodes = getNumNodes();
  ASSERT(0 <= start_id && start_id < num_nodes,
         "Invalid source index: 0 <= %d < %d", start_id, num_nodes);
  ISAAC_ASSERT_LE(0, current_target_);
  // While we have not reached the start position we keep going
  while (!queue_.empty() &&
         (dijkstra_processed_[start_id] != virtual_init_ ||
          dijkstra_distance_[start_id] > queue_.begin()->weight)) {
    // Get the node with the minimum distance not processed yet.
    const QueueNode current = *queue_.begin();
    queue_.erase(queue_.begin());
    // If we have already processed this node, we can move to the next one
    if (dijkstra_processed_[current.node_id] == virtual_init_) continue;
    // If we have not visited this node (it means some edge was removed), we skip it.
    if (dijkstra_visited_[current.node_id] != virtual_init_) continue;
    // Mark the node as processed, at this point there will be no shorter path from that node.
    dijkstra_processed_[current.node_id] = virtual_init_;
    // For each of the node that lead to the current node, we try to see if we are getting a shorter
    // path.
    for (const auto& node : nodes_[current.node_id].in) {
      if (!node.open) continue;
      const double weight = current.weight + node.weight;
      const int node_idx = node.node_id;
      if (dijkstra_visited_[node_idx] != virtual_init_ ||  // Never visited yet
          dijkstra_distance_[node_idx] > weight) {         // Or we find a better path
        queue_.erase({dijkstra_distance_[node_idx], node_idx});
        dijkstra_visited_[node_idx] = virtual_init_;
        dijkstra_distance_[node_idx] = weight;
        dijkstra_from_[node_idx] = current.node_id;
        dijkstra_processed_[node_idx] = virtual_init_ - 1;
        queue_.insert({weight, node_idx});
      }
    }
  }
  std::vector<int> path;
  if (dijkstra_processed_[start_id] == virtual_init_) {
    // We have reached the start, let's extract the path.
    while (start_id >= 0) {
      path.push_back(start_id);
      start_id = dijkstra_from_[start_id];
    }
  }
  return path;
}

}  // namespace graph
}  // namespace isaac
