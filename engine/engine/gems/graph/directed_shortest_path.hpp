/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <set>
#include <utility>
#include <vector>

namespace isaac {
namespace graph {

// Helper class to compute the shortest path in a directed graph based on Dijkstra algorithm.
// It takes the number of nodes, the list of weighted directed edge (weights needs to be positive)
// and compute the shortest path from every position to a given target location.
// It also keeps track of what was computed in the previous query to reuse it if the target has not
// changed.
class DirectedShortestPath {
 public:
  // Helper structure to represent an edge.
  struct Edge {
    // Weight of the edge
    double weight;
    // Id of the destionation node
    int node_id;
    // Whether or not the edge is currently open.
    bool open;
  };
  // Helper structure to represent a node
  struct Node {
    std::vector<Edge> in;
    std::vector<Edge> out;
  };

  // Default constructor
  DirectedShortestPath() = default;

  // Constructor to initialize with a set number of nodes.
  DirectedShortestPath(int num_nodes) { resetNumNodes(num_nodes); }

  // Resets the graph with a given number of nodes
  void resetNumNodes(int num_nodes);

  // Adds a directed edge between two nodes (from -> to) with a given weight.
  void addEdge(int from, int to, double weight);

  // Sets the current target node.
  void resetTarget(int target);

  // Marks an edge between two nodes (from -> out) to be be blocked.
  void blockEdge(int from, int out);

  // Marks an edge between two nodes (from -> out) to be be open.
  void openEdge(int from, int out);

  // Returns the shortest path from start_id to the current target
  std::vector<int> findShortestPath(int start_id);

  // Returns the number of nodes in the graph.
  int getNumNodes() const { return static_cast<int>(nodes_.size()); }

  // Returns the structure of the given node.
  const Node& getNode(int node_id) const { return nodes_[node_id]; }

 private:
  // Helper to describe a node in our queue.
  struct QueueNode {
    double weight;
    int node_id;
    bool operator<(const QueueNode& node) const {
      return weight < node.weight || (node.weight == weight && node_id < node.node_id);
    }
  };

  // Updates all the best paths that were using the edge from -> to.
  void updateBlockNodesFrom(int from, int to);

  // Hold the structure of the graph:
  std::vector<Node> nodes_;
  // Current target for the incoming queries
  int current_target_ = -1;
  // Helper to reinitialize in constant time when we change the target.
  int virtual_init_ = 0;
  // Contains the value virtual_init_ once a node has been fully processed for a given target.
  // At that point it means dijkstra_distance_ contains the smallest distance to the target.
  std::vector<int> dijkstra_processed_;
  // Contains the value virtual_init_ once dijkstra_distance_ has been reset with a value at least
  // as big as the smallest distance to the target.
  std::vector<int> dijkstra_visited_;
  // Contains the direction to go to reach the target. (Only valid once dijkstra_visited_ contains
  // the value virtual_init_)
  std::vector<int> dijkstra_from_;
  // Contains the current distance to go to reach the target. (Only valid once dijkstra_visited_
  // contains the value virtual_init_). Once dijkstra_processed_ == virtual_init_, it contains the
  // smallest distance to reach the target
  std::vector<double> dijkstra_distance_;
  // Current queue of node that needs to be explored.
  std::set<QueueNode> queue_;
};

}  // namespace graph
}  // namespace isaac
