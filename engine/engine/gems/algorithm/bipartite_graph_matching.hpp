/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// Finds a minimum matching in a weighted bipartite graph. All weights must non-negative.
// The matching is returned as a list of pairs. Each pair will be the indices of matching vertices.
// Runtime: O(n^3) for graphs with 2n vertices (eg. n in each partition).
//
// This algorithm will maintain the following three invariants throughout:
// 1. Every edge e = (x, y) satisfies u(x) + v(y) <= cost(x, y)
//    (if this holds with equality, then we say that e is tight)
// 2. Every edge in the matching will be tight.
// 3. The sum of every element of u and v will be a lower bound on the cost of any minimum matching.
template <typename K>
std::vector<std::pair<int, int>> MinWeightedBipartiteMatching(const MatrixX<K>& cost_matrix) {
  // The empty matching is the best we can do for a graph with no edges.
  if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) return {};

  // U and v will track a lower bound on the cost of a minimum matching.
  // (eg. the minimum matching must cost at least the combined sum of u and v).
  // We initialize them to 0 since this trivially satisfies them.
  const int n = std::max(cost_matrix.rows(), cost_matrix.cols());
  VectorX<K> u = VectorX<K>::Zero(n);
  VectorX<K> v = VectorX<K>::Zero(n);

  // Ensure that the matrix is square by padding it with a large value if needed.
  // By wrapping accesses to the matrix in a lambda function we can avoid copying it to do the
  // padding. Also, we implicity subtract the cost of each vertex.
  const K pad_value = cost_matrix.maxCoeff() + K(1);
  auto cost = [&](int row, int col) {
    if (row >= cost_matrix.rows() || col >= cost_matrix.cols()) {
      return pad_value - u(row) - v(col);
    }
    return cost_matrix(row, col) - u(row) - v(col);
  };

  // These vectors will be used for running Dijkstra's algorithm which is needed to find an
  // augmenting path.
  // dist: the slack on each edge incident to the unmatched vertex we are currently considering
  // parent: nodes which comes before in the augmenting path
  //         we will store the last vertex in the path, and earlier vertices can be read out using
  //         this array
  // seen: indicates whether this vertex has been processed in this iteration
  std::vector<K> dist(n);
  std::vector<int> parent(n);
  std::vector<bool> seen(n);

  // matching represents matches between the row and col vertices
  // (eg. matching[col] gives the corresponding matched row vertex)
  // The value of -1 will denote that a vertex is unmatched.
  std::vector<int> matching(n, -1);

  // Repeat until we have found a perfect matching.
  for (int source = 0; source < n; source++) {
    // Initialize variables needed for Dijkstras algorithm.
    // -1 indicates that no vertices have a parent
    std::fill(parent.begin(), parent.end(), -1);
    // 0 indicates that none of the vertices have been seen
    std::fill(seen.begin(), seen.end(), false);
    for (int i = 0; i < n; ++i) {
      dist[i] = cost(source, i);
    }

    // Run Dijkstras algorithm to find the shortest augmenting path (eg. an alternating path which
    // ends in any unmatched vertex). "Shortest" means that the edges have the least slack.
    // This algorithm works by greedily considering the closest vertex to the source by following
    // edges incident to the source of vertices that have been seen.
    int endpoint;
    while (true) {
      // Find the closest edge which has not been seen.
      int closest = -1;
      for (int k = 0; k < n; k++) {
        if (seen[k]) continue;
        if (closest == -1 || dist[k] < dist[closest]) closest = k;
      }
      ASSERT(closest != -1, "could not find an unseen edge");
      seen[closest] = 1;

      // If this edge is unmatched, we can use it to make an augmenting path.
      if (matching[closest] == -1) {
        endpoint = closest;
        break;
      }

      // Relax unseen neighbors if we can show a cheaper path exists (through the closest vertex).
      for (int i = 0; i < n; i++) {
        if (seen[i]) continue;

        // Consider a path from the source to i through the current vertex to see if it is cheaper.
        // The cost of the alternate path is the cost to get to the closest vertex added to
        // the cost to get from the current vertex to vertex i.
        const int closest_matched_vertex = matching[closest];
        const K alternate_dist = dist[closest] + cost(closest_matched_vertex, i);
        if (dist[i] > alternate_dist) {
          dist[i] = alternate_dist;
          parent[i] = closest;
        }
      }
    }

    // Update dual variables for vertices which were seen, but are not the endpoint. These vertices
    // are all part of the matching already and so their edges are tight.
    for (int i = 0; i < n; i++) {
      if (i == endpoint || !seen[i]) continue;
      // All k (except for the endpoint) must be matched if they have been seen, otherwise
      // we would have found an augmenting path sooner. When updating the dual variables, we ensure
      // that edges in the matching are still tight by adding and subtracting the same quantity.
      const K delta = dist[endpoint] - dist[i];
      v[i] -= delta;
      u[matching[i]] += delta;
    }
    // Since we are adding the source vertex to the matching, we ensure that it is tight.
    u[source] += dist[endpoint];

    // Flip the matched and unmatched edges to increase the size of the matching by 1.
    while (parent[endpoint] >= 0) {
      const int prev = parent[endpoint];
      matching[endpoint] = matching[prev];
      endpoint = prev;
    }
    matching[endpoint] = source;
  }

  // Read out the matching result.
  std::vector<std::pair<int, int>> pairs;
  pairs.reserve(std::min(cost_matrix.rows(), cost_matrix.cols()));
  for (int col = 0; col < cost_matrix.cols(); ++col) {
    const int row = matching[col];
    if (row < cost_matrix.rows()) {
      pairs.push_back({row, col});
    }
  }
  return pairs;
}

// Finds a maximum matching in a weighted bipartite graph.
// The matching is returned as a list of pairs. Each pair will be the indices of matching vertices.
// Runtime: O(n^3) for graphs with 2n vertices (eg. n in each partition).
template <typename K>
std::vector<std::pair<int, int>> MaxWeightedBipartiteMatching(const MatrixX<K>& cost_matrix) {
  const K max_value = cost_matrix.maxCoeff();
  return MinWeightedBipartiteMatching<K>(max_value - cost_matrix.array());
}

}  // namespace isaac
