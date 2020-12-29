/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace icosphere_sampling {

// Number of vertices Vn and the level of subdivision n satisfies Vn = 10*4^n + 2
inline int GetNVertex(int subdivision) {
  return static_cast<int>(std::pow(4.0, subdivision) * 10 + 2);
}

// Number of subdivisions to get at least n vertices.
inline int GetNSubdivision(int n) {
  if (n <= 12) return 0;    // the base icosahedron has 12 vertices
  return static_cast<int>(std::ceil(0.5 * std::log2((n - 2) * 0.1)));
}


// Number of faces Fn and the level of subdivision n satisfies Fn = 20*4^n
inline int GetNFace(int subdivision) {
  return static_cast<int>(std::pow(4.0, subdivision) * 20);
}

// Returns the mid point of v1 and v2, normalizing to radius unless v1 = -v2
template <typename K>
inline Vector3<K> HalfVertex(K radius, const Vector3<K>& v1, const Vector3<K>& v2) {
  Vector3<K> out = (v1 + v2) * 0.5;
  const K norm  = out.norm();
  if (!norm) return out;    // do not normalize if out is a zero vector
  const K scale = radius / norm;
  out *= scale;
  return out;
}

// Try to add an edge (define by a pair of indices) to a map of known edges, and return the
// mapped value. A helper function for Subdivide
int AddEdge(const Vector2i& edge, int val, std::map<std::pair<int, int>, int>& known_edges) {
  // sort the indices of the edge to use as key to search in the map
  const std::pair<int, int> key{std::min(edge[0], edge[1]), std::max(edge[0], edge[1])};
  std::map<std::pair<int, int>, int>::iterator it;
  if ((it = known_edges.find(key)) != known_edges.end()) {
    return it->second;
  } else {
    known_edges.insert({key, val});
    return val;
  }
}

// Fills the vertices and faces vector with the 12 vertices and 20 faces of an Icosahedron
template <typename K>
void Icosahedron(K radius, std::vector<Vector3<K>>& vertices, std::vector<Vector3i>& faces) {
  // vertices coordinate
  const Vector3<K> v{0.0, 1.0, (1.0 + std::sqrt(5.0)) * 0.5};
  const K scale = radius / v.norm();
  const K a = scale * v[0], b = scale * v[1], c = scale * v[2];
  const Vector3<K> kIcosahedronVertices[] = {{-b, c, a}, {b, c, a}, {-b, -c, a}, {b, -c, a},
                                             {a, -b, c}, {a, b, c}, {a, -b, -c}, {a, b, -c},
                                             {c, a, -b}, {c, a, b}, {-c, a, -b}, {-c, a, b}};
  int num_vertex = GetNVertex(0);
  vertices.resize(num_vertex);
  std::copy_n(kIcosahedronVertices, num_vertex, vertices.begin());

  // faces (list of vertex indices)
  const Vector3i kIcosahedronFaces[] = {
      {0, 11, 5},  {0, 5, 1},  {0, 1, 7},  {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
      {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
      {3, 8, 9},   {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
  int num_faces = GetNFace(0);
  faces.resize(num_faces);
  std::copy_n(kIcosahedronFaces, num_faces, faces.begin());
}

// Subdivide a icosahedron mesh multiple times. Every triangle is split into 4 equal sub-triangles
// in each division. Vertices and faces are modified to return the 3d coordinates of all the
// vertices, and the list of vertex indices of all the triangles.
template <typename K>
void Subdivide(int subdivision, K radius, std::vector<Vector3<K>>& vertices,
               std::vector<Vector3i>& faces) {
  int current_vertex = vertices.size();
  vertices.resize(GetNVertex(subdivision));

  int current_face = faces.size();
  faces.resize(GetNFace(subdivision));
  // map from a processed edge to its midpoint index
  std::map<std::pair<int, int>, int> edges;
  int index;

  // iterate all subdivision levels
  for (int i = 0; i < subdivision; ++i) {
    // copy prev vertex/index arrays and clear
    edges.clear();
    // perform subdivision for each triangle
    for (int j = current_face - 1; j >= 0; j--) {
      const auto& indices = faces[j];
      Vector3i new_indices;  // vertex indices of the three midpoints of the current triangle
      // get midpoints of three edges of the current face
      for (int k = 0; k < 3; k++) {
        Vector2i edge = {indices[k], indices[(k + 1) % 3]};
        // check if midpoint of edge has been calculated. if not, append it to vertices
        if ((index = AddEdge(edge, current_vertex, edges)) == current_vertex) {
          vertices[current_vertex++] = HalfVertex(radius, vertices[edge[0]], vertices[edge[1]]);
        }
        new_indices[k] = index;
      }
      // add the 4 sub-triangles
      faces[j * 4 + 3] = Vector3i{indices[0], new_indices[0], new_indices[2]};
      faces[j * 4 + 2] = Vector3i{indices[1], new_indices[1], new_indices[0]};
      faces[j * 4 + 1] = Vector3i{indices[2], new_indices[2], new_indices[1]};
      faces[j * 4 + 0] = new_indices;
    }
    current_face *= 4;
  }
}

}  // namespace icosphere_sampling

// Samples at least n points from a sphere of radius using Icosahedron subdivision. Returns a
// vector of the sampled point's 3d coordinates. If ordered is true, the result is sorted by
// z value. Otherwise it is ordered by subdivision level.
template <typename K>
std::vector<Vector3<K>> IcosphereSample(int n, K radius, const Vector3<K>& center,
                                        bool ordered = false) {
  static_assert(std::is_floating_point<K>::value, "Type must be float, double or long double");
  // get vertices from a Icosahedron
  std::vector<Vector3<K>> vertices;
  std::vector<Vector3i> faces;
  icosphere_sampling::Icosahedron(radius, vertices, faces);
  if (n > static_cast<int>(vertices.size())) {
    // Perform subdivision.
    const int subdivision = icosphere_sampling::GetNSubdivision(n);
    icosphere_sampling::Subdivide(subdivision, radius, vertices, faces);
  }
  if (!IsAlmostZero(center.squaredNorm())) {
    // Translate to the non-zero center
    for (Vector3<K>& vertex : vertices) {
      vertex += center;
    }
  }
  if (ordered) {
    std::sort(vertices.begin(), vertices.end(),
              [](const Vector3<K>& a, const Vector3<K>& b) { return a[2] < b[2]; });
  }
  return vertices;
}

}  // namespace isaac
