/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/algorithm/bipartite_graph_matching.hpp"
#include "gtest/gtest.h"

namespace isaac {

namespace {

// Check that the two matchings have the same cost.
void CheckMatchingsAreEqual(const std::vector<std::pair<int, int>>& a,
    const std::vector<std::pair<int, int>>& b, const MatrixXd& m) {
  double a_cost = 0.0;
  double b_cost = 0.0;

  for (size_t i = 0; i < a.size(); ++i) {
    a_cost += m(a[i].first, a[i].second);
    b_cost += m(b[i].first, b[i].second);
  }

  EXPECT_NEAR(a_cost, b_cost, 1e-7);
}

}  // namespace

TEST(MinWeightedBipartiteMatching, SquareExample) {
  MatrixXd m(4, 4);
  m << 80.0, 40.0, 50.0, 46.0,
       40.0, 70.0, 20.0, 25.0,
       30.0, 10.0, 20.0, 30.0,
       35.0, 20.0, 25.0, 30.0;
  const std::vector<std::pair<int, int>> correct = {
    {3, 0},
    {2, 1},
    {1, 2},
    {0, 3},
  };
  CheckMatchingsAreEqual(MinWeightedBipartiteMatching(m), correct, m);
}

TEST(MinWeightedBipartiteMatching, NonSquareExample) {
  MatrixXd m(5, 4);
  m << 17.0, 22.0, 19.0, 20.0,
       14.0, 23.0, 15.0, 16.0,
       18.0, 21.0, 17.0, 17.0,
       19.0, 24.0, 21.0, 22.0,
       16.0, 22.0, 17.0, 17.0;
  const std::vector<std::pair<int, int>> correct = {
    {0, 0},
    {2, 1},
    {1, 2},
    {4, 3},
  };
  CheckMatchingsAreEqual(MinWeightedBipartiteMatching(m), correct, m);

  const MatrixXd m_transpose = m.transpose();
  const std::vector<std::pair<int, int>> correct_transposed = {
    {0, 0},
    {1, 2},
    {2, 1},
    {3, 4},
  };
  CheckMatchingsAreEqual(MinWeightedBipartiteMatching(m_transpose),correct_transposed, m_transpose);
}

TEST(MaxWeightedBipartiteMatching, SquareExample) {
  MatrixXd m(3, 3);
  m << 0.0343421, 0.0473545, 10.0488,
       12.1128, 0.0663528, 0.0330071,
       0.091821,   9.48832, 0.0424188;
  const std::vector<std::pair<int, int>> correct = {
    {0, 2},
    {1, 0},
    {2, 1},
  };
  CheckMatchingsAreEqual(MaxWeightedBipartiteMatching(m), correct, m);
}

}  // namespace isaac