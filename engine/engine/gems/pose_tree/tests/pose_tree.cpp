/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <vector>

#include "engine/gems/interpolation/poses.hpp"
#include "engine/gems/math/pose_utils.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace pose_tree {

namespace {

std::default_random_engine s_rng;
Vector4d sigma(1.0, 1.0, 1.0, 0.4);

}  // namespace

TEST(PoseTree, Id) {
  for (int i=0; i<10; i++) {
    const std::string lhs = "lhs";
    const std::string rhs = "rhs";
    const Pose3d actual = PoseNormalDistribution(sigma, s_rng);
    PoseTree pg;
    pg.set(lhs, rhs, 0.0, actual);
    EXPECT_TRUE(pg.hasDirectConnection(lhs, rhs, 0.0));
    EXPECT_TRUE(pg.hasDirectConnection(lhs, lhs, 0.0));
    EXPECT_TRUE(pg.hasDirectConnection(rhs, rhs, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(lhs, rhs, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(lhs, lhs, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(rhs, rhs, 0.0));
    auto maybe = pg.get(rhs, rhs, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, Pose3d::Identity(), 1e-12);
    maybe = pg.get(lhs, lhs, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, Pose3d::Identity(), 1e-12);
  }
}

TEST(PoseTree, TwoNodes) {
  for (int i=0; i<10; i++) {
    const std::string lhs = "lhs";
    const std::string rhs = "rhs";
    const Pose3d actual = PoseNormalDistribution(sigma, s_rng);
    PoseTree pg;
    pg.set(lhs, rhs, 0.0, actual);
    auto maybe = pg.get(lhs, rhs, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, actual, 1e-12);
    maybe = pg.get(rhs, lhs, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, actual.inverse(), 1e-12);
  }
}

TEST(PoseTree, ThreeNodes) {
  for (int i=0; i<10; i++) {
    const std::string a = "a";
    const std::string b = "b";
    const std::string c = "c";
    const Pose3d aTb = PoseNormalDistribution(sigma, s_rng);
    const Pose3d bTc = PoseNormalDistribution(sigma, s_rng);
    const Pose3d aTc = aTb * bTc;
    PoseTree pg;
    pg.set(a, b, 0.0, aTb);
    pg.set(b, c, 0.0, bTc);
    EXPECT_TRUE(pg.hasDirectConnection(a, b, 0.0));
    EXPECT_TRUE(pg.hasDirectConnection(b, c, 0.0));
    EXPECT_FALSE(pg.hasDirectConnection(a, c, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(a, b, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(b, c, 0.0));
    EXPECT_TRUE(pg.hasIndirectConnection(a, c, 0.0));
    auto maybe = pg.get(a, c, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, aTc, 1e-12);
    maybe = pg.get(c, a, 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, aTc.inverse(), 1e-12);
  }
}

TEST(PoseTree, Tree) {
  // Create a pose graph which looks like a tree and go from top to bottom.
  constexpr int kRepetitions = 10;
  constexpr int kDepth = 10;
  constexpr int kBranching = 10;
  for (int i=0; i<kRepetitions; i++) {
    PoseTree pg;
    Pose3d actual = Pose3d::Identity();
    std::vector<std::vector<std::string>> nodes;
    nodes.push_back({"nodes_" + std::to_string(i)});
    for (int d=1; d<kDepth; d++) {
      nodes.push_back(std::vector<std::string>{});
      for (size_t b=0; b<kBranching; b++) {
        const Pose3d pose = PoseNormalDistribution(sigma, s_rng);
        nodes[d].push_back(
            "branches_" + std::to_string(i) + "_" + std::to_string(d) + "_" + std::to_string(b));
        pg.set(nodes[d-1][0], nodes[d][b], 0.0, pose);
        if (b == 0) {
          actual = actual * pose;
        }
      }
    }
    auto maybe = pg.get(nodes[0][0], nodes[kDepth-1][0], 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, actual, 1e-12);
    maybe = pg.get(nodes[kDepth-1][0], nodes[0][0], 0.0);
    ASSERT_TRUE(maybe);
    ISAAC_EXPECT_POSE_NEAR(*maybe, actual.inverse(), 1e-12);
  }
}

TEST(PoseTree, CheckCycle) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  PoseTree pg;
  EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Identity()));
  EXPECT_TRUE(pg.set(b, c, 0.0, Pose3d::Identity()));
  EXPECT_FALSE(pg.set(a, c, 0.0, Pose3d::Identity()));
}

TEST(PoseTree, CheckOrderedUpdates) {
  const std::string a = "a";
  const std::string b = "b";
  PoseTree pg;
  EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Translation(Vector3d(1.0, 0.0, 0.0))));
  EXPECT_TRUE(pg.set(a, b, 1.0, Pose3d::Translation(Vector3d(2.0, 0.0, 0.0))));
  EXPECT_TRUE(pg.set(a, b, 1.0, Pose3d::Translation(Vector3d(3.0, 0.0, 0.0))));
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 1.0), Pose3d::Translation(Vector3d(3.0, 0.0, 0.0)), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, a, 1.0), Pose3d::Translation(Vector3d(-3.0, 0.0, 0.0)), 1e-12);
  EXPECT_TRUE(pg.set(a, b, 2.0, Pose3d::Translation(Vector3d(4.0, 0.0, 0.0))));
  EXPECT_FALSE(pg.set(a, b, 1.5, Pose3d::Identity()));
}

TEST(PoseTree, TemporalDirect) {
  const std::string a = "a";
  const std::string b = "b";
  const Pose3d aTb0 = Pose3d::Translation({0.7, -1.3, 2.0});
  const Pose3d aTb1 = Pose3d::Translation({-0.7, -1.6, 2.4});
  PoseTree pg;
  pg.set(a, b, 0.0, aTb0);
  pg.set(a, b, 1.0, aTb1);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.0), aTb0, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 1.0), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 12.6), aTb1, 1e-12);
  EXPECT_FALSE(pg.hasDirectConnection(a, b, -0.1));
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.5), Pose3d::Translation({0.0, -1.45, 2.2}), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.3), Interpolate(0.3, aTb0, aTb1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.6), Interpolate(0.6, aTb0, aTb1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.9), Interpolate(0.9, aTb0, aTb1), 1e-12);
}

TEST(PoseTree, TemporalIndirect) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  const Pose3d aTb0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d aTb1 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc1 = PoseNormalDistribution(sigma, s_rng);
  PoseTree pg;
  pg.set(a, b, 0.0, aTb0);
  pg.set(a, b, 1.0, aTb1);
  pg.set(b, c, 0.0, bTc0);
  pg.set(b, c, 1.0, bTc1);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.0), aTb0, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 1.0), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 12.6), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 0.0), bTc0, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 1.0), bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 12.6), bTc1, 1e-12);
  EXPECT_FALSE(pg.hasIndirectConnection(b, c, -0.1));
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.3), Interpolate(0.3, aTb0, aTb1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.5), Interpolate(0.5, aTb0, aTb1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.9), Interpolate(0.9, aTb0, aTb1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.3), Interpolate(0.3, aTb0, aTb1) * Interpolate(0.3, bTc0, bTc1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.5), Interpolate(0.5, aTb0, aTb1) * Interpolate(0.5, bTc0, bTc1), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.9), Interpolate(0.9, aTb0, aTb1) * Interpolate(0.9, bTc0, bTc1), 1e-12);
}

TEST(PoseTree, Latest) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  const Pose3d aTb0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d aTb1 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc1 = PoseNormalDistribution(sigma, s_rng);
  PoseTree original;
  original.set(a, b, 0.0, aTb0);
  original.set(a, b, 1.0, aTb1);
  original.set(b, c, 0.0, bTc0);
  original.set(b, c, 1.0, bTc1);

  PoseTree pg = original.latest();
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.0), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 1.0), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 12.6), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 0.0), bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 1.0), bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(b, c, 12.6), bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.3), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.5), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, b, 0.9), aTb1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.3), aTb1 * bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.5), aTb1 * bTc1, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, c, 0.9), aTb1 * bTc1, 1e-12);
}

TEST(PoseTree, getLatest) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  const std::string d = "d";
  const std::string e = "e";
  const std::string f = "f";
  const Pose3d aTb0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d aTb1 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d bTc1 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d cTd0 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d cTd1 = PoseNormalDistribution(sigma, s_rng);
  const Pose3d eTf0 = PoseNormalDistribution(sigma, s_rng);
  PoseTree pg;
  pg.set(a, b, 0.0, aTb0);
  pg.set(a, b, 2.0, aTb1);
  pg.set(b, c, 0.0, bTc0);
  pg.set(b, c, 2.5, bTc1);
  pg.set(c, d, 0.0, cTd0);
  pg.set(c, d, 1.0, cTd1);
  pg.set(e, f, 5.0, eTf0);

  EXPECT_NEAR(pg.getLatest(a, b)->second, 2.0, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(pg.getLatest(a, b)->first, aTb1, 1e-12);

  EXPECT_NEAR(pg.getLatest(a, c)->second, 2.5, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(pg.getLatest(a, c)->first, aTb1 * bTc1, 1e-12);

  EXPECT_NEAR(pg.getLatest(a, d)->second, 2.5, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(pg.getLatest(a, d)->first, aTb1 * bTc1 * cTd1, 1e-12);

  EXPECT_EQ(std::nullopt, pg.getLatest(a, e));
}

TEST(PoseTree, removeEdge) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  {
    PoseTree pg;
    EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(b, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.set(a, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.removeEdge(a, b, 0.0));
    EXPECT_TRUE(pg.removeEdge(a, b, 1.0));
    EXPECT_FALSE(pg.removeEdge(a, b, 1.0));
    EXPECT_TRUE(pg.hasIndirectConnection(a, c, 0.0));
    EXPECT_TRUE(pg.hasIndirectConnection(c, a, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(a, c, 1.0));
    EXPECT_FALSE(pg.hasIndirectConnection(c, a, 1.0));
    EXPECT_FALSE(pg.set(a, c, 0.99, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(a, c, 1.0, Pose3d::Identity()));
  }
  {
    PoseTree pg;
    EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(b, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.set(a, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.removeEdge(a, c, 1.0));
    EXPECT_TRUE(pg.removeEdge(c, b, 1.0));
    EXPECT_TRUE(pg.hasIndirectConnection(a, c, 0.0));
    EXPECT_TRUE(pg.hasIndirectConnection(c, a, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(a, c, 1.0));
    EXPECT_FALSE(pg.hasIndirectConnection(c, a, 1.0));
    EXPECT_FALSE(pg.set(a, c, 0.99, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(a, c, 1.0, Pose3d::Identity()));
  }
}

TEST(PoseTree, removeFrame) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  {
    PoseTree pg;
    EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(b, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.set(a, c, 0.0, Pose3d::Identity()));
    EXPECT_FALSE(pg.removeFrame(b, 0.0));
    EXPECT_TRUE(pg.removeFrame(b, 1.0));
    EXPECT_TRUE(pg.hasIndirectConnection(a, c, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(a, c, 1.0));
    EXPECT_TRUE(pg.hasIndirectConnection(c, a, 0.0));
    EXPECT_FALSE(pg.hasIndirectConnection(c, a, 1.0));

    EXPECT_TRUE(pg.hasDirectConnection(a, b, 0.0));
    EXPECT_FALSE(pg.hasDirectConnection(a, b, 1.0));
    EXPECT_TRUE(pg.hasDirectConnection(b, a, 0.0));
    EXPECT_FALSE(pg.hasDirectConnection(b, a, 1.0));
    EXPECT_TRUE(pg.set(a, c, 1.0, Pose3d::Identity()));
    EXPECT_TRUE(pg.set(a, b, 1.0, Pose3d::Identity()));
  }
}

TEST(PoseTree, removeEdgeAndQueries) {
  const std::string a = "a";
  const std::string b = "b";
  const std::string c = "c";
  const std::string d = "d";
  PoseTree pg;
  EXPECT_TRUE(pg.set(a, b, 0.0, Pose3d::Translation(Vector3d(1.0, 0.0, 0.0))));
  EXPECT_TRUE(pg.set(b, c, 0.0, Pose3d::Translation(Vector3d(0.0, 1.0, 0.0))));
  EXPECT_TRUE(pg.set(c, d, 0.0, Pose3d::Translation(Vector3d(0.0, 0.0, 1.0))));
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 1.0), Pose3d::Translation(Vector3d(1.0, 1.0, 1.0)), 1e-12);
  pg.removeEdge(b, c, 1.0);
  EXPECT_EQ(pg.get(a, d, 1.0), std::nullopt);
  EXPECT_TRUE(pg.set(b, c, 2.0, Pose3d::Translation(Vector3d(0.0, 2.0, 0.0))));
  pg.removeEdge(b, c, 3.0);
  EXPECT_TRUE(pg.set(b, c, 4.0, Pose3d::Translation(Vector3d(0.0, 4.0, 0.0))));
  EXPECT_TRUE(pg.set(b, c, 5.0, Pose3d::Translation(Vector3d(0.0, 5.0, 0.0))));
  pg.removeEdge(b, c, 6.0);
  EXPECT_TRUE(pg.set(b, c, 7.0, Pose3d::Translation(Vector3d(0.0, 7.0, 0.0))));
  pg.removeEdge(b, c, 8.0);
  EXPECT_TRUE(pg.set(b, c, 8.0, Pose3d::Translation(Vector3d(0.0, 8.0, 0.0))));

  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 2.5), Pose3d::Translation(Vector3d(1.0, 2.0, 1.0)), 1e-12);
  EXPECT_EQ(pg.get(a, d, 3.5), std::nullopt);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 4.5), Pose3d::Translation(Vector3d(1.0, 4.5, 1.0)), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 5.5), Pose3d::Translation(Vector3d(1.0, 5.0, 1.0)), 1e-12);
  EXPECT_EQ(pg.get(a, d, 6.5), std::nullopt);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 7.99), Pose3d::Translation(Vector3d(1.0, 7.0, 1.0)), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(*pg.get(a, d, 8.0), Pose3d::Translation(Vector3d(1.0, 8.0, 1.0)), 1e-12);
}

}  // namespace pose_tree
}  // namespace isaac
