/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace sight {

// @internal
// A bridge to publish the application pose tree to websight using JSON messages
class PoseTreeJsonBridge : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // The application pose tree published as JSON. Currently a snapshot with the latest poses for
  // every edge in the pose tree is published.
  // The format of the JSON message is as follows:
  //   {
  //     "nodes": ["my_name_1", "my_name_2", "my_name_3", ...]
  //     "edges": [
  //       [0, 1, 0.32, [1.0, 0.0, 0.0, 0.0, -0.5, 1.2, 2.3]],
  //       [1, 2, 0.31, [...]],
  //       ...
  //     ]
  //   }
  // Here "nodes" is a list of strings which are the names for every node.
  // Each entry in "edges" consist of
  //   0: index into the list "nodes" of the name of node A (as one integer)
  //   1: index into the list "nodes" of the name of node B (as one integer)
  //   2: timestamp of latest information for this edge (as one double)
  //   3: pose a_T_b (as 7 doubles in order {qw, qx, qy, qz, px, py, pz})
  ISAAC_RAW_TX(nlohmann::json, pose_tree);
};

}  // namespace sight
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::sight::PoseTreeJsonBridge);
