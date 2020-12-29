#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xe93f7510a705b53b;

using import "math.capnp".Pose3dProto;

# Pose tree/kinematic tree: hierarchical kinematic representation
struct PoseTreeProto {

  # Pose tree name
  name @0: Text;
  # List of all nodes in tree
  nodes @1: List(Node);
  # List of all edges between nodes in the tree, i.e. adjacency list
  edges @2: List(Edge);

  #  Each node in the tree holds its pose relative to its immediate parent and its pose relative to the root
  struct Node {
    # Node name
    name @0: Text;
    # This node's parent index in the nodes list
    parentIndex @1: Int32;
    # Transformation of node relative to its parent
    parentTNode @2: Pose3dProto;
    # Transformation of node relative to global origin
    worldTNode @3: Pose3dProto;
  }

  # Each edge holds a parent name and child name
  # Every node knows its parent's index but this is for human readability
  struct Edge {
    # Parent name
    parentName @0: Text;
    # Child name
    childName @1: Text;
  }
}
