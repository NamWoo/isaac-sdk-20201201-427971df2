#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xaec0fa9c38fbb1ff;

using import "math.capnp".Pose2fProto;

# A general graph
struct GraphProto {
  # The number of nodes in the graph. Indices in the list of edges will
  # always be smaller than this number.
  nodeCount @0: Int32;
  # An edge in the graph connects two nodes.
  struct EdgeProto {
    source @0: Int32;
    target @1: Int32;
  }
  # The edges of the graph.
  edges @1: List(EdgeProto);
}

# A graph of Pose2f represented by a list of poses and a list of weighted edges between
# pair of nodes.
struct Pose2GraphProto {
  # The list of edges of the graph and the number of nodes.
  graph @0: GraphProto;
  # The list of the nodes of the graph. Each node corresponds to a Pose2f. The size of the list must
  # match graph.getNodeCount().
  nodes @1: List(Pose2fProto);
  # Optional list of the weight for each edge. If it is not empty, the size must match:
  # graph.getEdges().size()
  weights @2: List(Float32);
}