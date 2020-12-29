/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

// Creates interface for a subgraph.
// Subgraphs JSON files are modular and meaningful collections of nodes, edges, and configurations
// that are ready to be plugged-in when creating application JSON files. For ease of use, each
// subgraph has an interface node of isaac::alice::Subgraph type registered here. Interface node
// receives and transmits messages for the other nodes of the subgraph, so that an application
// using this subgraph can edge with the interface node instead of directly communicating with other
// nodes within the subgraph. In the future, in addition to message passing, interface nodes will
// 1. read parameters set by user and map them to other components in the subgraph,
// 2. pass handles to the behavior trees within the subgraph,
// 3. sync poses between inside and outside the subgraph.
class Subgraph : public Component {};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Subgraph)
