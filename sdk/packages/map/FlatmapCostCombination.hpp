/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <limits>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/map/FlatmapCost.hpp"

namespace isaac {
namespace map {

// Helper FlatmapCost that takes a list of flatmap cost and combines them together.
// This is still an interface and the cost function from FlatmapCost need to be implemented.
class FlatmapCostCombination : public FlatmapCost {
 public:
  // Helper function to load the list of flatmap_cost_layers_ and the minimum and maximum weight.
  void getFlatmapCostLayers();

  // The list of names of the FlatmapCost to be combined.
  // If empty or unset the all the FlatmapCost in the current node will be used.
  ISAAC_PARAM(std::vector<std::string>, flatmap_costs_names);
  // The maximum weight allowed.
  ISAAC_PARAM(double, maximum_weight, std::numeric_limits<double>::max());
  // The minimum weight allowed.
  ISAAC_PARAM(double, minimum_weight, -std::numeric_limits<double>::max());

 protected:
  double maximum_weight_;
  double minimum_weight_;
  std::vector<FlatmapCost*> flatmap_cost_layers_;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::map::FlatmapCostCombination);
