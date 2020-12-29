/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "FlatmapCostCombination.hpp"

#include "engine/core/math/utils.hpp"

namespace isaac {
namespace map {

void FlatmapCostCombination::getFlatmapCostLayers() {
  maximum_weight_ = get_maximum_weight();
  minimum_weight_ = get_minimum_weight();
  ISAAC_ASSERT_GT(maximum_weight_, minimum_weight_);
  flatmap_cost_layers_.clear();
  const auto maybe_flatmap_costs_names = try_get_flatmap_costs_names();
  if (maybe_flatmap_costs_names == std::nullopt) {
    flatmap_cost_layers_ = node()->getComponents<FlatmapCost>();
    // Make sure the current codelet is not added to the list
    for (size_t id = 0; id < flatmap_cost_layers_.size(); id++) {
      if (flatmap_cost_layers_[id]->name() == this->name()) {
        flatmap_cost_layers_[id] = flatmap_cost_layers_.back();
        flatmap_cost_layers_.pop_back();
        break;
      }
    }
  } else {
    for (const auto& name : *maybe_flatmap_costs_names) {
      auto* layer = node()->app()->findComponentByName<FlatmapCost>(name);
      ASSERT(layer != nullptr, "Could not load: %s", name.c_str());
      flatmap_cost_layers_.push_back(layer);
    }
  }
}

}  // namespace map
}  // namespace isaac
