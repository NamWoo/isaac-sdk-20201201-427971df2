/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "engine/gems/serialization/json_formatter.hpp"
#include "packages/composite/gems/quantity.hpp"
#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace serialization {
namespace json_formatter_details {

// Parameter support for composite::Quantity.
template <>
struct JsonSerializer<composite::Quantity> {
  // Try to get quantity from an json array of entity (string), measure (string) and optionally
  // an array of int for dimensions. If dimension is not provided, default to [1]
  // Example:
  // ["elbow", "rotation"]
  // ["elbow", "rotation", [4]]
  static std::optional<composite::Quantity> TryGet(const Json& json) {
    if (!json.is_array()) {
      return std::nullopt;
    }
    if (json.size() != 2 && json.size() != 3) {
      return std::nullopt;
    }

    const std::string entity = json[0].get<std::string>();
    const auto maybe_measure = composite::FromString(json[1].get<std::string>());
    if (!maybe_measure) {
      return std::nullopt;
    }
    if (json.size() == 2) {
      return composite::Quantity::Scalar(entity, *maybe_measure);
    }
    auto maybe_dimension = ::isaac::serialization::TryGet<VectorXi>(json[2]);
    if (!maybe_dimension) {
      return std::nullopt;
    }
    return composite::Quantity{entity, *maybe_measure, *maybe_dimension};
  }
  static void Set(Json& json, const composite::Quantity& quantity) {
    json.push_back(quantity.entity);
    json.push_back(composite::ToString(quantity.measure));
    Json dimensions_json;
    ::isaac::serialization::Set(dimensions_json, quantity.dimensions);
    json.push_back(dimensions_json);
  }
};

// Parameter support for composite::Schema.
template <>
struct JsonSerializer<composite::Schema> {
  // Try to get schema in two ways:
  // Option 1: from an array of quantity as specified in JsonSerializer<composite::Quantity>
  // Example: [["elbow", "position", [3], ["elbow", "rotation", [4]]
  // Option 2: from a list of entities and a measure. Dimension for each quantity default to [1].
  // Example: {"entity": {"elbow","wrist"}, "measure": "rotation}
  static std::optional<composite::Schema> TryGet(const Json& json) {
    if (json.is_array()) {
      // Option 1
      auto quantities = JsonSerializer<std::vector<composite::Quantity>>::TryGet(json);
      if (!quantities) {
        return std::nullopt;
      }
      return composite::Schema(std::move(*quantities));
    } else {
      // Option 2
      const auto maybe_entities = TryGetFromMap<std::vector<std::string>>(json, "entity");
      if (!maybe_entities) {
        return std::nullopt;
      }
      const auto maybe_measure_str = TryGetFromMap<std::string>(json, "measure");
      if (!maybe_measure_str) {
        return std::nullopt;
      }
      const auto maybe_measure = composite::FromString(*maybe_measure_str);
      if (!maybe_measure) {
        return std::nullopt;
      }
      return composite::Schema(*maybe_entities, *maybe_measure);
    }
  }
  static void Set(Json& json, const composite::Schema& schema) {
    JsonSerializer<std::vector<composite::Quantity>>::Set(json, schema.getQuantities());
  }
};

}  // namespace json_formatter_details
}  // namespace serialization
}  // namespace isaac
