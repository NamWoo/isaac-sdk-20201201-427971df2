/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/gems/geometry/types.hpp"
#include "packages/map/Layer.hpp"
#include "packages/map/ObstacleAtlas.hpp"

namespace isaac {
namespace map {

// A map layer which holds annotated polygons and provides various methods to access them
class PolygonMapLayer : public Layer {
 public:
  // A set of named waypoints
  using NamedPolygons = std::map<std::string, geometry::Polygon2D>;

  void start() override;
  Json toJson() const override;

  // Finds a named polygon
  std::optional<geometry::Polygon2D> findByName(const std::string& name) const;

  // Returns the list of all polygons
  std::vector<std::pair<std::string, geometry::Polygon2D>> polygons() const;

  // Returns true if robot at given location touches any polygon
  bool touchesAnyPolygon(const Vector2d& position, float robot_radius) const;

  // Add or update existing layer from JSON
  void addOrUpdateFromJson(const nlohmann::json& json);

  // Adds a polygon
  void addPolygon(const std::string& name, const geometry::Polygon2D& polygon);

  // Adds a list of polygons
  void addPolygons(const std::vector<std::pair<std::string, geometry::Polygon2D>>& polygons);

  // Remove all the polygons of the layers
  void removeAllPolygons();

  // Delete objects from layer
  void deletePolygon(const std::string& point_name);

  // Reset the data for this layer. (Only support 'color' for now)
  void setData(const std::map<std::string, Json>& data);

  // A json object from configuration containing the polygons.
  //
  // .. code-block:: javascript
  //
  //    Layout:
  //      {
  //        "poly1": {
  //           "points": [[<polygon point1>], [<polygon point2>]],
  //         },
  //      }
  ISAAC_PARAM(nlohmann::json, polygons, nlohmann::json::object());
  // Layer color.
  ISAAC_PARAM(Vector3i, color, (Vector3i{120, 120, 120}));
  // Frame the polygons are defined in.
  ISAAC_PARAM(std::string, frame, "world");

  // The maximum distance to consider to create the obstacle from the polygons
  ISAAC_PARAM(double, obstacle_max_distance, 1.5);
  // The resolution of the map used to create the obstacle from the polygons
  ISAAC_PARAM(double, obstacle_pixel_size, 0.1);

 private:
  // Updates JSON and visualization based on stored polygons
  void syncPolygons();
  // Create an obstacle with the list of polygons
  void createObstacle();

  mutable std::mutex mutex_;

  NamedPolygons polygons_;
  ObstacleAtlas* obstacle_atlas_;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::map::PolygonMapLayer);
