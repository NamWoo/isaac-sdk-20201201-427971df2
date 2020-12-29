/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "packages/map/gems/obstacle.hpp"
#include "packages/map/gems/obstacle_with_pose2.hpp"

namespace isaac {
namespace map {

// A component which holds a virtual representation of obstacles detected around the robot.
// Currently distance maps and spherical obstacles are available. This component is thread safe
// and can be accessed from other components without message passing.
class ObstacleAtlas : public alice::Component {
 public:
  // Class holding an obstacle and information about it.
  struct Obstacle {
    // The (unique) name of the obstacle
    std::string name;
    // The name of the coordinate frame in which this obstacle is located.
    std::string frame;
    // Timestamp when this obstacles was last updated
    int64_t timestamp;
    // Pointer to an obstacle
    std::unique_ptr<map::Obstacle> obstacle;
  };

  // Sets an obstacle. A potentially existing obstacle with the same name will be overwritten.
  void setObstacle(Obstacle&& obstacle);

  // Sets an obstacle. A potentially existing obstacle with the same name will be overwritten.
  void setObstacle(std::string name, std::string frame, int64_t timestamp,
                   std::unique_ptr<map::Obstacle> obstacle_ptr);

  // Removes the obstacle with given name.
  void removeObstacle(const std::string& obstacle_name);

  // Removes all obstacles by clearing the cache
  void removeAllObstacles();

  // Tries to gets an obstacle with a given name. Will return a shared ptr
  // which stays available even if the obstacle is updated afterwards. Will return nullptr in case
  // no obstacle under this name exists.
  std::shared_ptr<const Obstacle> tryGetObstacle(const std::string& name) const;

  // Gets a list of all obstacles of the given type. As the function above, these obstacles will
  // stay valid even if they are updated afterwards.
  template <typename T>
  std::vector<std::shared_ptr<const Obstacle>> getObstacles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<const Obstacle>> obstacles_of_type;
    for (const auto& kvp : obstacles_) {
      if (const T* other = dynamic_cast<const T*>(kvp.second->obstacle.get())) {
        obstacles_of_type.push_back(kvp.second);
      }
    }
    return obstacles_of_type;
  }

  // Retreives a list of obstacles by name
  std::vector<std::shared_ptr<const Obstacle>> getObstacles(
      const std::vector<std::string>& obstacle_names) const;

  // Extract a list of obstacles with the pose to transform the obstacle into the specified frame.
  std::vector<ObstacleWithPose2> extractObstacles(
      const std::vector<std::shared_ptr<const Obstacle>>& obstacles,
      const std::string& frame, double time) const;

  // Frame which can be considered static, it is used to do time synchronization of obstacles.
  ISAAC_PARAM(std::string, static_frame, "world");

 private:
  // A mutex used to provide exclusive access.
  mutable std::mutex mutex_;

  // List of all obstacles
  std::unordered_map<std::string, std::shared_ptr<const Obstacle>> obstacles_;
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::map::ObstacleAtlas)
