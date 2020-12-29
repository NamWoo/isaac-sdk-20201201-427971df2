/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ObstacleAtlas.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace isaac {
namespace map {

void ObstacleAtlas::setObstacle(Obstacle&& obstacle) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string name = obstacle.name;
  obstacles_[name] = std::make_shared<const Obstacle>(std::move(obstacle));
}

void ObstacleAtlas::setObstacle(std::string name, std::string frame, int64_t timestamp,
                                std::unique_ptr<map::Obstacle> obstacle_ptr) {
  Obstacle obstacle;
  obstacle.name = std::move(name);
  obstacle.frame = std::move(frame);
  obstacle.timestamp = timestamp;
  obstacle.obstacle = std::move(obstacle_ptr);
  setObstacle(std::move(obstacle));
}

void ObstacleAtlas::removeObstacle(const std::string& obstacle_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.erase(obstacle_name);
}

void ObstacleAtlas::removeAllObstacles() {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
}

std::shared_ptr<const ObstacleAtlas::Obstacle> ObstacleAtlas::tryGetObstacle(
    const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = obstacles_.find(name);
  if (it == obstacles_.end()) return nullptr;
  return it->second;
}

std::vector<std::shared_ptr<const ObstacleAtlas::Obstacle>>
ObstacleAtlas::getObstacles(const std::vector<std::string>& obstacle_names) const {
  std::vector<std::shared_ptr<const Obstacle>> obstacles;
  obstacles.reserve(obstacle_names.size());
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& name : obstacle_names) {
    const auto it = obstacles_.find(name);
    if (it != obstacles_.end()) {
      obstacles.push_back(it->second);
    }
  }
  return obstacles;
}

std::vector<ObstacleWithPose2> ObstacleAtlas::extractObstacles(
    const std::vector<std::shared_ptr<const Obstacle>>& obstacles,
    const std::string& frame, double time) const {
  // Get transformation to static frame
  const std::string static_frame = get_static_frame();
  auto static_frame_T_frame = node()->pose().tryGetPose2XY(static_frame, frame, time);
  if (!static_frame_T_frame) {
    LOG_WARNING("The frame %s and %s are not connected in the PoseTree",
                static_frame.c_str(), frame.c_str());
    return {};
  }

  // Extract obstacles and compute the corresponding frame
  std::vector<ObstacleWithPose2> ret;
  ret.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    auto obsacle_frame_T_static_frame = node()->pose().tryGetPose2XY(
        obstacle->frame, static_frame, ToSeconds(obstacle->timestamp));
    if (obsacle_frame_T_static_frame) {
      ret.push_back({obstacle->obstacle.get(),
                    (*obsacle_frame_T_static_frame) * (*static_frame_T_frame)});
    }
  }
  return ret;
}

}  // namespace map
}  // namespace isaac
