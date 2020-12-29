/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

namespace isaac {
namespace path_planner {

// Interface implementing a global planner model.
// It needs to provide helper functions to help compute a global path between two states
template <typename K, class R>
class PlannerModel {
 public:
  virtual ~PlannerModel() = default;

  // Returns whether or not the robot is in a valid configuration.
  virtual bool validState(const R& state) = 0;

  // Returns the distance between two states.
  virtual K distance(const R& start, const R& end) = 0;

  // Returns whether or not the 'direct' path between start and end is collision free.
  // This class define what is a direct path for the given problem and the planner will provide a
  // list of configuration as a path such as the direct path between two consecutives states is
  // collision free.
  virtual bool validDirectPath(const R& start, const R& end) = 0;

  // Returns the distance to the nearest obstacle and populate the gradient if provided.
  virtual K distanceToObstacle(const R& state, R* gradient = nullptr) = 0;

  // Returns a random state.
  // Note: the state does not have to be fully random nor uniform random, but in order to ensure the
  // planner eventually find a path if such path exist, all the state needs to have a chance to be
  // returned.
  virtual R randomState() = 0;

  // Returns true if and only if the control is reversible, this means that:
  // - distance(a, b) == distance(b, a)
  // - validDirectPath(a, b) == validDirectPath(b, a)
  virtual bool isReversible() { return true; }
};

}  // namespace path_planner
}  // namespace isaac
