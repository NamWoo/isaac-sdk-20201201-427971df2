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

class Application;

// Manages event distribution
class EventManager  {
 public:
  EventManager(Application* app);
  // Notifies about a status update
  void onStatusUpdate(Component* component);

 private:
  Application* app_;
};

}  // namespace alice
}  // namespace isaac
