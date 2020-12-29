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
namespace alice {

// The lifecycle stage of an object, e.g. a node or a component.
enum class LifecycleStage {
  kNone          =  0,  // The C++ constructor of the object was not yet called.
  kConstructed   =  1,  // The C++ constructor of the object was called.
  kPreStart      = 10,  // The object is starting but not all its parts are started yet.
  kStarted       = 11,  // The object and all its parts are started.
  kPreStopped    = 30,  // The object is stopping but not all its parts are stopped yet.
  kStopped       = 31,  // The object and all its parts are stopped.
  kDeinitialized = 40   // The object and all its parts are deinitialized.
};

}  // namespace alice
}  // namespace isaac
