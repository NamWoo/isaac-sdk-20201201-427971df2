/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "gtest/gtest.h"
#include "packages/engine_gems/state/state.hpp"

namespace isaac {
namespace state {

template <typename K>
struct FooBarTurImpl : State<K, 3> {
  ISAAC_STATE_VAR(0, foo);
  ISAAC_STATE_VAR(1, bar);
  ISAAC_STATE_VAR(2, tur);
};

using FooBarTur = FooBarTurImpl<double>;

}  // namespace state
}  // namespace isaac
