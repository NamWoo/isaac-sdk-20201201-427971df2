/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Polyline2Generator.hpp"

#include <vector>
#include "messages/geometry.hpp"

namespace isaac {
namespace message_generators {

void Polyline2Generator::start() {
  tickPeriodically();
}

void Polyline2Generator::tick() {
  const std::vector<Vector2d> prototype = get_prototype();
  auto lineProto = tx_polyline().initProto().initLine(prototype.size());

  for (size_t i = 0; i < prototype.size(); i++) {
    ToProto(prototype[i], lineProto[i]);
  }

  tx_polyline().publish();
}

}  // namespace message_generators
}  // namespace isaac
