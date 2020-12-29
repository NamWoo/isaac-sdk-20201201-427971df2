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
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/geometry.hpp"
#include "messages/parking.capnp.h"

namespace isaac {
namespace message_generators {

// Publishes messages to simulate parking spot perception.
// A set of num_detections spots are created in a row along the +X axis.
class ParkingSpotListGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output the generated parking spot list
  ISAAC_PROTO_TX(ParkingSpotListProto, parking_spot_list);

  // The number of parking spot detections to generate
  ISAAC_PARAM(int, num_detections, 3);
  // The width(meters) of the generated parking spot detections
  ISAAC_PARAM(double, spot_width, 3.0);
  // The length (meters) of the generated parking spot detections
  ISAAC_PARAM(double, spot_length, 6.0);
  // The pitch (meters) between the generated parking spot detections
  ISAAC_PARAM(double, spot_pitch, 3.1);
  // The entry line index (0-3) for the generated parking spot detections
  ISAAC_PARAM(int, spot_entry_line_index, 3);
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::ParkingSpotListGenerator);
