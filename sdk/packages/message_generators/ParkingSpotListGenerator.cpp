/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ParkingSpotListGenerator.hpp"

#include "messages/geometry.hpp"

namespace isaac {
namespace message_generators {

void ParkingSpotListGenerator::start() {
  tickPeriodically();
}

void ParkingSpotListGenerator::tick() {
  auto parking_spot_list = tx_parking_spot_list().initProto();
  auto detections = parking_spot_list.initDetections(get_num_detections());

  const double width = get_spot_width();
  const double length = get_spot_length();
  const double pitch = get_spot_pitch();
  const int entry_line_index = get_spot_entry_line_index();

  uint32_t index = 0;
  for (auto detection : detections) {
    const double offset = static_cast<double>(index++) * pitch;

    geometry::Polyline2d boxLine;
    boxLine.push_back(Vector2d(0.0, offset));
    boxLine.push_back(Vector2d(length, offset));
    boxLine.push_back(Vector2d(length, width + offset));
    boxLine.push_back(Vector2d(0.0, width + offset));

    ToProto(boxLine, detection.initBox());
    detection.setEntryLineIndex(entry_line_index);
    detection.setConfidence(1.0f);
  }

  tx_parking_spot_list().publish();
}

}  // namespace message_generators
}  // namespace isaac
