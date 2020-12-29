/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "SimpleLed.hpp"

#include "engine/core/math/types.hpp"
#include "messages/math.hpp"

namespace isaac {

void SimpleLed::start() {
  tickPeriodically();
  fill_color_ = "red";
}

void SimpleLed::tick() {
  // cycle through red - green - blue on each tick

  auto led_strip = tx_led_strip().initProto();
  led_strip.setEnabled(true);

  Vector3ub color;

  if (fill_color_ == "red") {
    color = Vector3ub(255, 0, 0);
    fill_color_ = "green";

  } else if (fill_color_ == "green") {
    color = Vector3ub(0, 255, 0);
    fill_color_ = "blue";

  } else if (fill_color_ == "blue") {
    color = Vector3ub(0, 0, 255);
    fill_color_ = "red";
  }

  // set the color for all LEDs on strip
  ToProto(color, led_strip.initColor());

  // set the skip and offset parameters to 0
  led_strip.setSkip(0);
  led_strip.setOffset(0);

  tx_led_strip().publish();
}

}  // namespace isaac
