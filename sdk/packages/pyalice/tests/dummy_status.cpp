/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "dummy_status.hpp"

namespace isaac {
namespace dummy {

void DummyStatus::start() {
  tickPeriodically(0.1);
}

void DummyStatus::tick() {
  const auto status = get_status();
  LOG_WARNING("Status %d", status);
  if (status > 0) {
    reportSuccess();
  } else if (status < 0) {
    reportFailure();
  }
}

}  // namespace dummy
}  // namespace isaac
