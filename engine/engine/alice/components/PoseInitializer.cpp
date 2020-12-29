/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/components/PoseInitializer.hpp"

#include "engine/core/constants.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

void PoseInitializer::start() {
  // Set the pose at current timestamp
  if (!node()->pose().trySet(get_lhs_frame(), get_rhs_frame(), get_pose(), getTickTime())) {
    if (!get_disable_failure()) {
      reportFailure("Failed to set: %s_T_%s at time %lf",
                    get_lhs_frame().c_str(), get_rhs_frame().c_str(), getTickTime());
    }
  }
  if (get_report_success()) {
    reportSuccess();
  }
  tickPeriodically(0.2);  // Should tick on configuration change.
}

void PoseInitializer::tick() {
  if (!get_attach_interactive_marker()) {
    const Pose3d pose_corrected
        = Pose3d::Rotation(Vector3d{0, 0, 1}, DegToRad(get_add_roll_degrees()))
        * Pose3d::Rotation(Vector3d{0, 1, 0}, DegToRad(get_add_pitch_degrees()))
        * Pose3d::Rotation(Vector3d{1, 0, 0}, DegToRad(get_add_yaw_degrees()))
        * get_pose();
    if (!node()->pose().trySet(get_lhs_frame(), get_rhs_frame(), pose_corrected, getTickTime()) &&
        !get_disable_failure()) {
      reportFailure("Failed to set: %s_T_%s at time %lf",
                    get_lhs_frame().c_str(), get_rhs_frame().c_str(), getTickTime());
    }
  }
}

}  // namespace alice
}  // namespace isaac
