/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CameraIntrinsicsGenerator.hpp"

#include <string>
#include <vector>

#include "messages/camera.hpp"

namespace isaac {
namespace message_generators {

void CameraIntrinsicsGenerator::start() {
  tickOnMessage(rx_image());
}

void CameraIntrinsicsGenerator::tick() {
  int64_t acqtime = getTickTimestamp();
  acqtime = rx_image().acqtime();

  // Pinhole for color camera
  auto intrinsics = tx_intrinsics().initProto();
  auto pinhole = intrinsics.initPinhole();
  ToProto(get_focal_length(), pinhole.getFocal());
  ToProto(get_optical_center(), pinhole.getCenter());
  pinhole.setCols(rx_image().getProto().getCols());
  pinhole.setRows(rx_image().getProto().getRows());

  // Distortion parameters for color camera
  auto distortion = intrinsics.initDistortion();
  const std::string distortion_model = get_distortion_model();
  if (distortion_model == "brown") {
    distortion.setModel(DistortionProto::DistortionModel::BROWN);
  } else if (distortion_model == "fisheye") {
    distortion.setModel(DistortionProto::DistortionModel::FISHEYE);
  } else {
    reportFailure("Unsupported Distoration model : %s", distortion_model.c_str());
    return;
  }
  ToProto(get_distortion_coefficients(), distortion.getCoefficients());

  tx_intrinsics().publish(acqtime);
}

}  // namespace message_generators
}  // namespace isaac
