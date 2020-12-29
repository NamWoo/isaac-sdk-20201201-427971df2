/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToCameraIntrinsics.hpp"

#include <vector>

#include "engine/gems/geometry/pinhole.hpp"

namespace isaac {
namespace ros_bridge {

bool RosToCameraIntrinsics::rosToProto(const sensor_msgs::CameraInfo::ConstPtr& ros_message,
                                       std::optional<ros::Time>& ros_time,
                                       alice::ProtoTx<CameraIntrinsicsProto>& tx_proto) {
  ros_time = ros_message->header.stamp;
  auto builder = tx_proto.initProto();

  // Distortion parameters
  if (ros_message->distortion_model == "plumb_bob") {
    auto distortion = builder.initDistortion();
    distortion.setModel(DistortionProto::DistortionModel::BROWN);
    Vector5d disto;
    disto << ros_message->D[0], ros_message->D[1], ros_message->D[2], ros_message->D[3],
             ros_message->D[4];
    ToProto(disto, distortion.getCoefficients());
  } else {
    reportFailure("unsupported distoration model %s", ros_message->distortion_model.c_str());
    return false;
  }

  // Pinhole camera model parameters
  const Vector2i dimensions{ros_message->height, ros_message->width};
  const Vector2d focal{ros_message->K[0], ros_message->K[4]};
  const Vector2d center{ros_message->K[2], ros_message->K[5]};
  const geometry::Pinhole<double> pinhole{dimensions, focal, center};
  ToProto(pinhole, builder.initPinhole());

  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
