/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CameraIntrinsicsToRos.hpp"

#include "messages/camera.hpp"

namespace isaac {
namespace ros_bridge {

bool CameraIntrinsicsToRos::protoToRos(const alice::ProtoRx<CameraIntrinsicsProto>& rx_proto,
                                       const ros::Time& ros_time,
                                       sensor_msgs::CameraInfo& ros_message) {
  // Read from Isaac message
  auto reader = rx_proto.getProto();
  geometry::PinholeD pinhole;
  if (!FromProto(reader, pinhole)) {
    reportFailure("Failed to parse pinhole from message");
    return false;
  }
  const auto distortion_model = reader.getDistortion().getModel();

  // Populate data for ROS type
  ros_message.header.stamp = ros_time;
  ros_message.width = pinhole.dimensions[1];
  ros_message.height = pinhole.dimensions[0];
  ros_message.header.frame_id = get_frame_id();
  // Assign supported distortion model used.
  if (distortion_model == DistortionProto::DistortionModel::BROWN) {
    ros_message.distortion_model = "plumb_bob";
  } else if (distortion_model == DistortionProto::DistortionModel::FISHEYE) {
    ros_message.distortion_model = "rational_polynomial";
  } else {
    reportFailure("Unsupported distortion model.");
    return false;
  }
  // Intrinsic camera matrix for the raw (distorted) images.
  ros_message.K[0] = pinhole.focal[0];
  ros_message.K[1] = 0;
  ros_message.K[2] = pinhole.center[0];
  ros_message.K[3] = 0;
  ros_message.K[4] = pinhole.focal[1];
  ros_message.K[5] = pinhole.center[1];
  ros_message.K[6] = 0;
  ros_message.K[7] = 0;
  ros_message.K[8] = 1;

  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
