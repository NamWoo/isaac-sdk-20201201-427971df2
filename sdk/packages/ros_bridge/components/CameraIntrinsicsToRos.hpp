/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "messages/camera.capnp.h"
#include "packages/ros_bridge/components/ProtoToRosConverter.hpp"
#include "sensor_msgs/CameraInfo.h"

namespace isaac {
namespace ros_bridge {

// This codelet receives CameraIntrinsicsProto data within Isaac application and publishes it to
// ROS.
class CameraIntrinsicsToRos : public ProtoToRosConverter<CameraIntrinsicsProto,
                                     sensor_msgs::CameraInfo> {
 public:
  bool protoToRos(const alice::ProtoRx<CameraIntrinsicsProto>& rx_proto, const ros::Time& ros_time,
                  sensor_msgs::CameraInfo& ros_message) override;

  // This param will populate frame_id in ROS CameraInfo message.
  // Details at http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  ISAAC_PARAM(std::string, frame_id, "camera");
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::CameraIntrinsicsToRos);
