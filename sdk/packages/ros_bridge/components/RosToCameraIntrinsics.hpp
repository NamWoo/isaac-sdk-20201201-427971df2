/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "messages/camera.hpp"
#include "packages/ros_bridge/components/RosToProtoConverter.hpp"
#include "sensor_msgs/CameraInfo.h"

namespace isaac {
namespace ros_bridge {

// ROS's sensor_msgs/CameraInfo.msg message defines meta information for a camera.
// Details:http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html. This ROS bridge
// converter codelet helps convert ROS's sensor_msgs/CameraInfo.msg into Isaac's PinholeProto Type.
// This codelet can be used, for example, to use Isaac's GPU accelerated perception algorithms
// on ROS images.
class RosToCameraIntrinsics : public RosToProtoConverter<CameraIntrinsicsProto,
                                     sensor_msgs::CameraInfo> {
 public:
  bool rosToProto(const sensor_msgs::CameraInfo::ConstPtr& ros_message,
                  std::optional<ros::Time>& ros_time,
                  alice::ProtoTx<CameraIntrinsicsProto>& tx_proto) override;
};

}  // namespace ros_bridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ros_bridge::RosToCameraIntrinsics);
