/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RosToDifferentialBaseCommand.hpp"


#include "messages/state/differential_base.hpp"
#include "packages/engine_gems/state/io.hpp"

namespace isaac {
namespace ros_bridge {

bool RosToDifferentialBaseCommand::rosToProto(const geometry_msgs::Twist::ConstPtr& ros_message,
                                              std::optional<ros::Time>& ros_time,
                                              alice::ProtoTx<StateProto>& tx_proto) {
  // Convert ROS twist to Isaac command
  messages::DifferentialBaseControl command;
  command.linear_speed() = ros_message->linear.x;
  command.angular_speed() = ros_message->angular.z;
  ToProto(command, tx_proto.initProto(), tx_proto.buffers());
  return true;
}

}  // namespace ros_bridge
}  // namespace isaac
