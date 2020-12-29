/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "messages/actor_group.capnp.h"
#include "messages/alice.capnp.h"
#include "messages/audio_data.capnp.h"
#include "messages/audio_file_playback.capnp.h"
#include "messages/basic.capnp.h"
#include "messages/camera.capnp.h"
#include "messages/chat_message.capnp.h"
#include "messages/collision.capnp.h"
#include "messages/composite.capnp.h"
#include "messages/detections.capnp.h"
#include "messages/differential_base.capnp.h"
#include "messages/dynamixel_motors.capnp.h"
#include "messages/element_type.capnp.h"
#include "messages/fiducial_list.capnp.h"
#include "messages/flatscan.capnp.h"
#include "messages/geometry.capnp.h"
#include "messages/heatmap.capnp.h"
#include "messages/image.capnp.h"
#include "messages/imu.capnp.h"
#include "messages/joystick_state.capnp.h"
#include "messages/json.capnp.h"
#include "messages/label.capnp.h"
#include "messages/led_strip.capnp.h"
#include "messages/map.capnp.h"
#include "messages/marker_list.capnp.h"
#include "messages/math.capnp.h"
#include "messages/mission.capnp.h"
#include "messages/obstacles.capnp.h"
#include "messages/parking.capnp.h"
#include "messages/ping.capnp.h"
#include "messages/point_cloud.capnp.h"
#include "messages/pose_tree.capnp.h"
#include "messages/pwm_channel_set.capnp.h"
#include "messages/range_scan.capnp.h"
#include "messages/rigid_body_3_group.capnp.h"
#include "messages/robot_state.capnp.h"
#include "messages/segmentation_prediction.capnp.h"
#include "messages/state.capnp.h"
#include "messages/superpixels.capnp.h"
#include "messages/tensor.capnp.h"
#include "messages/trajectory.capnp.h"
#include "messages/uuid.capnp.h"
#include "messages/voice_command_detection.capnp.h"
