#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x9372099e79d17250;

using import "uuid.capnp".UuidProto;
using import "json.capnp".JsonProto;

# A message representing a mission to run by the application. A mission consists of a ISAAC
# configuration to apply to the application as well as a behavior tree to trigger. The application
# configuration is embedded in the MissionProto message and the behavior tree depends on which
# components receives the MissionProto.
struct MissionProto {
  # Uniquely identifies a mission across all systems
  uuid @0: UuidProto;
  # Json encoding an ISAAC configuration to apply before running the mission.
  config @1 : JsonProto;
}

# A message which identifies the status of a mission started by a MissionProto message.
# This message is sent out as a response to a MissionProto message to indicate that
# a mission has begun or to indicate that a mission has completed.
struct MissionStatusProto {
  # Uniquely identifies a mission across all systems. This should match the uuid of the
  # MissionProto message that started the mission.
  uuid @0: UuidProto;
  # An enum specifying the possible statuses of a mission
  enum MissionStatus {
    running @0;
    success @1;
    failure @2;
  }
  # The status of the mission
  missionStatus @1: MissionStatus;
}
