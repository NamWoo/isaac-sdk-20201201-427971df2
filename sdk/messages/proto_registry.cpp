/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "proto_registry.hpp"

#include <memory>

#include "capnp/compat/json.h"
#include "capnp/dynamic.h"
#include "capnp/message.h"
#include "capnp/schema.h"

#include "engine/core/assert.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace alice {

// General Entrance
std::optional<::capnp::DynamicStruct::Builder> GetRootBuilderByTypeId(
    const uint64_t type_id, ::capnp::MallocMessageBuilder& message_builder) {
  return Singleton<ProtoRegistry>::Get().getRootBuilderByTypeId(type_id, message_builder);
}

// General Entrance
std::optional<::capnp::DynamicStruct::Reader> GetRootReaderByTypeId(
    const uint64_t type_id, ::capnp::SegmentArrayMessageReader& message_reader) {
  return Singleton<ProtoRegistry>::Get().getRootReaderByTypeId(type_id, message_reader);
}

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_PROTO(ActorGroupProto)
ISAAC_ALICE_REGISTER_PROTO(ActorGroupStatusProto)
ISAAC_ALICE_REGISTER_PROTO(AudioDataProto)
ISAAC_ALICE_REGISTER_PROTO(AudioFilePlaybackProto)
ISAAC_ALICE_REGISTER_PROTO(BooleanProto)
ISAAC_ALICE_REGISTER_PROTO(CameraIntrinsicsProto)
ISAAC_ALICE_REGISTER_PROTO(ChatMessageProto)
ISAAC_ALICE_REGISTER_PROTO(CollisionProto)
ISAAC_ALICE_REGISTER_PROTO(ColorCameraProto)
ISAAC_ALICE_REGISTER_PROTO(CompositeProto)
ISAAC_ALICE_REGISTER_PROTO(ConfusionMatrixProto)
ISAAC_ALICE_REGISTER_PROTO(DepthCameraProto)
ISAAC_ALICE_REGISTER_PROTO(Detections2Proto)
ISAAC_ALICE_REGISTER_PROTO(Detections3Proto)
ISAAC_ALICE_REGISTER_PROTO(DifferentialTrajectoryPlanProto)
ISAAC_ALICE_REGISTER_PROTO(DisparityCameraProto)
ISAAC_ALICE_REGISTER_PROTO(DistortionProto)
ISAAC_ALICE_REGISTER_PROTO(DynamixelMotorsProto)
ISAAC_ALICE_REGISTER_PROTO(FiducialListProto)
ISAAC_ALICE_REGISTER_PROTO(FiducialProto)
ISAAC_ALICE_REGISTER_PROTO(FlatscanProto)
ISAAC_ALICE_REGISTER_PROTO(Goal2FeedbackProto)
ISAAC_ALICE_REGISTER_PROTO(Goal2Proto)
ISAAC_ALICE_REGISTER_PROTO(GoalWaypointProto)
ISAAC_ALICE_REGISTER_PROTO(GroundPlaneProto)
ISAAC_ALICE_REGISTER_PROTO(HeatmapProto)
ISAAC_ALICE_REGISTER_PROTO(ImageProto)
ISAAC_ALICE_REGISTER_PROTO(ImuProto)
ISAAC_ALICE_REGISTER_PROTO(JoystickStateProto)
ISAAC_ALICE_REGISTER_PROTO(JsonProto)
ISAAC_ALICE_REGISTER_PROTO(LabelProto)
ISAAC_ALICE_REGISTER_PROTO(LedStripProto)
ISAAC_ALICE_REGISTER_PROTO(LogMessageProto)
ISAAC_ALICE_REGISTER_PROTO(MarkerListProto)
ISAAC_ALICE_REGISTER_PROTO(MessageChannelIndexProto)
ISAAC_ALICE_REGISTER_PROTO(MessageHeaderProto)
ISAAC_ALICE_REGISTER_PROTO(MissionProto)
ISAAC_ALICE_REGISTER_PROTO(MissionStatusProto)
ISAAC_ALICE_REGISTER_PROTO(ObstaclesProto)
ISAAC_ALICE_REGISTER_PROTO(Odometry2Proto)
ISAAC_ALICE_REGISTER_PROTO(ParkingSpotProto)
ISAAC_ALICE_REGISTER_PROTO(ParkingSpotListProto)
ISAAC_ALICE_REGISTER_PROTO(PingProto)
ISAAC_ALICE_REGISTER_PROTO(PinholeProto)
ISAAC_ALICE_REGISTER_PROTO(Plan2Proto)
ISAAC_ALICE_REGISTER_PROTO(PlaneProto)
ISAAC_ALICE_REGISTER_PROTO(PointCloudProto)
ISAAC_ALICE_REGISTER_PROTO(Polyline2dProto)
ISAAC_ALICE_REGISTER_PROTO(Pose2dProto)
ISAAC_ALICE_REGISTER_PROTO(Pose2fProto)
ISAAC_ALICE_REGISTER_PROTO(Pose3dProto)
ISAAC_ALICE_REGISTER_PROTO(Pose3fProto)
ISAAC_ALICE_REGISTER_PROTO(PoseTreeEdgeProto)
ISAAC_ALICE_REGISTER_PROTO(PoseTreeProto)
ISAAC_ALICE_REGISTER_PROTO(PredictionProto)
ISAAC_ALICE_REGISTER_PROTO(PwmChannelSetDutyCycleProto)
ISAAC_ALICE_REGISTER_PROTO(PwmChannelSetPulseLengthProto)
ISAAC_ALICE_REGISTER_PROTO(QuaterniondProto)
ISAAC_ALICE_REGISTER_PROTO(QuaternionfProto)
ISAAC_ALICE_REGISTER_PROTO(RangeScanProto)
ISAAC_ALICE_REGISTER_PROTO(RectangleProto)
ISAAC_ALICE_REGISTER_PROTO(RigidBody3GroupProto)
ISAAC_ALICE_REGISTER_PROTO(RigidBody3Proto)
ISAAC_ALICE_REGISTER_PROTO(RobotStateProto)
ISAAC_ALICE_REGISTER_PROTO(SegmentationCameraProto)
ISAAC_ALICE_REGISTER_PROTO(SegmentationPredictionProto)
ISAAC_ALICE_REGISTER_PROTO(SO2dProto)
ISAAC_ALICE_REGISTER_PROTO(SO2fProto)
ISAAC_ALICE_REGISTER_PROTO(SO3dProto)
ISAAC_ALICE_REGISTER_PROTO(SO3fProto)
ISAAC_ALICE_REGISTER_PROTO(StateProto)
ISAAC_ALICE_REGISTER_PROTO(SuperpixelLabelsProto)
ISAAC_ALICE_REGISTER_PROTO(SuperpixelsProto)
ISAAC_ALICE_REGISTER_PROTO(TensorProto)
ISAAC_ALICE_REGISTER_PROTO(UdpPackageHeaderProto)
ISAAC_ALICE_REGISTER_PROTO(UuidProto)
ISAAC_ALICE_REGISTER_PROTO(Vector2dProto)
ISAAC_ALICE_REGISTER_PROTO(Vector2fProto)
ISAAC_ALICE_REGISTER_PROTO(Vector2iProto)
ISAAC_ALICE_REGISTER_PROTO(Vector3dProto)
ISAAC_ALICE_REGISTER_PROTO(Vector3fProto)
ISAAC_ALICE_REGISTER_PROTO(Vector3iProto)
ISAAC_ALICE_REGISTER_PROTO(Vector3ubProto)
ISAAC_ALICE_REGISTER_PROTO(Vector4dProto)
ISAAC_ALICE_REGISTER_PROTO(Vector4fProto)
ISAAC_ALICE_REGISTER_PROTO(Vector4iProto)
ISAAC_ALICE_REGISTER_PROTO(VectorXdProto)
ISAAC_ALICE_REGISTER_PROTO(VectorXfProto)
ISAAC_ALICE_REGISTER_PROTO(VectorXiProto)
ISAAC_ALICE_REGISTER_PROTO(VoiceCommandDetectionProto)
