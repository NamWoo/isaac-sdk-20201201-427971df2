/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/skeleton_pose_estimation/SkeletonsPnP.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/gems/geometry/pinhole.hpp"
#include "messages/camera.hpp"
#include "messages/geometry.hpp"
#include "messages/math.hpp"
#include "packages/pnp/gems/pnp.hpp"

namespace isaac {
namespace skeleton_pose_estimation {

namespace {
// A helper that converts a vector of points in N-D space into Eigen Matrix
// This allows to match isaac::pnp::ComputeCameraPoseEpnp format:
//  points3        Input 3D points as 3xN matrix (N>=6)
//  points2        Input 2D points as 2xN matrix (in corresponding order to 3D points)
template<typename K, int N>
Matrix<K, N, Eigen::Dynamic> PointListToMatrix(const std::vector<Vector<K, N>>& points) {
  Matrix<K, N, Eigen::Dynamic> matrix(N, points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    matrix.col(i) = points[i];
  }
  return matrix;
}

// A helper that initializes a number of elements in the CapNProto list from a source list.
// NOTE: the source qualifier is not const, as this is not allowed by CapNProto.
template <typename T>
void CopyList(T source, T destination, const size_t elements_number) {
  for (size_t i = 0; i < elements_number; ++i) {
    destination[i] = source[i];
  }
}

}  // namespace

void SkeletonsPnP::start() {
  tickOnMessage(rx_skeletons());
  synchronize(rx_skeletons(), rx_intrinsics());
}

void SkeletonsPnP::tick() {
  // Read in the pinhole, skeletons_of_interest, keypoint priors
  const geometry::PinholeD& pinhole = FromProto(rx_intrinsics().getProto().getPinhole());
  const int64_t acqtime = rx_skeletons().acqtime();
  const auto& skeletons = rx_skeletons().getProto().getSkeletons();
  const std::string& skeletons_of_interest = get_skeletons_of_interest();

  ::capnp::MallocMessageBuilder message;
  auto skeletons_reprojected = message.initRoot<::capnp::List<Skeleton2Proto> >(skeletons.size());
  auto poses = message.initRoot<::capnp::List<Pose3dProto> >(skeletons.size());
  auto predictions = message.initRoot<::capnp::List<PredictionProto> >(skeletons.size());
  size_t skeleton_number = 0;

  // Reads the lists of skeletons detections
  for (Skeleton2Proto::Reader skeleton : skeletons) {
    std::vector<Vector2d> keypoints2;         // NOTE: to optimize, could be declared outside of
    std::vector<Vector3d> keypoints3;         //       the for() loop and cleared at each iteration
    std::vector<Pose3d> frame_T_skeleton_joints;
    const std::string& skeleton_label = skeleton.getLabel().getLabel();

    if (skeleton_label.empty() || skeletons_of_interest.find(skeleton_label) == std::string::npos) {
      LOG_WARNING("Received a skeleton %s that doesn't match any skeletons of interest, "
                  "skipping refinement.", skeleton_label.c_str());
      continue;
    }
    auto joints = skeleton.getJoints();
    if (joints.size() < static_cast<size_t>(get_minimum_joints_threshold())) {
      LOG_INFO("Received a skeleton with %zu joints, skipping pose estimation.", joints.size());
      continue;
    }

    for (const auto& joint : joints) {
      const std::string& joint_label = joint.getLabel().getLabel();
      const std::string id = skeleton_label + '.' + joint_label;
      const auto& frame_T_skeleton_joint = this->node()->pose().tryGet(
                                              skeleton_label, joint_label, ToSeconds(acqtime));
      if (!frame_T_skeleton_joint) {
        LOG_WARNING("Detected skeleton for which %s_T_%s transformation is unknown.",
                    skeleton_label.c_str(), joint_label.c_str());
        continue;
      }

      frame_T_skeleton_joints.push_back(*frame_T_skeleton_joint);

      // As per ComputeCameraPoseEpnp documentation, these are: "3D points
      // in the world frame". So, using "keypoint_T_frame.translation" here.
      // Note: frame_T_skeleton_joint.translation = frame_T_skeleton_joint * Vector3d::Zero()
      keypoints3.push_back(frame_T_skeleton_joint->translation);

      // axes of ToProto first parameters are [down, right]
      // As per ComputeCameraPoseEpnp documentation, these are: "Input 2D points"
      // From samples, the order is: "U V  ..."
      keypoints2.push_back(FromProto(joint.getPosition()));
    }

    // Computing the camera pose using the PnP algorithm from 3D points and corresponding
    // 2D projections. Respective columns of points2 and points3 are 2D-3D matches
    // under perspective projection.
    const Matrix3Xd& points3 = PointListToMatrix(keypoints3);
    const Matrix2Xd& points2 = PointListToMatrix(keypoints2);
    const double focal_u = pinhole.focal[1];       // fx, i.e. 700
    const double focal_v = pinhole.focal[0];       // fy, i.e. 700
    const double principal_u = pinhole.center[1];  // cx, i.e. 640 for a 1280x720 camera
    const double principal_v = pinhole.center[0];  // cy, i.e. 360 for a 1280x720 camera

    Pose3d camera_T_frame;    // maps 3D points from the world frame into the camera frame.
    if (pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                   principal_v, &camera_T_frame) == pnp::Status::kSuccess) {
      const double reprojection_error_threshold = get_reprojection_error_threshold();
      double max_reprojection_error = 0.0;

      // Populate joints, calculate reprojection error. But, can still discard this skeleton.
      Skeleton2Proto::Builder skeleton_reprojected = skeletons_reprojected[skeleton_number];
      skeleton_reprojected.setJoints(skeleton.getJoints());

      for (size_t i = 0; i < joints.size(); ++i) {
        const std::string&  joint_label = joints[i].getLabel().getLabel();
        const std::string id = skeleton_label + '.' + joint_label;

        // Populate 3D Pose - orientation and translation
        const Pose3d& frame_T_skeleton_joint = frame_T_skeleton_joints[i];
        const Pose3d camera_T_skeleton_joint = camera_T_frame * frame_T_skeleton_joint;

        // Reproject keypoints back, calculate reprojection error
        // Note: camera_T_skeleton_joint.translation = camera_T_skeleton_joint * Vector3d::Zero()
        const Vector3d& camera_coord = camera_T_skeleton_joint.translation;
        const Vector2d joint_reprojected = pinhole.project(camera_coord);
        const Vector2d joint = FromProto(joints[i].getPosition());
        double reprojection_error = (Vector2d {joint[1], joint[0]} - joint_reprojected).norm();
        max_reprojection_error = std::max(reprojection_error, max_reprojection_error);
        if (reprojection_error > reprojection_error_threshold) {
          LOG_INFO("Skeleton joint %s: %.4f, reprojection error is above the threshold. ",
                   "No pose estimation will be available.", id.c_str(), reprojection_error);
          break;
        }

        Vector2dProto::Builder position = skeleton_reprojected.getJoints()[i].getPosition();
        ToProto(Vector2d{joint_reprojected[1], joint_reprojected[0]}, position);

        PredictionProto::Builder prediction = skeleton_reprojected.getJoints()[i].getLabel();
        prediction.setConfidence((reprojection_error_threshold - reprojection_error) /
                                 reprojection_error_threshold);
      }

      // Populate the rest of the output, if the reprojection error less than threshold
      if (max_reprojection_error < reprojection_error_threshold) {
        skeleton_reprojected.setGraph(skeleton.getGraph());
        skeleton_reprojected.setLabel(skeleton.getLabel());

        const double confidence = (reprojection_error_threshold - max_reprojection_error) /
                                   reprojection_error_threshold;
        predictions[skeleton_number].setLabel(skeleton.getLabel().getLabel());
        predictions[skeleton_number].setConfidence(confidence);

        ToProto(camera_T_frame, poses[skeleton_number]);

        skeleton_number++;
      }
    }
  }

  // Prepare TX to publish and populate skeletons and detections (using reference frame pose).
  auto skeletons_proto = tx_reprojected_skeletons().initProto();
  auto detections = tx_detections().initProto();
  CopyList(skeletons_reprojected, skeletons_proto.initSkeletons(skeleton_number), skeleton_number);
  CopyList(poses, detections.initPoses(skeleton_number), skeleton_number);
  CopyList(predictions, detections.initPredictions(skeleton_number), skeleton_number);

  tx_detections().publish(acqtime);
  tx_reprojected_skeletons().publish(acqtime);
}

}  // namespace skeleton_pose_estimation
}  // namespace isaac
