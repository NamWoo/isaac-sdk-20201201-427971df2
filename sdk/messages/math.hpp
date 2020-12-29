/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "messages/math.capnp.h"

namespace isaac {

inline Quaternionf FromProto(::QuaternionfProto::Reader reader) {
  return {reader.getW(), reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Quaternionf q, ::QuaternionfProto::Builder builder) {
  builder.setX(q.x());
  builder.setY(q.y());
  builder.setZ(q.z());
  builder.setW(q.w());
}

inline Quaterniond FromProto(::QuaterniondProto::Reader reader) {
  return {reader.getW(), reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Quaterniond& q, ::QuaterniondProto::Builder builder) {
  builder.setX(q.x());
  builder.setY(q.y());
  builder.setZ(q.z());
  builder.setW(q.w());
}

inline Vector2f FromProto(::Vector2fProto::Reader reader) {
  return {reader.getX(), reader.getY()};
}

inline void ToProto(const Vector2f& v, ::Vector2fProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
}

inline Vector3f FromProto(::Vector3fProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Vector3f& v, ::Vector3fProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
}

inline Vector4f FromProto(::Vector4fProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ(), reader.getW()};
}

inline void ToProto(const Vector4f& v, ::Vector4fProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
  builder.setW(v.w());
}

inline VectorXf FromProto(::VectorXfProto::Reader reader) {
  auto Coefficients = reader.getCoefficients();
  VectorXf return_value(Coefficients.size());
  for (size_t i = 0; i < Coefficients.size(); i++) {
    return_value[i] = Coefficients[i];
  }
  return return_value;
}

inline void ToProto(const VectorXf& v, ::VectorXfProto::Builder builder) {
  auto Coefficients = builder.initCoefficients(v.rows());
  for (int i = 0; i < v.rows(); i++) {
    Coefficients.set(i, v[i]);
  }
}

inline Vector2d FromProto(::Vector2dProto::Reader reader) {
  return {reader.getX(), reader.getY()};
}

inline void ToProto(const Vector2d& v, ::Vector2dProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
}

inline Vector3d FromProto(::Vector3dProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Vector3d& v, ::Vector3dProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
}

inline Vector4d FromProto(::Vector4dProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ(), reader.getW()};
}

inline void ToProto(const Vector4d& v, ::Vector4dProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
  builder.setW(v.w());
}

inline VectorXd FromProto(::VectorXdProto::Reader reader) {
  auto Coefficients = reader.getCoefficients();
  VectorXd return_value(Coefficients.size());
  for (size_t i = 0; i < Coefficients.size(); i++) {
    return_value[i] = Coefficients[i];
  }
  return return_value;
}

inline void ToProto(const VectorXd& v, ::VectorXdProto::Builder builder) {
  auto Coefficients = builder.initCoefficients(v.rows());
  for (int i = 0; i < v.rows(); i++) {
    Coefficients.set(i, v[i]);
  }
}

inline Vector2i FromProto(::Vector2iProto::Reader reader) {
  return {reader.getX(), reader.getY()};
}

inline void ToProto(const Vector2i& v, ::Vector2iProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
}

inline Vector3i FromProto(::Vector3iProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Vector3i& v, ::Vector3iProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
}

inline Vector4i FromProto(::Vector4iProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ(), reader.getW()};
}

inline void ToProto(const Vector4i& v, ::Vector4iProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
  builder.setW(v.w());
}

inline VectorXi FromProto(::VectorXiProto::Reader reader) {
  auto Coefficients = reader.getCoefficients();
  VectorXi return_value(Coefficients.size());
  for (size_t i = 0; i < Coefficients.size(); i++) {
    return_value[i] = Coefficients[i];
  }
  return return_value;
}

inline void ToProto(const VectorXi& v, ::VectorXiProto::Builder builder) {
  auto Coefficients = builder.initCoefficients(v.rows());
  for (int i = 0; i < v.rows(); i++) {
    Coefficients.set(i, v[i]);
  }
}

inline Vector3ub FromProto(::Vector3ubProto::Reader reader) {
  return {reader.getX(), reader.getY(), reader.getZ()};
}

inline void ToProto(const Matrix3d& v, ::Matrix3dProto::Builder builder) {
  ToProto(v.row(0), builder.initRow1());
  ToProto(v.row(1), builder.initRow2());
  ToProto(v.row(2), builder.initRow3());
}

inline Matrix3d FromProto(::Matrix3dProto::Reader reader) {
  Matrix3d result;
  result.row(0) = FromProto(reader.getRow1());
  result.row(1) = FromProto(reader.getRow2());
  result.row(2) = FromProto(reader.getRow3());
  return result;
}

inline void ToProto(const Vector3ub& v, ::Vector3ubProto::Builder builder) {
  builder.setX(v.x());
  builder.setY(v.y());
  builder.setZ(v.z());
}

inline SO2f FromProto(::SO2fProto::Reader reader) {
  return SO2f::FromDirection(FromProto(reader.getQ()));
}

inline void ToProto(const SO2f& rot, ::SO2fProto::Builder builder) {
  ToProto(rot.asDirection(), builder.initQ());
}

inline SO2d FromProto(::SO2dProto::Reader reader) {
  return SO2d::FromDirection(FromProto(reader.getQ()));
}

inline void ToProto(const SO2d& rot, ::SO2dProto::Builder builder) {
  ToProto(rot.asDirection(), builder.initQ());
}

inline SO3f FromProto(::SO3fProto::Reader reader) {
  return SO3f::FromQuaternion(FromProto(reader.getQ()));
}

inline void ToProto(const SO3f& rot, ::SO3fProto::Builder builder) {
  ToProto(rot.quaternion(), builder.initQ());
}

inline SO3d FromProto(::SO3dProto::Reader reader) {
  return SO3d::FromQuaternion(FromProto(reader.getQ()));
}

inline void ToProto(const SO3d& rot, ::SO3dProto::Builder builder) {
  ToProto(rot.quaternion(), builder.initQ());
}

inline Pose2f FromProto(::Pose2fProto::Reader reader) {
  Pose2f pose;
  pose.rotation = FromProto(reader.getRotation());
  pose.translation = FromProto(reader.getTranslation());
  return pose;
}

inline void ToProto(const Pose2f& pose, ::Pose2fProto::Builder builder) {
  ToProto(pose.rotation, builder.initRotation());
  ToProto(pose.translation, builder.initTranslation());
}

inline Pose2d FromProto(::Pose2dProto::Reader reader) {
  Pose2d pose;
  pose.rotation = FromProto(reader.getRotation());
  pose.translation = FromProto(reader.getTranslation());
  return pose;
}

inline void ToProto(const Pose2d& pose, ::Pose2dProto::Builder builder) {
  ToProto(pose.rotation, builder.initRotation());
  ToProto(pose.translation, builder.initTranslation());
}

inline Pose3f FromProto(::Pose3fProto::Reader reader) {
  Pose3f pose;
  pose.rotation = FromProto(reader.getRotation());
  pose.translation = FromProto(reader.getTranslation());
  return pose;
}

inline void ToProto(const Pose3f& pose, ::Pose3fProto::Builder builder) {
  ToProto(pose.rotation, builder.initRotation());
  ToProto(pose.translation, builder.initTranslation());
}

inline Pose3d FromProto(::Pose3dProto::Reader reader) {
  Pose3d pose;
  pose.rotation = FromProto(reader.getRotation());
  pose.translation = FromProto(reader.getTranslation());
  return pose;
}

inline void ToProto(const Pose3d& pose, ::Pose3dProto::Builder builder) {
  ToProto(pose.rotation, builder.initRotation());
  ToProto(pose.translation, builder.initTranslation());
}

}  // namespace isaac
