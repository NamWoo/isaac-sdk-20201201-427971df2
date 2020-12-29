/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ZedImuReader.hpp"

#include <string>

#include "messages/math.hpp"
#include "packages/zed/ZedCamera.hpp"

namespace isaac {

namespace zed {

Pose3f GetCameraFromImuTransform(const sl::Transform& imu_T_camera) {
  const sl::Orientation orientation = imu_T_camera.getOrientation();
  const sl::Translation translation = imu_T_camera.getTranslation();
  // ZED API uses floats
  const auto rotation = Quaternionf(orientation.w, orientation.x, orientation.y, orientation.z);
  return {SO3f::FromQuaternion(rotation), Vector3f(translation.x, translation.y, translation.z)};
}

void ZedImuReader::start() {
  camera_holder_ = node()->getComponentOrNull<ZedCamera>();
  if (camera_holder_ == nullptr) {
    reportFailure("[ZedImuReader] Can't obtain the Zed camera component.");
    return;
  }
  if (!camera_holder_->get_enable_imu()) {
    LOG_WARNING("[ZedImuReader] Zed IMU data capture is disabled.");
    return;
  }
  left_camera_T_imu_initialized_ = false;
  use_vendor_imu_calibration_ = get_vendor_imu_calibration();
  last_imu_timestamp_ = 0;
  // The ZED camera ISP obtains the IMU readings at a rate independent of the codelet frequency.
  tickPeriodically();
}

void ZedImuReader::stop() {
  camera_holder_ = nullptr;
  last_imu_timestamp_ = 0;
}

void ZedImuReader::tick() {
  auto camera = camera_holder_->getZedCamera();
  // ZED camera initialization process is performed in parallel to the ZedImuReader execution
  if (camera == nullptr) {
    return;
  }
  // We can't obtain camera information at ZedImuReader::start.
  const auto camera_information = camera_holder_->getCameraInformation();
  sl::SensorsData sensors_data = {};
  const sl::ERROR_CODE result = camera->getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);
  if (sl::ERROR_CODE::SUCCESS == result) {
    // The vendor API call might return the same IMU measurement several times in row.
    // The timestamp comparison is used to avoid publishing duplicate IMU data.
    const sl::SensorsData::IMUData imu_data = sensors_data.imu;
    if (imu_data.timestamp > last_imu_timestamp_) {
      last_imu_timestamp_ = imu_data.timestamp;
      publishImuData(sensors_data.imu);
    }
  } else if (sl::ERROR_CODE::SENSORS_NOT_AVAILABLE == result) {
      reportFailure("This ZED camera model doesn't have the built-in IMU.");
      return;
  } else {
    // ZED API doesn't guarantee 100% reliability of every call.
    // Skipping a few IMU readings is not a critical problem for consumers
    // like visual inertial odometry
    LOG_WARNING("[ZedImuReader] Error capturing IMU data: %s", sl::toString(result).c_str());
  }

  // This can't be done at start() as the ZED camera is accessed and initialized concurrently to
  // to the start-up of this codelet
  if (!left_camera_T_imu_initialized_) {
    const auto camera_imu_transform = camera_information.sensors_configuration.camera_imu_transform;
    const Pose3d left_camera_T_imu =
        GetCameraFromImuTransform(camera_imu_transform).cast<double>();

    const std::string left_frame = camera_holder_->get_lhs_camera_frame();
    left_camera_T_imu_initialized_ =
        node()->pose().trySet(left_frame, get_imu_frame(), left_camera_T_imu, 0);
  }
}

void ZedImuReader::publishImuData(const sl::SensorsData::IMUData& imu_data) {
  auto imu_datamsg = tx_imu_raw().initProto();
  // set accelerometer data
  imu_datamsg.setLinearAccelerationX(use_vendor_imu_calibration_
                                         ? imu_data.linear_acceleration.x
                                         : imu_data.linear_acceleration_uncalibrated.x);
  imu_datamsg.setLinearAccelerationY(use_vendor_imu_calibration_
                                         ? imu_data.linear_acceleration.y
                                         : imu_data.linear_acceleration_uncalibrated.y);
  imu_datamsg.setLinearAccelerationZ(use_vendor_imu_calibration_
                                         ? imu_data.linear_acceleration.z
                                         : imu_data.linear_acceleration_uncalibrated.z);

  // set gyroscope data
  imu_datamsg.setAngularVelocityX(use_vendor_imu_calibration_
                                      ? DegToRad(imu_data.angular_velocity.x)
                                      : DegToRad(imu_data.angular_velocity_uncalibrated.x));
  imu_datamsg.setAngularVelocityY(use_vendor_imu_calibration_
                                      ? DegToRad(imu_data.angular_velocity.y)
                                      : DegToRad(imu_data.angular_velocity_uncalibrated.y));
  imu_datamsg.setAngularVelocityZ(use_vendor_imu_calibration_
                                      ? DegToRad(imu_data.angular_velocity.z)
                                      : DegToRad(imu_data.angular_velocity_uncalibrated.z));

  const int64_t isaac_time = camera_holder_->zedToIsaacTimestamp(imu_data.timestamp);
  tx_imu_raw().publish(isaac_time);

  // Show IMU data in Sight
  show("LinearAcceleration.x", imu_data.linear_acceleration.x);
  show("LinearAcceleration.y", imu_data.linear_acceleration.y);
  show("LinearAcceleration.z", imu_data.linear_acceleration.z);
  show("AngularVelocity.x", imu_data.angular_velocity.x);
  show("AngularVelocity.y", imu_data.angular_velocity.y);
  show("AngularVelocity.z", imu_data.angular_velocity.z);
}

}  // namespace zed
}  // namespace isaac
