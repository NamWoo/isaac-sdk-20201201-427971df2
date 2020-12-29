/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ZedCamera.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"
#include "packages/zed/gems/time_offset_calculator.hpp"

namespace isaac {
namespace {

// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 4;

// Helper function to retrieve the camera extrinsics
Pose3d GetCameraExtrinsics(const sl::CalibrationParameters& calibration_params) {
  Pose3d out;
  const sl::Transform& stereo_transform = calibration_params.stereo_transform;
  const sl::Orientation orientation = stereo_transform.getOrientation();
  out.rotation = SO3<double>::FromQuaternion(
      Quaterniond(orientation.ow, orientation.ox, orientation.oy, orientation.oz));
  const sl::Translation translation = stereo_transform.getTranslation();
  out.translation = Vector3d(translation.x, translation.y, translation.z);
  return out;
}

// Helper function to copy camera intrinsics to CameraIntrinsicsProto
void SetCameraProtoIntrinsics(const sl::CameraParameters& in,
                              ::CameraIntrinsicsProto::Builder out) {
  // Pinhole camera model parameters
  auto pinhole = out.initPinhole();
  ToProto(Vector2d(in.fy, in.fx), pinhole.getFocal());
  ToProto(Vector2d(in.cy, in.cx), pinhole.getCenter());
  pinhole.setCols(in.image_size.width);
  pinhole.setRows(in.image_size.height);

  // Distortion parameters
  auto distortion = out.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  Vector5d disto;
  disto << in.disto[0], in.disto[1], in.disto[2], in.disto[3], in.disto[4];
  ToProto(disto, distortion.getCoefficients());
}

// Helper function to change the specified camera parameter if needed.
void SynchronizeCameraSetting(const int64_t userValue, int64_t& storedValue,
                              const sl::VIDEO_SETTINGS settingId, sl::Camera& zed) {
  if (userValue != storedValue) {
    zed.setCameraSettings(settingId, userValue);
    storedValue = userValue;
  }
}

// Gets a view on a ZED image
ImageConstView1ub GrayZedImageView(const sl::Mat& mat) {
  ASSERT(mat.getStepBytes() == mat.getWidth(), "Not yet supported");
  return CreateImageView<uint8_t, 1>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

// Gets a view on a ZED image
ImageConstView4ub BgraZedImageView(const sl::Mat& mat) {
  return CreateImageView<uint8_t, 4>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

}  // namespace

void ZedCamera::start() {
  if (!initializeZedCamera()) {
    // reportFailure() was already called in initializeZedCamera() with an error-specific message.
    return;
  }

  show("Zed Camera Model", sl::toString(zed_info_.camera_model).c_str());
  show("Zed Camera Serial Number", zed_info_.serial_number);
  show("Zed Camera Firmware Version", zed_configuration_.firmware_version);

  // publish the camera extrinsics into the pose tree
  const bool left_T_right_camera_set =
      node()->pose().trySet(get_lhs_camera_frame(), get_rhs_camera_frame(),
                            GetCameraExtrinsics(zed_configuration_.calibration_parameters), 0);
  if (!left_T_right_camera_set) {
    reportFailure("Failed to insert the left_T_right ZED camera transform into the Pose Tree.");
    return;
  }

  tickBlocking();

  sl::Camera& zed_camera{*zed_.get()};
  const std::function<int64_t()> zed_clock_f{
      [&zed_camera]() { return zed_camera.getTimestamp(sl::TIME_REFERENCE::CURRENT); }};

  const alice::Clock& isaac_clock{*(this->node()->clock())};
  const std::function<int64_t()> isaac_clock_f{
      [&isaac_clock]() { return isaac_clock.timestamp(); }};

  timestamp_offset_ = zed::TimeOffset(zed_clock_f, isaac_clock_f);
}

void ZedCamera::tick() {
  // A value of 0 disables the auto white balance, while 1 activates it.
  if (get_auto_white_balance()) {
    // Due to a ZED firmware quirk the color temperature must be explicitly set to default for
    // AWB to work.
    SynchronizeCameraSetting(sl::VIDEO_SETTINGS_VALUE_AUTO, color_temperature_,
                             sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, *zed_);
    SynchronizeCameraSetting(1, auto_white_balance_, sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, *zed_);
  } else {
    auto_white_balance_ = 0;
    // Setting a color temperature value automatically disables VIDEO_SETTINGS::WHITEBALANCE_AUTO
    SynchronizeCameraSetting(get_color_temperature(), color_temperature_,
                             sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, *zed_);
  }
  SynchronizeCameraSetting(get_brightness(), brightness_, sl::VIDEO_SETTINGS::BRIGHTNESS, *zed_);
  SynchronizeCameraSetting(get_contrast(), contrast_, sl::VIDEO_SETTINGS::CONTRAST, *zed_);
  if (get_auto_exposure()) {
    exposure_ = sl::VIDEO_SETTINGS_VALUE_AUTO;
    gain_ = sl::VIDEO_SETTINGS_VALUE_AUTO;
    SynchronizeCameraSetting(1, auto_exposure_, sl::VIDEO_SETTINGS::AEC_AGC, *zed_);
  } else {
    auto_exposure_ = 0;
    // Setting of the exposure or gain value disables automatic exposure control.
    SynchronizeCameraSetting(get_exposure(), exposure_, sl::VIDEO_SETTINGS::EXPOSURE, *zed_);
    SynchronizeCameraSetting(get_gain(), gain_, sl::VIDEO_SETTINGS::GAIN, *zed_);
  }
  SynchronizeCameraSetting(get_gamma(), gamma_, sl::VIDEO_SETTINGS::GAMMA, *zed_);
  SynchronizeCameraSetting(get_hue(), hue_, sl::VIDEO_SETTINGS::HUE, *zed_);
  SynchronizeCameraSetting(get_saturation(), saturation_, sl::VIDEO_SETTINGS::SATURATION, *zed_);
  SynchronizeCameraSetting(get_sharpness(), sharpness_, sl::VIDEO_SETTINGS::SHARPNESS, *zed_);

  // If no images are available yet, ERROR_CODE "ERROR_CODE_NOT_A_NEW_FRAME" will be returned.
  // This function is meant to be called frequently in the main loop of your application.
  sl::ERROR_CODE result = zed_->grab(zed_run_params_);
  if (sl::ERROR_CODE::SUCCESS == result) {
    const int64_t isaac_time = zedToIsaacTimestamp(zed_->getTimestamp(sl::TIME_REFERENCE::IMAGE));

    retriveImages();
    publishImageData(isaac_time);

    publishCameraData(isaac_time);
  } else {
    // ZED API doesn't guarantee 100% reliability of every call.
    // Skipping a few frames is not a critical problem for video stream consumers
    // like stereo visual odometry
    LOG_WARNING("[ZedCamera] Error capturing images: %s", sl::toString(result).c_str());
  }
}

void ZedCamera::stop() {
  if (zed_.get() != nullptr) {
    zed_->close();
    zed_.reset();
  }
}

bool ZedCamera::initializeZedCamera() {
  sl::InitParameters params = {};
  params.camera_resolution = static_cast<sl::RESOLUTION>(get_resolution());
  // ZED Camera FPS is
  // 1. not tied to a codelet tick rate as the camera has an independent on-board ISP
  // 2. much lower than an IMU poll rate that's equal to the ZedImuReader codelet tick frequency
  params.camera_fps = get_camera_fps();
  // params.camera_linux_id = get_device_id();
  params.coordinate_units = sl::UNIT::METER;
  params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
  params.depth_mode = sl::DEPTH_MODE::NONE;
  // Enable the ZED ISP Enhanced Contrast Technology, to improve image quality.
  params.enable_image_enhancement = true;
  // This should be set for ZedImuReader to read the Zed-M and ZED 2 IMU data
  params.sensors_required = !get_enable_imu();
  params.sdk_cuda_ctx = isaac::cuda::GetOrCreateCudaContext(get_gpu_id());
  params.sdk_gpu_id = get_gpu_id();
  params.sdk_verbose = true;
  params.optional_settings_path = sl::String(get_settings_folder_path().c_str());

  // Create the camera object and open the camera
  auto camera = std::make_unique<sl::Camera>();
  sl::ERROR_CODE err = camera->open(params);
  if (err == sl::ERROR_CODE::CALIBRATION_FILE_NOT_AVAILABLE ||
      err == sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
    camera->close();
    // Get the device serial number
    const std::vector<sl::DeviceProperties> device_list = camera->getDeviceList();
    const int selected_device_id = get_device_id();
    const auto selected_device = std::find_if(
        device_list.begin(), device_list.end(),
        [selected_device_id](const auto& device) { return device.id == selected_device_id; });

    ASSERT(selected_device != device_list.end(), "Invalid selected zed camera %d", selected_device);
    // The camera is not calibrated or we couldn't find the calibration file.
    // We cannot proceed without calibration therefore we fail this component.
    reportFailure(
        "Calibration file was not found for your zed camera.\n"
        " Please download the factory calibration or calibrate your camera using the zed"
        " calibration utility.\n"
        " The serial number of your zed camera is %d\n"
        " In order to download factory calibration:\n"
        " 1. call ./engine/engine/build/scripts/download_zed_calibration.sh -s %d\n"
        " 2. copy the downloaded file, SN%d.conf, to the target device and specify the path to the"
        " containing folder using settings_folder_path setting\n",
        selected_device->serial_number, selected_device->serial_number,
        selected_device->serial_number);
    return false;
  } else if (err != sl::ERROR_CODE::SUCCESS) {
    zed_.reset();
    // The camera is unresponsive. We cannot proceed without the camera
    // therefore we fail this component.
    reportFailure("[ZedCamera] Error initializing Zed camera: %s", sl::toString(err).c_str());
    return false;
  }
  // Retrieve the camera calibration parameters from the ZED firmware
  // Zed Camera recalibrates itself for a selected resolution at sl::Camera::open
  zed_info_ = camera->getCameraInformation();
  zed_configuration_ = zed_info_.camera_configuration;
  zed_resolution_ = zed_configuration_.resolution;

  zed_run_params_.enable_depth = false;

  zed_.swap(camera);
  return true;
}

void ZedCamera::retriveImages() {
  const bool gray_scale = get_gray_scale();
  const bool rgb = get_rgb();
  const bool enable_factory_rectification = get_enable_factory_rectification();

  // Retrieve left and right images
  if (gray_scale) {
    const sl::VIEW left_view =
        (enable_factory_rectification) ? sl::VIEW::LEFT_GRAY : sl::VIEW::LEFT_UNRECTIFIED_GRAY;
    zed_->retrieveImage(left_image_gray_, left_view);

    const sl::VIEW right_view =
        (enable_factory_rectification) ? sl::VIEW::RIGHT_GRAY : sl::VIEW::RIGHT_UNRECTIFIED_GRAY;
    zed_->retrieveImage(right_image_gray_, right_view);
  }
  if (rgb) {
    const sl::VIEW left_view =
        (enable_factory_rectification) ? sl::VIEW::LEFT : sl::VIEW::LEFT_UNRECTIFIED;
    zed_->retrieveImage(left_image_rgb_, left_view);

    const sl::VIEW right_view =
        (enable_factory_rectification) ? sl::VIEW::RIGHT : sl::VIEW::RIGHT_UNRECTIFIED;
    zed_->retrieveImage(right_image_rgb_, right_view);
  }
}

void ZedCamera::publishImageData(const int64_t acq_time) {
  const bool gray_scale = get_gray_scale();
  const bool rgb = get_rgb();

  if (gray_scale) {
    publishGrayData(acq_time);
  }
  if (rgb) {
    publishRgbData(acq_time);
  }
}

void ZedCamera::publishCameraData(const int64_t acq_time) {
  // Retrieve camera parameters
  // 'raw' specifies if we want the parameters for unrectified images (true)
  // or rectified images (false)
  const bool raw = !get_enable_factory_rectification();
  const auto camera_parameters = raw ? zed_configuration_.calibration_parameters_raw
                                     : zed_configuration_.calibration_parameters;

  // Publish camera extrinsic parameters
  const auto ext = tx_extrinsics().initProto();
  ToProto(GetCameraExtrinsics(camera_parameters), ext);
  tx_extrinsics().publish(acq_time);

  // Publish camera intrinsics
  SetCameraProtoIntrinsics(camera_parameters.left_cam, tx_left_intrinsics().initProto());
  SetCameraProtoIntrinsics(camera_parameters.right_cam, tx_right_intrinsics().initProto());
  tx_left_intrinsics().publish(acq_time);
  tx_right_intrinsics().publish(acq_time);
}

void ZedCamera::publishGrayData(int64_t acq_time) {
  Image1ub buffer_left_gray(zed_resolution_.height, zed_resolution_.width);
  Copy(GrayZedImageView(left_image_gray_), buffer_left_gray);
  show("left_gray_thumbnail",
       [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray)); });
  ToProto(std::move(buffer_left_gray), tx_left_camera_gray().initProto(),
          tx_left_camera_gray().buffers());

  Image1ub buffer_right_gray(zed_resolution_.height, zed_resolution_.width);
  Copy(GrayZedImageView(right_image_gray_), buffer_right_gray);
  show("right_gray_thumbnail",
       [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray)); });
  ToProto(std::move(buffer_right_gray), tx_right_camera_gray().initProto(),
          tx_right_camera_gray().buffers());

  tx_left_camera_gray().publish(acq_time);
  tx_right_camera_gray().publish(acq_time);
}

void ZedCamera::publishRgbData(int64_t acq_time) {
  Image3ub buffer_left_rgb(zed_resolution_.height, zed_resolution_.width);
  ConvertBgraToRgb(BgraZedImageView(left_image_rgb_), buffer_left_rgb);
  show("left_rgb_thumbnail",
       [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_rgb)); });
  show("left_rgb", [&](sight::Sop& sop) { sop.add(buffer_left_rgb); });
  ToProto(std::move(buffer_left_rgb), tx_left_camera_rgb().initProto(),
          tx_left_camera_rgb().buffers());

  Image3ub buffer_right_rgb(zed_resolution_.height, zed_resolution_.width);
  ConvertBgraToRgb(BgraZedImageView(right_image_rgb_), buffer_right_rgb);
  show("right_rgb_thumbnail",
       [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_rgb)); });
  ToProto(std::move(buffer_right_rgb), tx_right_camera_rgb().initProto(),
          tx_right_camera_rgb().buffers());

  tx_left_camera_rgb().publish(acq_time);
  tx_right_camera_rgb().publish(acq_time);
}

}  // namespace isaac
