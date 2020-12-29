/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "gtest/gtest.h"
#include "packages/engine_c_api/isaac_c_api.h"

#include "engine/core/logger.hpp"

TEST(CApiApplication, Basics) {
  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/test.app.json";
  const char* module_path[1] = {(char*)""};
  isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);

  EXPECT_NE(0, app);
  auto error1 = isaac_start_application(app);
  EXPECT_EQ(error1, isaac_error_success);

  auto error2 = isaac_stop_application(app);
  EXPECT_EQ(error2, isaac_error_success);

  isaac_destroy_application(&app);
  EXPECT_EQ(0, app);

  int64_t time;
  auto error3 = isaac_get_time(app, &time);
  EXPECT_EQ(isaac_error_invalid_handle, error3);
}

TEST(CApiApplication, CreateDestroyMessage) {
  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/test.app.json";
  const char* module_path[1] = {(char*)""};
  isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);

  isaac_uuid_t message;
  auto error1 = isaac_create_message(app, &message);
  EXPECT_EQ(error1, isaac_error_success);
  EXPECT_NE(0, message.lower);
  EXPECT_NE(0, message.upper);

  auto error2 = isaac_destroy_message(app, &message);
  EXPECT_EQ(error2, isaac_error_success);

  isaac_destroy_application(&app);
  EXPECT_EQ(0, app);
}

TEST(CApiApplication, GetSetParam) {
  isaac_error_t error;

  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/test.app.json";
  const char* module_path[1] = {(char*)""};
  isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);
  EXPECT_NE(0, app);

  error = isaac_start_application(app);
  EXPECT_EQ(error, isaac_error_success);

  sleep(1);

  // Test basic get functionality.
  {
    isaac_json_t json = isaac_create_null_json();
    error = isaac_get_parameter(app, "flatscan_generator", "generator", "beam_count", &json);
    EXPECT_EQ(isaac_error_data_not_read, error);
    json.data = static_cast<char*>(std::malloc(json.size));
    error = isaac_get_parameter(app, "flatscan_generator", "generator", "beam_count", &json);
    EXPECT_EQ(isaac_error_success, error);
    EXPECT_STREQ(json.data, "1000");
    std::free(json.data);
  }

  // Test basic set functionality.
  {
    error = isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "beam_count",
                                            "10000");
    EXPECT_EQ(isaac_error_success, error);
  }
  {
    isaac_json_t json;
    json.size = 6;
    json.data = static_cast<char*>(std::malloc(json.size));
    error = isaac_get_parameter(app, "flatscan_generator", "generator", "beam_count", &json);
    EXPECT_EQ(isaac_error_success, error);
    EXPECT_STREQ(json.data, "10000");
    std::free(json.data);
  }

  std::vector<char> buffer(100);
  isaac_json_t json;
  json.data = buffer.data();

  // Test each param type
  // String
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "tick_period", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "\"1000Hz\"");
  error = isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "tick_period",
                                          "\"100Hz\"");
  ASSERT_EQ(error, isaac_error_success);
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "tick_period", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "\"100Hz\"");

  // Boolean
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy1", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "true");
  error =
      isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "dummy1", "false");
  ASSERT_EQ(error, isaac_error_success);
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy1", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "false");

  // Number
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy2", &json);
  EXPECT_STREQ(json.data, "1.111");
  error =
      isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "dummy2", "3.14159");
  ASSERT_EQ(error, isaac_error_success);
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy2", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "3.14159");

  // Array
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy3", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "[1,2,3]");
  error = isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "dummy3",
                                          "[4,\"hi\",7]");
  ASSERT_EQ(error, isaac_error_success);
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy3", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "[4,\"hi\",7]");

  // Object
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy4", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "{\"test1\":100,\"test2\":\"hi\"}");
  error = isaac_set_parameter_from_string(app, "flatscan_generator", "generator", "dummy4",
                                          "{\"test4\":1.11,\"test5\":6,\"test6\":\"by\"}");
  ASSERT_EQ(error, isaac_error_success);
  json.size = buffer.size();
  error = isaac_get_parameter(app, "flatscan_generator", "generator", "dummy4", &json);
  ASSERT_EQ(error, isaac_error_success);
  EXPECT_STREQ(json.data, "{\"test4\":1.11,\"test5\":6,\"test6\":\"by\"}");

  sleep(1);

  error = isaac_stop_application(app);
  EXPECT_EQ(error, isaac_error_success);

  isaac_destroy_application(&app);
  EXPECT_EQ(0, app);
}

TEST(CApiApplication, GetSetPoseAndTime) {
  isaac_error_t error;

  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/test.app.json";
  const char* module_path[1] = {(char*)""};
  isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);

  EXPECT_NE(0, app);
  error = isaac_start_application(app);
  EXPECT_EQ(error, isaac_error_success);

  isaac_pose_t pose;
  pose.px = 1;
  pose.py = 2;
  pose.pz = 3;
  pose.qw = .707107;
  pose.qx = .707107;
  pose.qy = 0;
  pose.qz = 0;

  int64_t time1 = 0;
  isaac_get_time(app, &time1);
  isaac_set_pose(app, "test1", "test2", time1, pose);

  // Check time is monotonic

  sleep(1);
  int64_t time2 = 0;
  isaac_get_time(app, &time2);
  isaac_pose_t out_pose;
  isaac_get_pose(app, "test1", "test2", time2, &out_pose);

  EXPECT_GT(time2 - time1, 100000000);

  EXPECT_FLOAT_EQ(pose.px, out_pose.px);
  EXPECT_FLOAT_EQ(pose.py, out_pose.py);
  EXPECT_FLOAT_EQ(pose.pz, out_pose.pz);
  EXPECT_FLOAT_EQ(pose.qw, out_pose.qw);
  EXPECT_FLOAT_EQ(pose.qx, out_pose.qx);
  EXPECT_FLOAT_EQ(pose.qy, out_pose.qy);
  EXPECT_FLOAT_EQ(pose.qz, out_pose.qz);

  error = isaac_stop_application(app);
  EXPECT_EQ(error, isaac_error_success);

  isaac_destroy_application(&app);
  EXPECT_EQ(0, app);
}
