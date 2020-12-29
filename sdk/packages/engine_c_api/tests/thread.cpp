/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "engine/core/buffers/buffer.hpp"
#include "engine/core/logger.hpp"
#include "packages/engine_c_api/isaac_c_api.h"

#include "gtest/gtest.h"

namespace isaac {
namespace alice {

class ThreadingMessage : public ::testing::Test {
 protected:
  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/thread.app.json";
  const char* module_path[1] = {(char*)""};

 public:
  ThreadingMessage() {
    isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);
    isaac_start_application(app);
  }

  virtual ~ThreadingMessage() {
    isaac_stop_application(app);
    isaac_destroy_application(&app);
  }
};

TEST_F(ThreadingMessage, ThreadedPublish) {
  const uint64_t kPingProtoId = 13914612606212000694llu;  // proto_id of PingProto msg

  // const uint64_t ImageProtoId = 16240193334533147313u;
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"Proto payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  // Waits for connection
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::vector<std::thread> threads;
  const int kNumThreads = 2;
  const double kMessageDurationSeconds = 5;

  const auto starting_time = std::chrono::steady_clock::now();

  std::atomic<int64_t> receive_cnt_foo1{0};
  std::atomic<int64_t> receive_cnt_foo2{0};

  // Data
  const int rows = 720, cols = 1280;
  CpuBuffer dummy_rgb_image(rows * cols * 3);       // rgb
  CpuBuffer dummy_label_image(rows * cols);         //  uint16
  CpuBuffer dummy_instance_image(rows * cols * 2);  //  uint16

  for (int i = 0; i < kNumThreads; ++i) {
    threads.push_back(std::thread([&] {
      isaac_error_t error;
      double elapsed_seconds = 0;
      while (elapsed_seconds < kMessageDurationSeconds) {
        // send message
        isaac_uuid_t out_message;
        error = isaac_create_message(app, &out_message);
        EXPECT_EQ(error, isaac_error_success);
        error = isaac_write_message_json(app, &out_message, &out_json);
        EXPECT_EQ(error, isaac_error_success);
        // enable JSON <-> Proto conversion
        error = isaac_set_message_proto_id(app, &out_message, kPingProtoId);
        EXPECT_EQ(error, isaac_error_success);
        error = isaac_set_message_auto_convert(app, &out_message, isaac_message_type_proto);
        EXPECT_EQ(error, isaac_error_success);

        // Prepare some data
        isaac_buffer_t append_buffer;
        append_buffer.pointer = dummy_rgb_image.begin();
        append_buffer.size = dummy_rgb_image.size();
        append_buffer.storage = isaac_memory_host;
        // attach buffers
        int64_t buffer_index = -1;
        error = isaac_message_append_buffer(app, &out_message, &append_buffer, &buffer_index);
        EXPECT_EQ(error, isaac_error_success);
        EXPECT_EQ(buffer_index, 0);

        error = isaac_publish_message(app, "ping", "TcpPublisher", "foo1", &out_message);
        EXPECT_EQ(error, isaac_error_success);

        // Receive message
        isaac_uuid_t message;
        error = isaac_receive_latest_new_message(app, "pong", "TcpSubscriber", "foo1", &message);
        EXPECT_TRUE(error == isaac_error_success || error == isaac_error_no_message_available
                    || error == isaac_error_message_queue_not_found);
        if (error == isaac_error_success) {
          receive_cnt_foo1++;

          // Get the pointer
          isaac_const_json_t const_json = isaac_create_null_const_json();
          error = isaac_get_message_json(app, &message, &const_json);
          EXPECT_EQ(error, isaac_error_success);
          EXPECT_EQ(const_json.size, out_json.size);
          EXPECT_EQ(strcmp(const_json.data, out_json.data), 0);

          // get proto id
          int64_t proto_id = 0;
          error = isaac_get_message_proto_id(app, &message, &proto_id);
          EXPECT_EQ(error, isaac_error_success);
          EXPECT_EQ(proto_id, kPingProtoId);

          error = isaac_release_message(app, &message);
          EXPECT_EQ(error, isaac_error_success);
        }
        elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                              std::chrono::steady_clock::now() - starting_time)
                              .count();
      }
    }));
  }

  for (int i = 0; i < kNumThreads; ++i) {
    threads.push_back(std::thread([&] {
      isaac_error_t error;
      double elapsed_seconds = 0;
      while (elapsed_seconds < kMessageDurationSeconds) {
        // send message
        isaac_uuid_t out_message;
        error = isaac_create_message(app, &out_message);
        EXPECT_EQ(error, isaac_error_success);
        error = isaac_write_message_json(app, &out_message, &out_json);
        EXPECT_EQ(error, isaac_error_success);
        // enable JSON <-> Proto conversion
        error = isaac_set_message_proto_id(app, &out_message, kPingProtoId);
        EXPECT_EQ(error, isaac_error_success);
        error = isaac_set_message_auto_convert(app, &out_message, isaac_message_type_proto);
        EXPECT_EQ(error, isaac_error_success);

        // Prepare some data
        isaac_buffer_t append_buffer[2];
        append_buffer[0].pointer = dummy_label_image.begin();
        append_buffer[0].size = dummy_label_image.size();
        append_buffer[0].storage = isaac_memory_host;
        append_buffer[1].pointer = dummy_instance_image.begin();
        append_buffer[1].size = dummy_instance_image.size();
        append_buffer[1].storage = isaac_memory_host;
        // attach buffers
        int64_t buffer_index = -1;
        error = isaac_message_append_buffer(app, &out_message, &append_buffer[0], &buffer_index);
        EXPECT_EQ(error, isaac_error_success);
        EXPECT_EQ(buffer_index, 0);
        error = isaac_message_append_buffer(app, &out_message, &append_buffer[1], &buffer_index);
        EXPECT_EQ(error, isaac_error_success);
        EXPECT_EQ(buffer_index, 1);

        error = isaac_publish_message(app, "ping", "TcpPublisher", "foo2", &out_message);
        EXPECT_EQ(error, isaac_error_success);

        // Receive message
        isaac_uuid_t message;
        error = isaac_receive_latest_new_message(app, "pong", "TcpSubscriber", "foo2", &message);
        EXPECT_TRUE(error == isaac_error_success || error == isaac_error_no_message_available
                    || error == isaac_error_message_queue_not_found);
        if (error == isaac_error_success) {
          receive_cnt_foo2++;

          // Get the pointer
          isaac_const_json_t const_json = isaac_create_null_const_json();
          error = isaac_get_message_json(app, &message, &const_json);
          EXPECT_EQ(error, isaac_error_success);
          EXPECT_EQ(const_json.size, out_json.size);
          EXPECT_EQ(strcmp(const_json.data, out_json.data), 0);

          // get proto id
          int64_t proto_id = 0;
          error = isaac_get_message_proto_id(app, &message, &proto_id);
          EXPECT_EQ(error, isaac_error_success);
          EXPECT_EQ(proto_id, kPingProtoId);

          error = isaac_release_message(app, &message);
          EXPECT_EQ(error, isaac_error_success);
        }
        elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                              std::chrono::steady_clock::now() - starting_time)
                              .count();
      }
    }));
  }

  // Waits for everything stops
  for (auto& t : threads) {
    t.join();
  }

  EXPECT_GT(receive_cnt_foo1, 0);
  EXPECT_GT(receive_cnt_foo2, 0);
};

}  // namespace alice
}  // namespace isaac
