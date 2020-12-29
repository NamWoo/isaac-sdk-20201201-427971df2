/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <unistd.h>
#include <cstring>

#include "engine/gems/serialization/capnp.hpp"
#include "gtest/gtest.h"
#include "messages/ping.capnp.h"
#include "packages/engine_c_api/isaac_c_api.h"

namespace {
constexpr int64_t kMegaByte = 1024 * 1024;  // in bytes
constexpr int64_t kMaxTimeDiff = 100'000;   // in nanoseconds
}  // namespace

class CApiMessage : public ::testing::Test {
 protected:
  isaac_error_t error;
  isaac_handle_t app{0};
  const char* json_files[1] = {(char*)""};
  const char* asset_path = (char*)"";
  const char* app_file = (char*)"packages/engine_c_api/tests/test.app.json";
  const char* module_path[1] = {(char*)""};

 public:
  CApiMessage() {
    isaac_create_application(asset_path, app_file, module_path, 1, json_files, 0, &app);
    isaac_start_application(app);
  }

  virtual ~CApiMessage() {
    isaac_stop_application(app);
    isaac_destroy_application(&app);
  }
};

TEST_F(CApiMessage, BuildMessage) {
  isaac_uuid_t message;
  isaac_create_message(app, &message);
  EXPECT_NE(message.upper, 0);
  EXPECT_NE(message.lower, 0);

  // pub time
  int64_t pubtime;
  isaac_get_message_pubtime(app, &message, &pubtime);
  EXPECT_EQ(pubtime, 0);

  // acq time
  isaac_set_message_acqtime(app, &message, 200000);
  int64_t acqtime;
  isaac_get_message_acqtime(app, &message, &acqtime);
  EXPECT_EQ(acqtime, 200000);

  // proto id
  isaac_set_message_proto_id(app, &message, 1234);
  int64_t proto_id;
  isaac_get_message_proto_id(app, &message, &proto_id);
  EXPECT_EQ(proto_id, 1234);

  // Prepare some data
  std::vector<unsigned char> data(kMegaByte);
  for (size_t i = 0; i < data.size(); i++) {
    data[i] = i % 256;
  }

  // Add two buffers to the message
  isaac_buffer_t buffer;
  buffer.pointer = data.data();
  buffer.size = data.size();
  buffer.storage = isaac_memory_host;
  int64_t index;
  error = isaac_message_append_buffer(app, &message, &buffer, &index);
  ASSERT_EQ(error, isaac_error_success);
  ASSERT_EQ(index, 0);
  error = isaac_message_append_buffer(app, &message, &buffer, &index);
  ASSERT_EQ(error, isaac_error_success);
  ASSERT_EQ(index, 1);

  // Check that there are two buffers on the message
  int64_t count;
  error = isaac_message_get_buffers(app, &message, nullptr, &count, isaac_memory_host);
  ASSERT_EQ(error, isaac_error_success);
  ASSERT_EQ(count, 2);

  // Check that the buffers have the correct data
  isaac_buffer_t buffers[2];
  error = isaac_message_get_buffers(app, &message, buffers, &count, isaac_memory_host);
  ASSERT_EQ(error, isaac_error_success);
  ASSERT_NE(buffers[0].pointer, nullptr);
  ASSERT_EQ(buffers[0].size, data.size());
  ASSERT_EQ(buffers[0].storage, isaac_memory_host);
  ASSERT_NE(buffers[1].pointer, nullptr);
  ASSERT_EQ(buffers[1].size, data.size());
  ASSERT_EQ(buffers[1].storage, isaac_memory_host);
  for (size_t i = 0; i < data.size(); i++) {
    ASSERT_EQ(data[i], buffers[0].pointer[i]);
    ASSERT_EQ(data[i], buffers[1].pointer[i]);
  }

  isaac_destroy_message(app, &message);
}

TEST_F(CApiMessage, SendJsonMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  // send message
  isaac_uuid_t out_message;
  isaac_create_message(app, &out_message);
  error = isaac_write_message_json(app, &out_message, &out_json);
  EXPECT_EQ(error, isaac_error_success);
  error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
  EXPECT_EQ(error, isaac_error_success);
}

TEST_F(CApiMessage, ReadSizeJsonMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;
    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      isaac_json_t json = isaac_create_null_json();
      EXPECT_EQ(json.data, nullptr);  // NOTE: json.data == nullptr

      // Get the size
      error = isaac_read_message_json(app, &message, &json);
      // Since json.data is null, we expect an error but size set correctly
      EXPECT_EQ(error, isaac_error_data_not_read);
      EXPECT_EQ(json.size, out_json.size);

      // Release Message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, ReadJsonMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;
    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // Read the data
      isaac_json_t json = isaac_create_null_json();
      char buffer[128] = {'\0'};
      json.data = buffer;
      json.size = sizeof(buffer);
      error = isaac_read_message_json(app, &message, &json);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(json.size, out_json.size);
      EXPECT_EQ(strcmp(json.data, out_json.data), 0);

      // Release Message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, GetJsonMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;

    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // Get the pointer
      isaac_const_json_t const_json = isaac_create_null_const_json();
      error = isaac_get_message_json(app, &message, &const_json);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(const_json.size, out_json.size);
      EXPECT_EQ(strcmp(const_json.data, out_json.data), 0);

      // Release message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, AutoAcqTimeMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    int64_t send_time = 0;
    error = isaac_get_time(app, &send_time);
    EXPECT_EQ(error, isaac_error_success);
    EXPECT_GT(send_time, 0);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;

    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // times must be between these values for test to pass
      const int64_t kMinTimeThreshold = send_time;
      const int64_t kMaxTimeThreshold = send_time + kMaxTimeDiff;

      // Get acqtime
      int64_t acqtime = 0;
      isaac_get_message_acqtime(app, &message, &acqtime);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_GT(acqtime, kMinTimeThreshold);
      EXPECT_LT(acqtime, kMaxTimeThreshold);

      // Release message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, ManualAcqTimeMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    int64_t set_time = 0;
    error = isaac_get_time(app, &set_time);
    EXPECT_EQ(error, isaac_error_success);
    EXPECT_GT(set_time, 0);
    error = isaac_set_message_acqtime(app, &out_message, set_time);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;

    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // Get acqtime
      int64_t acqtime = 0;
      isaac_get_message_acqtime(app, &message, &acqtime);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(acqtime, set_time);

      // Release message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, AutoPubTimeMessage) {
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"JSON payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    int64_t send_time = 0;
    error = isaac_get_time(app, &send_time);
    EXPECT_EQ(error, isaac_error_success);
    EXPECT_GT(send_time, 0);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;

    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // get a rough estimate as to when receive returned
      int64_t receive_time = 0;
      error = isaac_get_time(app, &receive_time);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_GT(receive_time, 0);

      // times must be between these values for test to pass
      const int64_t kMinTimeThreshold = send_time;
      const int64_t kMaxTimeThreshold = send_time + kMaxTimeDiff;

      // get pubtime
      int64_t pubtime = 0;
      isaac_get_message_pubtime(app, &message, &pubtime);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_GT(pubtime, kMinTimeThreshold);
      EXPECT_LT(pubtime, kMaxTimeThreshold);

      // Release message
      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, ProtoAutoConvert) {
  const uint64_t kPingProtoId = 13914612606212000694llu;  // proto_id of PingProto msg
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"Proto payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);
    // enable JSON <-> Proto conversion
    isaac_set_message_proto_id(app, &out_message, kPingProtoId);
    EXPECT_EQ(error, isaac_error_success);
    isaac_set_message_auto_convert(app, &out_message, isaac_message_type_proto);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;
    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
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
  }
}

TEST_F(CApiMessage, MessageWithBuffers) {
  const int64_t kBufferCount = 3;
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"Proto payload"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  // Prepare some data
  std::vector<unsigned char> data[kBufferCount];
  isaac_buffer_t buffers[kBufferCount];
  for (unsigned char i = 0; i < kBufferCount; i++) {
    data[i] = {i, i, i, i};
    buffers[i].pointer = data[i].data();
    buffers[i].size = data[i].size();
    buffers[i].storage = isaac_memory_host;
  }

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    EXPECT_EQ(error, isaac_error_success);

    // attach buffers
    for (int64_t i = 0; i < kBufferCount; i++) {
      int64_t buffer_index = -1;
      error = isaac_message_append_buffer(app, &out_message, &buffers[i], &buffer_index);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(buffer_index, i);
    }

    // publish message
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;
    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // Get the buffers
      isaac_buffer_t receive_buffers[kBufferCount];
      int64_t receive_buffers_count = kBufferCount;
      error = isaac_message_get_buffers(app, &message, receive_buffers, &receive_buffers_count,
                                        isaac_memory_host);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(receive_buffers_count, kBufferCount);

      // make sure buffers match what we sent
      for (unsigned char i = 0; i < receive_buffers_count; i++) {
        EXPECT_EQ(0, memcmp(receive_buffers[i].pointer, data[i].data(), data[i].size()));
        EXPECT_EQ(receive_buffers[i].size, data[i].size());
        EXPECT_EQ(receive_buffers[i].storage, isaac_memory_host);
      }

      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}

TEST_F(CApiMessage, ProtoMessage) {
  const uint64_t kPingProtoId = 13914612606212000694llu;  // proto_id of PingProto msg
  isaac_const_json_t out_json;
  out_json.data = R"%%%({"message":"foo"})%%%";
  out_json.size = strlen(out_json.data) + 1;

  // initialize a message
  ::capnp::MallocMessageBuilder message;
  PingProto::Builder test = message.initRoot<PingProto>();
  test.setMessage("foo");

  isaac::CpuBuffer bytes;
  const auto segments = message.getSegmentsForOutput();
  std::vector<const uint8_t*> segments_ptrs;
  std::vector<int64_t> segments_sizes;
  segments_ptrs.resize(segments.size());
  segments_sizes.resize(segments.size());
  constexpr uint16_t word_length = sizeof(capnp::word);
  for (uint64_t i = 0; i < segments.size(); ++i) {
    segments_ptrs[i] = reinterpret_cast<const uint8_t*>(segments[i].begin());
    segments_sizes[i] = segments[i].size() * word_length;
  }
  EXPECT_GT(segments.size(), 0);

  for (int i = 0; i < 5; i++) {
    // send message
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_set_message_proto_segments(
        app, &out_message, reinterpret_cast<const void**>(segments_ptrs.data()),
        reinterpret_cast<int64_t*>(segments_sizes.data()), segments_ptrs.size());
    EXPECT_EQ(error, isaac_error_success);
    // enable JSON <-> Proto conversion
    isaac_set_message_proto_id(app, &out_message, kPingProtoId);
    EXPECT_EQ(error, isaac_error_success);
    error = isaac_publish_message(app, "node", "ledger", "in", &out_message);
    EXPECT_EQ(error, isaac_error_success);

    // Receive message
    isaac_uuid_t message;
    error = isaac_receive_latest_new_message(app, "node", "ledger", "out", &message);
    EXPECT_EQ(error, isaac_error_success);
    if (error == isaac_error_success) {
      // Gets the Json representation
      isaac_const_json_t const_json = isaac_create_null_const_json();
      error = isaac_get_message_json(app, &message, &const_json);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(const_json.size, out_json.size);
      EXPECT_EQ(strcmp(const_json.data, out_json.data), 0);

      // Gets the Proto bytes
      uint64_t num_segments = 0;
      std::string bytes_read;
      error = isaac_read_message_proto_segments(app, &message, nullptr, nullptr, &num_segments);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(num_segments, segments.size());

      std::vector<const uint8_t*> recv_segment_ptrs;
      std::vector<uint64_t> recv_segment_sizes;
      recv_segment_ptrs.resize(num_segments);
      recv_segment_sizes.resize(num_segments);
      error = isaac_read_message_proto_segments(
          app, &message, reinterpret_cast<const void**>(recv_segment_ptrs.data()),
          recv_segment_sizes.data(), &num_segments);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(recv_segment_sizes[0], segments_sizes[0]);
      EXPECT_EQ(strcmp(reinterpret_cast<const char*>(recv_segment_ptrs[0]),
                       reinterpret_cast<const char*>(segments[0].begin())),
                0);

      // get proto id
      int64_t proto_id = 0;
      error = isaac_get_message_proto_id(app, &message, &proto_id);
      EXPECT_EQ(error, isaac_error_success);
      EXPECT_EQ(proto_id, kPingProtoId);

      // Reads proto
      // ::capnp::SegmentArrayMessageReader
      std::vector<kj::ArrayPtr<const ::capnp::word>> kj_segments;
      for (uint64_t i = 0; i < num_segments; i++) {
        kj_segments.push_back(kj::ArrayPtr<const ::capnp::word>(
            reinterpret_cast<const ::capnp::word*>(recv_segment_ptrs[i]),
            reinterpret_cast<const ::capnp::word*>(recv_segment_ptrs[i] + recv_segment_sizes[i])));
      }
      ::capnp::SegmentArrayMessageReader reader(
          kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>>(kj_segments.data(),
                                                                kj_segments.size()));

      auto proto_reader = reader.getRoot<PingProto>();
      EXPECT_TRUE(proto_reader.hasMessage());
      auto payload = proto_reader.getMessage();
      EXPECT_EQ(std::strcmp(payload.cStr(), "foo"), 0);

      error = isaac_release_message(app, &message);
      EXPECT_EQ(error, isaac_error_success);
    }
  }
}
