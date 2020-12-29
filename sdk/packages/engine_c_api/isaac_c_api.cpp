/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "isaac_c_api.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "packages/engine_c_api/application_c_api.hpp"
#include "packages/engine_c_api/message_c_api.hpp"

namespace {

#define VALIDATE_HANDLE(handle, map)     \
  do {                                   \
    if ((map).count(handle) == 0) {      \
      return isaac_error_invalid_handle; \
    }                                    \
  } while (0)

static std::atomic<int64_t> nextHandle{0};
static std::unordered_map<isaac_handle_t, std::unique_ptr<isaac::alice::ApplicationCApi>>
    id_to_app_map{};

// Generate a unique handle.
// Handles are not reused.
isaac_error_t GenerateUniqueHandle(isaac_handle_t* handle) {
  int64_t potentialHandle =
      nextHandle.fetch_add(1) + 1;  // fetch_add is post-increment
                                    // add +1 so it behaves like a pre-increment

  if (potentialHandle == std::numeric_limits<decltype(potentialHandle)>::max()) {
    // we have exhausted all possible handles
    *handle = 0;
    return isaac_error_no_handles_available;
  }

  *handle = potentialHandle;
  return isaac_error_success;
}

}  // namespace

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
//                 APPLICATION API                                            //
////////////////////////////////////////////////////////////////////////////////

isaac_error_t isaac_create_application(const char* asset_path, const char* app_filename,
                                       const char** module_paths, int32_t num_module_paths,
                                       const char** json_files, int32_t num_json_files,
                                       isaac_handle_t* app_handle) {
  if (num_module_paths < 0 || num_json_files < 0 || !app_handle) {
    return isaac_error_invalid_parameter;
  }

  isaac_error_t status = GenerateUniqueHandle(app_handle);
  RETURN_ON_ERROR(status);

  std::unique_ptr<isaac::alice::ApplicationCApi> app{
      isaac::alice::ApplicationCApi::CreateApplicationCApi(
          asset_path, app_filename, module_paths, num_module_paths, json_files, num_json_files)};

  if (!app) {
    return isaac_error_bad_allocation;
  }

  id_to_app_map[*app_handle] = std::move(app);

  return isaac_error_success;
}

isaac_error_t isaac_destroy_application(isaac_handle_t* app_handle) {
  if (!app_handle) {
    return isaac_error_invalid_parameter;
  }

  id_to_app_map.erase(*app_handle);
  *app_handle = 0;
  return isaac_error_success;
}

isaac_error_t isaac_start_application(isaac_handle_t app_handle) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->startApplication();
}

isaac_error_t isaac_stop_application(isaac_handle_t app_handle) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->stopApplication();
}

isaac_error_t isaac_create_message(isaac_handle_t app_handle, isaac_uuid_t* message_uuid) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->createMessage(message_uuid);
}

isaac_error_t isaac_destroy_message(isaac_handle_t app_handle, isaac_uuid_t* message_uuid) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->destroyMessage(message_uuid);
}

isaac_error_t isaac_publish_message(isaac_handle_t app_handle, const char* node_name,
                                    const char* component_name, const char* channel_name,
                                    const isaac_uuid_t* message_uuid) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->publishMessage(node_name, component_name, channel_name,
                                                   message_uuid);
}

isaac_error_t isaac_receive_latest_new_message(isaac_handle_t app_handle, const char* node_name,
                                               const char* component_name, const char* channel_name,

                                               isaac_uuid_t* message_uuid) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->receiveNewMessage(node_name, component_name, channel_name,
                                                      message_uuid);
}

isaac_error_t isaac_release_message(isaac_handle_t app_handle, isaac_uuid_t* message_uuid) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->releaseMessage(message_uuid);
}

isaac_error_t isaac_get_time(isaac_handle_t app_handle, int64_t* time) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->getTime(time);
}

isaac_error_t isaac_get_pose(isaac_handle_t app_handle, const char* lhs, const char* rhs,
                             int64_t time, isaac_pose_t* pose) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->getPose(lhs, rhs, time, pose);
}

isaac_error_t isaac_set_pose(isaac_handle_t app_handle, const char* lhs, const char* rhs,
                             int64_t time, isaac_pose_t pose) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->setPose(lhs, rhs, pose, time);
}

isaac_error_t isaac_get_parameter(isaac_handle_t app_handle, const char* node,
                                  const char* component, const char* key, isaac_json_t* json) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->getParameter(node, component, key, json);
}

isaac_error_t isaac_set_parameter(isaac_handle_t app_handle, const char* node,
                                  const char* component, const char* key,
                                  const isaac_const_json_t* json) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->setParameter(node, component, key, json);
}

isaac_error_t isaac_set_parameter_from_string(isaac_handle_t app_handle, const char* node,
                                              const char* component, const char* key,
                                              const char* json_string) {
  isaac_const_json_type json;
  json.data = json_string;
  json.size = std::strlen(json.data);
  return isaac_set_parameter(app_handle, node, component, key, &json);
}

////////////////////////////////////////////////////////////////////////////////
//                     MESSAGE API                                            //
////////////////////////////////////////////////////////////////////////////////

isaac_error_t isaac_get_message_json(isaac_handle_t app_handle, const isaac_uuid_t* message_uuid,
                                     isaac_const_json_t* json) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->getJson(json);
}

isaac_error_t isaac_read_message_json(isaac_handle_t app_handle, const isaac_uuid_t* message_uuid,
                                      isaac_json_type* json) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->readJson(json);
}

isaac_error_t isaac_write_message_json(isaac_handle_t app_handle, const isaac_uuid_t* message_uuid,
                                       const isaac_const_json_t* json) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->writeJson(json);
}

isaac_error_t isaac_set_message_auto_convert(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                             isaac_message_convert_t flag) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->setConvertFlag(flag);
}

isaac_error_t isaac_get_message_acqtime(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                        int64_t* time) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->getAcqtime(time);
}

isaac_error_t isaac_set_message_acqtime(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                        int64_t time) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->setAcqtime(time);
}

isaac_error_t isaac_get_message_pubtime(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                        int64_t* time) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->getPubtime(time);
}

isaac_error_t isaac_get_message_proto_id(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                         int64_t* proto_id) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->getProtoId(proto_id);
}

isaac_error_t isaac_set_message_proto_id(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                         uint64_t proto_id) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->setProtoId(proto_id);
}

isaac_error_t isaac_read_message_proto_segments(isaac_handle_t app_handle,
                                                isaac_uuid_t* message_uuid, const void** segments,
                                                uint64_t* sizes, uint64_t* num_segments) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->readProtoBytes(segments, sizes, num_segments);
}

isaac_error_t isaac_set_message_proto_segments(isaac_handle_t app_handle,
                                               isaac_uuid_t* message_uuid, const void** segments,
                                               const int64_t* sizes, const int64_t num_segments) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->setProtoBytes(segments, sizes, num_segments);
}

////////////////////////////////////////////////////////////////////////////////
//                     BUFFER API                                            //
////////////////////////////////////////////////////////////////////////////////

isaac_error_t isaac_message_get_buffers(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                        isaac_buffer_t* buffers, int64_t* buffer_count,
                                        isaac_memory_t preferred_storage) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->getBuffers(buffers, buffer_count, preferred_storage);
}

isaac_error_t isaac_message_append_buffer(isaac_handle_t app_handle, isaac_uuid_t* message_uuid,
                                          const isaac_buffer_t* buffer, int64_t* buffer_index) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  isaac::alice::MessageCApi* message = nullptr;
  id_to_app_map[app_handle]->getMessage(message_uuid, &message);
  return message->appendBuffer(buffer, buffer_index);
}

isaac_json_type isaac_create_null_json() {
  isaac_json_type json;
  json.data = nullptr;
  json.size = 0;
  return json;
}

isaac_const_json_type isaac_create_null_const_json() {
  isaac_const_json_t json;
  json.data = nullptr;
  json.size = 0;
  return json;
}

isaac_error_t isaac_get_external_time_difference(isaac_handle_t app_handle, double external_time,
                                                 int64_t* difference) {
  VALIDATE_HANDLE(app_handle, id_to_app_map);
  return id_to_app_map[app_handle]->getTimeDifference(external_time, difference);
}

#ifdef __cplusplus
}
#endif
