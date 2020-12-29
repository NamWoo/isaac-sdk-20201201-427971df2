/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "application_c_api.hpp"

#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/clock.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "messages/proto_registry.hpp"
#include "messages/utils/utils.hpp"
#include "packages/engine_c_api/message_c_api.hpp"

namespace isaac {
namespace alice {

namespace {

#define VALIDATE_MSG_UUID(handle)                                   \
  do {                                                              \
    {                                                               \
      std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_); \
      if (uuid_to_message_map_.count(handle) == 0) {                \
        return isaac_error_invalid_handle;                          \
      }                                                             \
    }                                                               \
  } while (0)

// Helper function for constructing message endpoints and accessing message ledgers
isaac_error_t GetLedger(Application* app, const char* node_name, const char* component_name,
                        const char* channel_name, alice::MessageLedger** ledger,
                        MessageLedger::Endpoint& endpoint) noexcept {
  auto* node = app->findNodeByName(node_name);
  if (!node) {
    return isaac_error_node_not_found;
  }
  *ledger = node->getComponentOrNull<alice::MessageLedger>();
  if (!ledger) {
    return isaac_error_message_ledger_not_found;
  }
  auto* component = node->findComponentByName(component_name);
  // Build target endpoint.
  if (component) {
    endpoint.component = component;
  } else {
    endpoint.component = *ledger;
  }
  endpoint.tag = channel_name;
  return isaac_error_success;
}

}  // namespace

ApplicationCApi* ApplicationCApi::CreateApplicationCApi(
    const char* asset_path, const char* app_filename, const char** module_paths,
    int num_module_paths, const char** json_files, int num_json_files) {
  alice::ApplicationJsonLoader loader(asset_path);

  std::vector<std::string> modules;
  for (int i = 0; i < num_module_paths; i++) {
    modules.emplace_back(module_paths[i]);
  }
  loader.appendModulePaths(modules);

  auto app_json = isaac::serialization::TryLoadJsonFromFile(app_filename);
  if (!app_json) {
    return nullptr;
  }
  loader.loadApp(*app_json);

  for (int i = 0; i < num_json_files; i++) {
    auto more_json = isaac::serialization::TryLoadJsonFromFile(json_files[i]);
    if (!more_json) {
      return nullptr;
    }
    loader.loadMore(*more_json);
  }

  auto capi_instance = std::unique_ptr<ApplicationCApi>(new ApplicationCApi());
  capi_instance->app_ = std::make_unique<Application>(loader);
  capi_instance->app_->disableStopOnCtrlC();
  capi_instance->json_codec_ = std::make_unique<::capnp::JsonCodec>();
  return capi_instance.release();
}

isaac_error_t ApplicationCApi::startApplication() noexcept {
  app_->runAsync();
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::stopApplication() noexcept {
  app_->stopBlocking();
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::createMessage(isaac_uuid_t* uuid) noexcept {
  if (!uuid) {
    return isaac_error_invalid_parameter;
  }

  auto created_message = std::make_unique<MessageCApi>();

  auto gen_uuid = Uuid::Generate();
  created_message->setUuid(gen_uuid.lower(), gen_uuid.upper());
  created_message->getUuid(&uuid->lower, &uuid->upper);

  {
    std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_);
    uuid_to_message_map_[gen_uuid] = std::move(created_message);
  }

  return isaac_error_success;
}

isaac_error_t ApplicationCApi::destroyMessage(const isaac_uuid_t* uuid) noexcept {
  auto message_to_delete = Uuid::FromUInt64(uuid->lower, uuid->upper);
  {
    std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_);
    uuid_to_message_map_.erase(message_to_delete);
  }
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::getMessage(const isaac_uuid_t* uuid,
                                          MessageCApi** message) noexcept {
  if (!uuid || !message) {
    return isaac_error_invalid_parameter;
  }

  auto msg_uuid = Uuid::FromUInt64(uuid->lower, uuid->upper);
  VALIDATE_MSG_UUID(msg_uuid);
  {
    std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_);
    *message = uuid_to_message_map_[msg_uuid].get();
  }
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::publishMessage(const char* node_name, const char* component_name,
                                              const char* channel_name,
                                              const isaac_uuid_t* uuid) noexcept {
  if (!uuid) {
    return isaac_error_invalid_parameter;
  }
  const auto msg_uuid = Uuid::FromUInt64(uuid->lower, uuid->upper);
  VALIDATE_MSG_UUID(msg_uuid);
  MessageCApi* msg = nullptr;
  {
    std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_);
    msg = uuid_to_message_map_[msg_uuid].get();
  }

  // Cannot publish a received message.
  if (msg->message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }

  MessageLedger::Endpoint endpoint;
  MessageLedger* ledger;
  isaac_error_t error =
      GetLedger(app_.get(), node_name, component_name, channel_name, &ledger, endpoint);
  RETURN_ON_ERROR(error);

  MessageBasePtr out_message;
  if (msg->json_string_.size() > 0) {
    if (msg->convert_flag_ == isaac_message_type_proto) {
      // Publishes as ProtoMessage
      auto message_builder = std::make_unique<::capnp::MallocMessageBuilder>();
      auto maybe_builder = GetRootBuilderByTypeId(msg->proto_id_, *message_builder);
      if (!maybe_builder) {
        RETURN_ON_ERROR(isaac_error_invalid_message);
      }
      json_codec_->decode((::kj::StringPtr)msg->json_string_.c_str(), *maybe_builder);
      out_message =
          std::make_shared<MallocProtoMessage>(std::move(message_builder), msg->proto_id_);
    } else {
      // Publishes as JsonMessage
      auto json_message = std::make_shared<alice::JsonMessage>();
      json_message->data = nlohmann::json::parse(msg->json_string_);
      out_message = json_message;
    }
  } else if (msg->proto_segments_.size() > 0 && msg->proto_id_ != 0) {
    // Publishes a ProtoMessage from raw Proto bytes
    std::vector<CpuBuffer> buffers;
    out_message = utils::CreateProtoMessage(msg->uuid_, msg->proto_id_,
                                            std::move(msg->proto_segments_), std::move(buffers));
  } else {
    return isaac_error_invalid_message;
  }

  // Write remaining parts of the message header
  out_message->uuid = msg->uuid_;
  int64_t time;
  getTime(&time);
  out_message->acqtime = msg->acqtime_ == 0 ? time : msg->acqtime_;
  out_message->pubtime = time;

  out_message->type = msg->proto_id_;

  // Write buffers
  for (auto& buffer : msg->buffers_) {
    out_message->buffers.emplace_back(std::move(buffer));
  }
  msg->buffers_.clear();

  // Publish the message
  ledger->provide(endpoint, std::move(out_message));
  ledger->notifyScheduler(endpoint, time);

  // Invalidate the message since it has been published
  destroyMessage(uuid);

  return isaac_error_success;
}

isaac_error_t ApplicationCApi::generateMessageHandle(ConstMessageBasePtr new_message,
                                                     ApplicationCApi* app,
                                                     isaac_uuid_t* uuid) noexcept {
  if (!new_message) {
    return isaac_error_no_message_available;
  }

  app->createMessage(uuid);
  auto msg_uuid = Uuid::FromUInt64(uuid->lower, uuid->upper);
  MessageCApi* msg = nullptr;
  {
    std::lock_guard<std::mutex> lock(uuid_to_message_map_mutex_);
    msg = uuid_to_message_map_[msg_uuid].get();
  }

  // Extract message header
  msg->setUuid(new_message->uuid.upper(), new_message->uuid.lower());
  msg->setAcqtime(new_message->acqtime);
  msg->setPubtime(new_message->pubtime);

  // if the message is a proto message extract the proto id
  auto* proto_msg = dynamic_cast<const ProtoMessageBase*>(new_message.get());
  if (proto_msg) {
    msg->setProtoId(proto_msg->proto_id());
  }

  msg->message_ptr_ = new_message;
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::receiveNewMessage(const char* node_name, const char* component_name,
                                                 const char* channel_name,
                                                 isaac_uuid_t* uuid) noexcept {
  if (!uuid) {
    return isaac_error_invalid_parameter;
  }

  uuid->lower = 0;
  uuid->upper = 0;

  MessageLedger::Endpoint endpoint;
  MessageLedger* ledger;
  isaac_error_t error =
      GetLedger(app_.get(), node_name, component_name, channel_name, &ledger, endpoint);
  RETURN_ON_ERROR(error);

  if (auto* queue = ledger->tryGetQueue(endpoint)) {
    queue->sync();
    if (queue->empty()) {
      return isaac_error_no_message_available;
    } else {
      const auto error2 = generateMessageHandle(queue->latest(), this, uuid);
      queue->popAll();  // FIXME(yangl) Should not only return latest message but give queue access.
      return error2;
    }
  } else {
    return isaac_error_message_queue_not_found;
  }
}

isaac_error_t ApplicationCApi::releaseMessage(isaac_uuid_t* uuid) noexcept {
  // there is no difference between release and destroy
  return destroyMessage(uuid);
}

isaac_error_t ApplicationCApi::getTime(int64_t* time) noexcept {
  if (!time) {
    return isaac_error_invalid_parameter;
  }

  *time = app_->backend()->clock()->timestamp();
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::setPose(const char* lhs, const char* rhs, isaac_pose_t pose,
                                       int64_t time) noexcept {
  Pose3d lhs_T_rhs;
  lhs_T_rhs.translation.x() = pose.px;
  lhs_T_rhs.translation.y() = pose.py;
  lhs_T_rhs.translation.z() = pose.pz;
  auto q = Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz);
  lhs_T_rhs.rotation = lhs_T_rhs.rotation.FromQuaternion(q);

  if (!(app_->backend()->pose_tree()->trySet(lhs, rhs, lhs_T_rhs, ToSeconds(time)))) {
    return isaac_error_pose;
  }
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::getPose(const char* lhs, const char* rhs, int64_t time,
                                       isaac_pose_t* pose) noexcept {
  if (!pose) {
    return isaac_error_invalid_parameter;
  }

  std::optional<Pose3d> lhs_T_rhs = app_->backend()->pose_tree()->tryGet(lhs, rhs, ToSeconds(time));
  if (!lhs_T_rhs) {
    return isaac_error_pose;
  }
  pose->px = (*lhs_T_rhs).translation.x();
  pose->py = (*lhs_T_rhs).translation.y();
  pose->pz = (*lhs_T_rhs).translation.z();
  auto q = (*lhs_T_rhs).rotation.quaternion();
  pose->qw = q.w();
  pose->qx = q.x();
  pose->qy = q.y();
  pose->qz = q.z();

  return isaac_error_success;
}

isaac_error_t ApplicationCApi::getParameter(const char* node, const char* component,
                                            const char* key, isaac_json_t* json) noexcept {
  if (json == nullptr) return isaac_error_invalid_parameter;

  // Get the parameter as a JSON string
  auto* maybe_json = app_->backend()->config_backend()->tryGetJson(node, component, key);
  if (!maybe_json) return isaac_error_parameter_not_found;
  const std::string source_string = maybe_json->dump();
  const size_t source_size = source_string.size();
  const size_t source_size_with_null = source_size + 1;

  // Copy the JSON string to the output buffer
  const bool is_buffer_valid = json->size >= source_size_with_null;
  if (is_buffer_valid) {
    source_string.copy(json->data, source_size);
    json->data[source_size] = '\0';
  }

  // Always return the size necessary to store the JSON string
  json->size = source_size_with_null;
  return is_buffer_valid ? isaac_error_success : isaac_error_data_not_read;
}

isaac_error_t ApplicationCApi::setParameter(const char* node, const char* component,
                                            const char* key,
                                            const isaac_const_json_t* json) noexcept {
  if (json == nullptr) return isaac_error_invalid_parameter;
  if (json->data == nullptr) return isaac_error_invalid_parameter;
  try {
    app_->backend()->config_backend()->setJson(node, component, key,
                                               nlohmann::json::parse(json->data));
  } catch (...) {
    return isaac_error_unknown;
  }
  return isaac_error_success;
}

isaac_error_t ApplicationCApi::getTimeDifference(double external_time, int64_t* difference) {
  if (!difference) return isaac_error_invalid_parameter;
  *difference = app_->backend()->clock()->timestamp() - SecondsToNano(external_time);
  return isaac_error_success;
}

}  // namespace alice
}  // namespace isaac
