/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "event_manager.hpp"

#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/clock.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/scheduler/scheduler.hpp"

namespace isaac {
namespace alice {

EventManager::EventManager(Application* app)
: app_(app) {}

void EventManager::onStatusUpdate(Component* component) {
  // The order of the following two blocks matters. The former one synchronously stops nodes which
  // are not `RUNNING` anymore and afterwards asynchronously notifies the backend about the updated
  // status. In order to avoid race conditions, the nodes to stop must be properly stopped before
  // the asynchronous notification to the backend happens. Otherwise, other nodes (such as parent
  // nodes) might react to the node's new status before the node itself really changed.

  // Stop the node if one failed or all succeeded.
  Node* node = component->node();
  const auto node_status = node->getStatus();
  if (node_status != Status::RUNNING) {
    if (node_status == Status::SUCCESS) {
      LOG_DEBUG("Stopping node '%s' because it reached status '%s'",
                node->name().c_str(), ToString(node_status));
    } else {
      LOG_ERROR("Stopping node '%s' because it reached status '%s'",
                node->name().c_str(), ToString(node_status));
    }
    app_->backend()->node_backend()->stopNode(node);
  }

  // Notify the backend about the updated status.
  const std::string event = component->full_name() + "/__status";
  const int64_t target_time = app_->backend()->clock()->timestamp();
  app_->backend()->scheduler()->notify(event, target_time);
}

}  // namespace alice
}  // namespace isaac
