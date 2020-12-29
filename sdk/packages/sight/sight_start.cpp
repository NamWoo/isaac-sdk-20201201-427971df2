/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/modules.hpp"
#include "engine/gems/sight/sight.hpp"
#include "packages/sight/AliceSight.hpp"
#include "packages/sight/PoseTreeJsonBridge.hpp"

extern "C" {

// The following string defines the websight graph and basic configuration
constexpr char kWebsightJsonText[] = R"???(
{
  "config": {
    "_statistics": {
      "NodeStatistics": {
        "tick_period": "1 Hz"
      }
    },
    "_pose_tree_bridge": {
      "PoseTreeJsonBridge": {
        "tick_period": "50ms"
      }
    },
    "_interactive_markers_bridge": {
      "InteractiveMarkersBridge": {
        "tick_period": "50ms"
      }
    }
  },
  "graph": {
    "nodes": [
      {
        "name": "websight",
        "start_order": -1100,
        "components": [
          {
            "name": "isaac.alice.MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "WebsightServer",
            "type": "isaac::sight::WebsightServer"
          },
          {
            "name": "isaac.alice.SightChannelStatus",
            "type": "isaac::alice::SightChannelStatus"
          },
          {
            "name": "isaac.sight.AliceSight",
            "type": "isaac::sight::AliceSight"
          }
        ]
      },
      {
        "name": "_config_bridge",
        "start_order": -1000,
        "components": [
          {
            "name": "isaac.alice.MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "isaac.alice.ConfigBridge",
            "type": "isaac::alice::ConfigBridge"
          }
        ]
      },
      {
        "name": "_statistics",
        "start_order": -1000,
        "components": [
          {
            "name": "isaac.alice.MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "NodeStatistics",
            "type": "isaac::alice::NodeStatistics"
          }
        ]
      },
      {
        "name": "_pose_tree_bridge",
        "start_order": -1000,
        "components": [
          {
            "name": "isaac.alice.MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "PoseTreeJsonBridge",
            "type": "isaac::sight::PoseTreeJsonBridge"
          }
        ]
      },
      {
        "name": "_interactive_markers_bridge",
        "start_order": -1000,
        "components": [
          {
            "name": "isaac.alice.MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "InteractiveMarkersBridge",
            "type": "isaac::alice::InteractiveMarkersBridge"
          }
        ]
      }
    ],
    "edges": [
      {
        "source": "websight/WebsightServer/config",
        "target": "_config_bridge/isaac.alice.ConfigBridge/request"
      },
      {
        "source": "_config_bridge/isaac.alice.ConfigBridge/reply",
        "target": "websight/WebsightServer/config_reply"
      },
      {
        "source": "websight/WebsightServer/statistics",
        "target": "_statistics/NodeStatistics/request"
      },
      {
        "source": "_statistics/NodeStatistics/statistics",
        "target": "websight/WebsightServer/statistics_reply"
      },
      {
        "source": "_pose_tree_bridge/PoseTreeJsonBridge/pose_tree",
        "target": "websight/WebsightServer/pose_tree_reply"
      },
      {
        "source": "websight/WebsightServer/interactive_markers",
        "target": "_interactive_markers_bridge/InteractiveMarkersBridge/request"
      },
      {
        "source": "_interactive_markers_bridge/InteractiveMarkersBridge/reply",
        "target": "websight/WebsightServer/interactive_markers_reply"
      }
    ]
  }
})???";

void IsaacModuleInitialize(void* void_app) {
  LOG_INFO("Loading websight...");

  auto* app = static_cast<isaac::alice::Application*>(void_app);
  if (app->findNodeByName("websight") == nullptr) {
    app->loadFromText(kWebsightJsonText);
  }

  // Set the correct pointer for the raw sight interface
  isaac::sight::ResetSight(
      app->findComponentByName<isaac::sight::AliceSight>("websight/isaac.sight.AliceSight"));
}
}
