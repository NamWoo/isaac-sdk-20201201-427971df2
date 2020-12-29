"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")
load("//bzl:module.bzl", "isaac_component")

isaac_component(
    name = "image_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "camera_intrinsics_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "flatscan_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "odometry_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "goal_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "goal_to_ros_action",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "pose_message_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "poses_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_cc_library(
    name = "proto_to_ros_converter",
    hdrs = ["ProtoToRosConverter.hpp"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//messages",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
        "@com_nvidia_isaac_engine//engine/alice",
        "@com_nvidia_isaac_engine//engine/core",
    ],
)

isaac_component(
    name = "ros_node",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_camera_intrinsics",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
        "@com_nvidia_isaac_engine//engine/gems/image",
    ],
)

isaac_component(
    name = "ros_to_differential_base_command",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//messages/state:differential_base",
        "//packages/engine_gems/state:io",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_image",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
        "@com_nvidia_isaac_engine//engine/gems/image",
    ],
)

isaac_component(
    name = "ros_to_pose2_mean_and_covariance",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_to_proto_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_component(
    name = "ros_to_poses",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
)

isaac_cc_library(
    name = "ros_to_proto_converter",
    hdrs = ["RosToProtoConverter.hpp"],
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros_node",
        "//messages",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
        "@com_nvidia_isaac_engine//engine/alice",
        "@com_nvidia_isaac_engine//engine/core",
    ],
)
