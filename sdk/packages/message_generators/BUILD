"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "message_generators",
    deps = [
        ":binary_tensor_generator",
        ":camera_generator",
        ":camera_intrinsics_generator",
        ":detections_2_generator",
        ":detections_3_generator",
        ":differential_base_control_generator",
        ":differential_base_state_generator",
        ":flatscan_generator",
        ":holonomic_base_control_generator",
        ":image_feature_generator",
        ":image_loader",
        ":lattice_generator",
        ":pan_tilt_state_generator",
        ":parking_spot_list_generator",
        ":plan_2_generator",
        ":point_cloud_generator",
        ":polyline_2_generator",
        ":pose_generator",
        ":range_scan_generator",
        ":tensor_generator",
        ":trajectory_list_generator",
    ],
)

isaac_component(
    name = "differential_base_control_generator",
    visibility = ["//visibility:public"],
    deps = [
        "//messages/state:differential_base",
        "//packages/engine_gems/state:io",
    ],
)

isaac_component(
    name = "differential_base_state_generator",
    visibility = ["//visibility:public"],
    deps = [
        "//messages/state:differential_base",
        "//packages/engine_gems/state:io",
    ],
)

isaac_component(
    name = "lattice_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "point_cloud_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "camera_generator",
    deps = [
        "@com_nvidia_isaac_engine//engine/core/image",
    ],
)

isaac_component(
    name = "tensor_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/gems/tensor",
    ],
)

isaac_component(
    name = "flatscan_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "holonomic_base_control_generator",
    visibility = ["//visibility:public"],
    deps = [
        "//messages/state:holonomic_base",
        "//packages/engine_gems/state:io",
    ],
)

isaac_component(
    name = "image_feature_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/image",
        "@com_nvidia_isaac_engine//engine/core/tensor",
    ],
)

isaac_component(
    name = "image_loader",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/image",
    ],
)

isaac_component(
    name = "camera_intrinsics_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/image",
    ],
)

isaac_component(
    name = "plan_2_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_component(
    name = "pose_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_component(
    name = "range_scan_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "rigid_body_3_group_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "detections_2_generator",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ml/gems:object_detection_utils",
        "@com_nvidia_isaac_engine//engine/gems/geometry:n_cuboid",
    ],
)

isaac_component(
    name = "confusion_matrix_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/tensor",
    ],
)

isaac_component(
    name = "trajectory_list_generator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "pan_tilt_state_generator",
    deps = [
        "//messages/state:pan_tilt",
        "//packages/engine_gems/state:io",
    ],
)

isaac_component(
    name = "detections_3_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_component(
    name = "binary_tensor_generator",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/gems/tensor",
    ],
)

isaac_component(
    name = "parking_spot_list_generator",
    visibility = ["//visibility:public"],
    deps = [],
)

isaac_component(
    name = "polyline_2_generator",
    visibility = ["//visibility:public"],
    deps = [],
)