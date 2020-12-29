"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")
load("//messages:messages.bzl", "create_message_proto_libraries", "message_proto_library_names")

filegroup(
    name = "capnp",
    srcs = glob(["*.capnp"]),
    visibility = ["//visibility:public"],
)

# Update //messages:messages.bzl if you change the proto dependencies.
create_message_proto_libraries()

isaac_cc_library(
    name = "camera",
    hdrs = ["camera.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "camera_proto",
        "@com_nvidia_isaac_engine//engine/gems/geometry:pinhole",
    ],
)

isaac_cc_library(
    name = "element_type",
    srcs = ["element_type.cpp"],
    hdrs = ["element_type.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "element_type_proto",
        "@com_nvidia_isaac_engine//engine/core/tensor:element_type",
        "@com_nvidia_isaac_engine//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "geometry",
    hdrs = ["geometry.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "geometry_proto",
        "math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:n_cuboid",
        "@com_nvidia_isaac_engine//engine/gems/geometry:plane",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polyline",
    ],
)

isaac_cc_library(
    name = "image",
    srcs = ["image.cpp"],
    hdrs = ["image.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "element_type",
        "image_proto",
        "tensor_proto",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
        "@com_nvidia_isaac_engine//engine/gems/image",
    ],
)

isaac_cc_library(
    name = "json",
    hdrs = ["json.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "json_proto",
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
    ],
)

isaac_cc_library(
    name = "math",
    hdrs = ["math.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "math_proto",
        "@com_nvidia_isaac_engine//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "tensor",
    srcs = ["tensor.cpp"],
    hdrs = ["tensor.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "element_type",
        "tensor_proto",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/buffers",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
        "@com_nvidia_isaac_engine//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "uuid",
    hdrs = ["uuid.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "uuid_proto",
        "@com_nvidia_isaac_engine//engine/gems/uuid",
    ],
)

isaac_cc_library(
    name = "messages",
    hdrs = ["messages.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "camera",
        "geometry",
        "image",
        "json",
        "math",
        "tensor",
        "uuid",
    ] + message_proto_library_names(),
)

isaac_cc_library(
    name = "proto_registry",
    srcs = [
        "proto_registry.cpp",
    ],
    hdrs = [
        "proto_registry.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":messages",
        "@com_nvidia_isaac_engine//engine/alice:message",
    ],
)
