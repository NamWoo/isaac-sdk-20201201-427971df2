"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_pkg")
load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_binary(
    name = "epnp_example",
    srcs = ["epnp_example.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pnp/gems:pnp",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/gems/math",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

isaac_cc_binary(
    name = "epnp_ransac_example",
    srcs = ["epnp_ransac_example.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pnp/gems:pnp",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/gems/math",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

isaac_pkg(
    name = "pnp-examples",
    testonly = True,
    srcs = [
        ":epnp_example",
        ":epnp_ransac_example",
    ],
    extension = "tar.gz",
)
