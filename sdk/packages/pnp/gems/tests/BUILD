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

isaac_cc_library(
    name = "simu",
    srcs = ["simu.cpp"],
    hdrs = ["simu.hpp"],
    deps = [
        "//packages/pnp/gems:pnp",
    ],
)

isaac_cc_binary(
    name = "epnp_eval",
    srcs = ["epnp_eval.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
    ],
)

cc_test(
    name = "gen_examples",
    size = "small",
    srcs = ["gen_examples.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

cc_test(
    name = "utils_test",
    size = "small",
    srcs = ["utils_test.cpp"],
    deps = [
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

cc_test(
    name = "simu_test",
    size = "small",
    srcs = ["simu_test.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

cc_test(
    name = "epnp_test",
    size = "small",
    srcs = ["epnp_test.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

cc_test(
    name = "epnp_ransac_test",
    size = "small",
    srcs = ["epnp_ransac_test.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pnp_test",
    size = "small",
    srcs = ["pnp_test.cpp"],
    deps = [
        "simu",
        "//packages/pnp/gems:pnp",
        "@gtest//:main",
    ],
)

isaac_pkg(
    name = "pnp-tests-pkg",
    testonly = True,
    srcs = [
        ":epnp_eval",
        ":epnp_ransac_test",
        ":epnp_test",
        ":pnp_test",
        ":simu_test",
        ":utils_test",
    ],
    extension = "tar.gz",
)
