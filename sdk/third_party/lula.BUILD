"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "lula",
    srcs = [
        "lib/libconsole_bridge.so.0.4",
        "lib/libglog.so.0",
        "lib/libglog.so.0.4.0",
        "lib/liblula_interface.so",
        "lib/liblula_kinematics.so",
        "lib/liblula_math.so",
        "lib/liblula_opt.so",
        "lib/liblula_rmpflow.so",
        "lib/liblula_util.so",
        "lib/libtinyxml.so.2.6.2",
        "lib/liburdfdom_model.so.1.0",
    ],
    hdrs = [
        "include/lula/lula.h",
        "include/lula/rmpflow.h",
        "include/lula/robot_description.h",
    ],
    linkopts = [
        "-Wl,--no-as-needed," +
        "-l:libyaml-cpp.so.0.6," +
        "--as-needed",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
        "@yaml-cpp//:libyaml-cpp.so",
    ],
)
