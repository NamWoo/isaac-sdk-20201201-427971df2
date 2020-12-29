"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# Configuration conditions to detect the target platform being built for.
# Host is always assumed to be x86_64. Both Jetson TX2 and Jetson Xavier imply AArch64.
# For more detail refer to
# https://docs.bazel.build/versions/master/configurable-attributes.html
config_setting(
    name = "platform_x86_64",
    define_values = {"target_platform": "x86_64"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "platform_jetpack44",
    define_values = {"target_platform": "jetpack44"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "cpu_host",
    values = {"cpu": "k8"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "cpu_aarch64",
    values = {"cpu": "aarch64"},
    visibility = ["//visibility:public"],
)

# Alias for NVCC compilers needed for toolchain.
filegroup(
    name = "nvcc",
    srcs = ["@nvcc_10"],
    visibility = ["//visibility:public"],
)

# Configuration setting to detect if we are building without GPU support
config_setting(
    name = "isaac_gpu_disabled",
    define_values = {"gpu": "0"},
    visibility = ["//visibility:public"],
)

# Configuration settings for windows
config_setting(
    name = "windows",
    values = {"cpu": "x64_windows"},
    visibility = ["//visibility:public"],
)

# Configuration settings for windows when using msvc compiler
config_setting(
    name = "windows_msvc",
    values = {"cpu": "x64_windows_msvc"},
    visibility = ["//visibility:public"],
)
