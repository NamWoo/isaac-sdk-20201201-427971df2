"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libml_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libml_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libml_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libml_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libml_module.so_release"],
    }),
    deps = [
        "//third_party:tensorrt",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libtensorrt_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtensorrt_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtensorrt_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtensorrt_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtensorrt_module.so_release"],
    }),
    deps = [
        "//third_party:cublas",
        "//third_party:cudnn",
        "//third_party:curand",
        "//third_party:cusolver",
        "//third_party:tensorrt",
        "@boringssl//:ssl",
        "@tlt_parser",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libtensorflow_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtensorflow_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtensorflow_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtensorflow_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtensorflow_module.so_release"],
    }),
    deps = [
        "//third_party:libtensorflow",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "pyml_target",
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:pyml.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:pyml.so_release"],
    }),
)

py_library(
    name = "pyml",
    srcs = [
        "__init__.py",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:pyml"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:pyml"],
    }),
)

cc_binary(
    name = "libtorch_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtorch_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtorch_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libtorch_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libtorch_module.so_release"],
    }),
    deps = [
        "//third_party:libtorch",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libevaluation_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libevaluation_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libevaluation_module.so_release"],
    }),
    visibility = ["//visibility:public"],
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/ml:libevaluation_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/ml:libevaluation_module.so_release"],
    }),
)