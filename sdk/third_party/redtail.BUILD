"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# The stereo_depth_dnn lib is used to run the stereoDNN.

load("@com_nvidia_isaac_engine//bzl:cc_cuda_library.bzl", "cc_cuda_library")

stereodnn_includes = ["stereoDNN/lib"]

stereodnn_hdrs = [
    "stereoDNN/lib/conv_utils.h",
    "stereoDNN/lib/internal_macros.h",
    "stereoDNN/lib/internal_utils.h",
    "stereoDNN/lib/redtail_tensorrt_plugins.h",
    "stereoDNN/sample_app/networks.h",
]

cc_library(
    name = "stereo_depth_dnn",
    srcs = [
        "stereoDNN/lib/conv3d_plugin.cpp",
        "stereoDNN/lib/conv3d_transpose_plugin.cpp",
        "stereoDNN/lib/conv_utils.cpp",
        "stereoDNN/lib/cost_volume_plugin.cpp",
        "stereoDNN/lib/elu_plugin.cpp",
        "stereoDNN/lib/internal_utils.cpp",
        "stereoDNN/lib/padding_plugin.cpp",
        "stereoDNN/lib/slice_plugin.cpp",
        "stereoDNN/lib/softargmax_plugin.cpp",
        "stereoDNN/lib/transform_plugin.cpp",
        "stereoDNN/sample_app/resnet18_2D_513x257_net.cpp",
    ],
    hdrs = stereodnn_hdrs,
    copts = [
        "-Wno-deprecated-declarations",
    ],
    includes = stereodnn_includes,
    visibility = ["//visibility:public"],
    deps = [
        "stereo_depth_dnn_cuda_kernels",
        "@com_nvidia_isaac_engine//third_party:cudnn",
    ],
)

cc_cuda_library(
    name = "stereo_depth_dnn_cuda_kernels",
    srcs = [
        "stereoDNN/lib/kernels.cu.cpp",
    ],
    hdrs = stereodnn_hdrs,
    copts = [
        "-Wno-deprecated-declarations",
    ],
    includes = stereodnn_includes,
    deps = [
        "@com_nvidia_isaac_sdk//third_party:tensorrt",
    ],
)

# Copy kernels.cu to kernels.cu.cpp so that isaac can build it
genrule(
    name = "kernels_cu_cpp",
    srcs = ["stereoDNN/lib/kernels.cu"],
    outs = ["stereoDNN/lib/kernels.cu.cpp"],
    cmd = "cp $< $@",
)

filegroup(
    name = "resnet18_2D_weights_and_plan",
    srcs = [
        "stereoDNN/models/ResNet-18_2D/TensorRT/trt_weights.bin",
        "stereoDNN/models/ResNet-18_2D/TensorRT/trt_weights.bin.plan.xavier",
        "stereoDNN/models/ResNet-18_2D/TensorRT/trt_weights_fp16.bin",
        "stereoDNN/models/ResNet-18_2D/TensorRT/trt_weights_fp16.bin.plan.tx2",
    ],
    visibility = ["//visibility:public"],
)
