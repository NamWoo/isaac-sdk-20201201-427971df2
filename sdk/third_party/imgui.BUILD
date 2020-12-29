"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "imgui",
    srcs = [
        "imgui.cpp",
        "imgui_draw.cpp",
        "imgui_widgets.cpp",
    ],
    hdrs = [
        "imconfig.h",
        "imgui.h",
        "imgui_internal.h",
        "imstb_rectpack.h",
        "imstb_textedit.h",
        "imstb_truetype.h",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "examples_glfw_opengl3",
    srcs = [
        "examples/imgui_impl_glfw.cpp",
        "examples/imgui_impl_opengl3.cpp",
    ],
    hdrs = [
        "examples/imgui_impl_glfw.h",
        "examples/imgui_impl_opengl3.h",
    ],
    includes = ["examples"],
    visibility = ["//visibility:public"],
    deps = [
        ":imgui",
        "@gl3w",
        "@glfw",
    ],
)

cc_library(
    name = "demo",
    srcs = [
        "imgui_demo.cpp",
    ],
    deps = [":imgui"],
)

cc_binary(
    name = "demo_glfw_opengl3",
    srcs = [
        "examples/example_glfw_opengl3/main.cpp",
    ],
    deps = [
        ":demo",
        ":examples_glfw_opengl3",
    ],
)

filegroup(
    name = "fonts",
    srcs = glob(["misc/fonts/*.ttf"]),
    visibility = ["//visibility:public"],
)
