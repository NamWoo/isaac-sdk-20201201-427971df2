"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "glfw",
    srcs = [
        # common source files
        "src/internal.h",
        "src/context.c",
        "src/init.c",
        "src/input.c",
        "src/monitor.c",
        "src/vulkan.c",
        "src/window.c",
        # x11 specific source files
        "src/egl_context.c",
        "src/egl_context.h",
        "src/glx_context.c",
        "src/glx_context.h",
        "src/linux_joystick.c",
        "src/linux_joystick.h",
        "src/posix_time.c",
        "src/posix_time.h",
        "src/posix_tls.c",
        "src/posix_tls.h",
        "src/x11_init.c",
        "src/x11_monitor.c",
        "src/x11_platform.h",
        "src/x11_window.c",
        "src/xkb_unicode.c",
        "src/xkb_unicode.h",
    ],
    hdrs = glob(["include/GLFW/*.h"]),
    copts = [
        "-Wno-format-truncation",
    ],
    defines = [
        "_GLFW_X11",
    ],
    includes = [
        "include",
        "src",
    ],
    linkopts = [
        "-ldl",
        "-lX11",
        "-lXrandr",
        "-lXinerama",
        "-lXcursor",
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
)
