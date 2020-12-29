"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# Description:
#  µWS ("microWS") is a WebSocket and HTTP implementation for clients and servers.
#  Simple, efficient and lightweight.

cc_library(
    name = "uwebsockets",
    srcs = [
        "src/Epoll.cpp",
        "src/Extensions.cpp",
        "src/Group.cpp",
        "src/HTTPSocket.cpp",
        "src/Hub.cpp",
        "src/Networking.cpp",
        "src/Node.cpp",
        "src/Socket.cpp",
        "src/WebSocket.cpp",
    ],
    hdrs = glob(["src/*.h"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
    deps = [
        "@boringssl//:ssl",
        "@net_zlib_zlib//:zlib",
    ],
)
