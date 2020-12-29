"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

exports_files(["COPYING"])

cc_library(
    name = "libusb",
    srcs = [
        "libusb/core.c",
        "libusb/descriptor.c",
        "libusb/hotplug.c",
        "libusb/io.c",
        "libusb/os/linux_udev.c",
        "libusb/os/linux_usbfs.c",
        "libusb/os/poll_posix.c",
        "libusb/os/threads_posix.c",
        "libusb/sync.c",
    ],
    hdrs = [
        "Xcode/config.h",
        "libusb/hotplug.h",
        "libusb/libusb.h",
        "libusb/libusbi.h",
        "libusb/os/linux_usbfs.h",
        "libusb/os/poll_posix.h",
        "libusb/os/threads_posix.h",
        "libusb/version.h",
        "libusb/version_nano.h",
    ],
    copts = [
        "-DUSE_UDEV",
        "-DHAVE_LIBUDEV",
    ],
    includes = [
        "Xcode",
        "libusb",
        "libusb/os",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": [
            "@libudev",
        ],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": [
            "@libudev_aarch64",
        ],
    }),
)
