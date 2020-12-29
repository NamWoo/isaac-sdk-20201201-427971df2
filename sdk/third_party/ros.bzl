"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:deps.bzl", "isaac_http_archive")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies required for ROS
def isaac_ros_workspace():
    isaac_http_archive(
        name = "isaac_ros_bridge_x86_64",
        build_file = clean_dep("//third_party:ros.BUILD"),
        sha256 = "3c8646eeac4866e1fcaa0db95023e28d6d5a506099aced0901c318683d6decee",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros_melodic_x86_64-20190809-tar-gz",
        type = "tar.gz",
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )

    isaac_http_archive(
        name = "isaac_ros_bridge_aarch64_xavier",
        build_file = clean_dep("//third_party:ros_xavier.BUILD"),
        sha256 = "be95ff7771400129cfa1c70439af5dd5ebedc9be3673db2391f9bcb9d0d0f848",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros_melodic_aarch64_xavier-20190916-tar-gz",
        type = "tar.gz",
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )
