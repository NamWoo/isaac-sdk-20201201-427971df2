"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:pybind.bzl", "pybind_library")

pybind_library(
    name = "bindings",
    cc_deps = [
        "//messages",
        "//messages:proto_registry",
        "//messages/utils",
        "@com_nvidia_isaac_engine//engine:version",
        "@com_nvidia_isaac_engine//engine/alice",
        "@com_nvidia_isaac_engine//engine/core/tensor:element_type",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
    ],
    cc_srcs = glob([
        "bindings/*.cpp",
        "bindings/*.hpp",
    ]),
)

py_library(
    name = "components",
    srcs = [
        "Application.py",
        "CapnpMessages.py",
        "Codelet.py",
        "CodeletBackend.py",
        "CodeletFlowControl.py",
        "CodeletHooks.py",
        "Component.py",
        "Composite.py",
        "Message.py",
        "Node.py",
        "Pose3.py",
        "__init__.py",
        "config.py",
        "gui/__init__.py",
        "gui/composite_widget.py",
        "module_explorer.py",
    ],
    data = ["@com_nvidia_isaac_engine//engine:py_init"],
    visibility = ["//visibility:public"],
    deps = [":bindings"],
)

py_library(
    name = "pyalice",
    srcs = [
        "isaac/__init__.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":components",
        "//packages/cask:pycask",
    ],
)

exports_files(["jupyter_launcher.sh.tpl"])
