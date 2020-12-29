"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "components",
    deps = [
        ":dummy_pose2d_codelets",
        ":dummy_status",
    ],
)

isaac_component(
    name = "dummy_pose2d_codelets",
    filename = "dummy_pose2d_codelets",
)

isaac_component(
    name = "dummy_status",
    filename = "dummy_status",
)

py_test(
    name = "pycodelet_test",
    srcs = ["pycodelet_test.py"],
    data = [
        "pycodelet_test_cpp2py.app.json",
        "pycodelet_test_img.app.json",
        "pycodelet_test_py2cpp.app.json",
        "pycodelet_test_py2py.app.json",
        "pycodelet_test_sync.app.json",
        "//messages:camera_proto",
        "//messages:math_proto",
        "//messages:uuid_proto",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/pyalice/tests:libcomponents_module.so",
        "@isaac_assets",
    ],
    tags = [
        "host",
        "performance",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "pynode_test",
    srcs = ["pynode_test.py"],
    data = [
        "pynode_failure.json",
        "pynode_success.json",
        "pynode_test_status.app.json",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/pyalice/tests:libcomponents_module.so",
    ],
    tags = ["host"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "buffer_test",
    srcs = ["buffer_test.py"],
    data = [
        "buffer_test.app.json",
        "//messages:camera_proto",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/pyalice/tests:libcomponents_module.so",
        "@ml_test_data",
    ],
    tags = ["host"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "pyapp_config_test",
    srcs = ["pyapp_config_test.py"],
    data = [
        "pyapp_config_test.app.json",
    ],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "pymessage_test",
    srcs = ["pymessage_test.py"],
    data = [
        "pymessage_test.app.json",
        "//messages:camera_proto",
        "//messages:detections_proto",
        "//packages/message_generators:libmessage_generators_module.so",
    ],
    tags = [
        "performance",
        "pytest",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "composite_test",
    srcs = ["composite_test.py"],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "application_test",
    srcs = ["application_test.py"],
    data = [
        "application_test.subgraph.json",
        "mock_asset_path/mock_installed_application_test.subgraph.json",
        "//packages/behavior_tree:libbehavior_tree_module.so",
        "//packages/json:libjson_module.so",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/viewers:libviewers_module.so",
    ],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "status_test",
    srcs = ["status_test.py"],
    data = [
    ],
    tags = [
        "performance",
        "pytest",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "component_test",
    srcs = ["component_test.py"],
    data = [
        "//packages/viewers:libviewers_module.so",
    ],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "pose3_test",
    srcs = ["pose3_test.py"],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "test_logger",
    srcs = ["test_logger.py"],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)

py_test(
    name = "test_version",
    srcs = ["test_version.py"],
    tags = ["pytest"],
    deps = [
        "//packages/pyalice",
    ],
)
