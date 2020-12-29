"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_pkg", "isaac_subgraph")

isaac_subgraph(
    name = "robot_inference_subgraph",
    modules = [
        "ml",
        "ml:tensorflow",
        "planner",
        "//packages/rl/components:rl",
        "//packages/rl/apps/dolly_navigation/components:dolly_navigation_task",
    ],
    subgraph = "robot_inference.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "dolly_demo_sim",
    data = [
        ":robot_inference_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "@dolly_docking_reinforcement_learning_policy",
    ],
    modules = [
        "controller",
        "ml",
        "ml:tensorflow",
        "navigation",
        "planner",
        "sight",
        "utils",
        "//packages/rl/components:rl",
        "//packages/rl/apps/dolly_navigation/components:dolly_navigation_task",
    ],
)

py_binary(
    name = "dolly_navigation",
    srcs = [
        "__init__.py",
        "dolly_navigation.py",
    ],
    data = [
        ":fof_pb_inference.json",
        ":fof_py_inference.json",
        ":multi_agent_pb_inference.json",
        ":multi_agent_py_inference.json",
        ":trainer.config.json",
        "//packages/controller:libcontroller_module.so",
        "//packages/engine_tcp_udp:libengine_tcp_udp_module.so",
        "//packages/ml:libml_module.so",
        "//packages/ml:libtensorflow_module.so",
        "//packages/navigation:libnavigation_module.so",
        "//packages/perception:libperception_module.so",
        "//packages/planner:libplanner_module.so",
        "//packages/rl/apps/dolly_navigation/components:libdolly_navigation_task_module.so",
        "//packages/rl/components:librl_module.so",
        "//packages/sight:libsight_module.so",
        "//packages/utils:libutils_module.so",
        "@dolly_docking_reinforcement_learning_policy",
    ],
    deps = [
        "//packages/ml:pyml",
        "//packages/pyalice",
        "//packages/rl/py_components:off_policy_trainer",
        "//packages/rl/py_components:soft_actor_critic",
    ],
)

isaac_pkg(
    name = "dolly_navigation-pkg",
    srcs = [":dolly_navigation"],
    data = [
        "//packages:py_init",
        "//packages/rl:py_init",
        "//packages/rl/apps:py_init",
        "//packages/rl/apps/dolly_navigation:py_init",
    ],
)
