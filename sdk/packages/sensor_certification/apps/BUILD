"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//packages/sensor_certification:sensor_certification.bzl", "sensor_certification_app")

sensor_certification_app(
    name = "sensor_certification",
    srcs = [
                "Cert.py",
                "Tests.py",
                "TestRunners.py",
                "Utils.py",
            ],
    main = "Cert.py",
    data = [
                "//external:libsensor_module.so",
                "//packages/fiducials:libapril_tags_module.so",
                "//packages/sensor_certification/evaluators:libevaluators_module.so",
                "//packages/sight:libsight_module.so",
                "//packages/viewers:libviewers_module.so",
                "scripts/platform.sh",
            ],
    deps = [
                "//packages/pyalice",
            ],
)
