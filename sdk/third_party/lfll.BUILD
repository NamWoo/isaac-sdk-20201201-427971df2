"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "lfll",
    hdrs = glob([
        "lfll/**/*.h",
        "lfll/*.h",
    ]),
    visibility = ["//visibility:public"],
)

# Python script that will be generating and hpp and c file
# form a FIS file ( fuzzy inference system )
py_binary(
    name = "fis_generator",
    srcs = ["python/LFLLFisImporter.py"],
    main = "LFLLFisImporter.py",
    visibility = ["//visibility:public"],
)
