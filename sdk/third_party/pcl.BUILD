"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

exports_files(["LICENSE.txt"])

# Generate pcl_config.h
genrule(
    name = "configure_pcl_config",
    srcs = [
        "build/include/pcl/pcl_config.h",
    ],
    outs = [
        "include/pcl/pcl_config.h",
    ],
    cmd = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": "cp $(SRCS) $@",
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": "sed '/HAVE_MM_MALLOC/d;/HAVE_SSE/d' $(SRCS) > $@",
    }),
)

cc_library(
    name = "config",
    hdrs = [
        "include/pcl/pcl_config.h",
    ],
    strip_include_prefix = "include",
)

cc_library(
    name = "common",
    srcs = glob([
        "common/src/**/*.cpp",
        "common/src/**/*.c",
    ]),
    hdrs = glob([
        "common/include/**/*.hpp",
        "common/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    defines = [],
    includes = [
        "common/include",
    ],
    linkopts = [
        "-lgomp",
    ],
    strip_include_prefix = "common/include",
    visibility = ["//visibility:public"],
    deps = [
        ":config",
        "@boost//:cstdint",
        "@boost//:foreach",
        "@boost//:function",
        "@boost//:noncopyable",
        "@boost//:signals2",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@flann",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "geometry",
    hdrs = glob([
        "geometry/include/**/*.hpp",
        "geometry/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "geometry/include",
)

cc_library(
    name = "features",
    srcs = glob([
        "features/src/**/*.cpp",
        "features/src/**/*.c",
    ]),
    hdrs = glob([
        "features/include/**/*.hpp",
        "features/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "features/include",
    visibility = ["//visibility:public"],
    deps = [
        ":2d",
        ":common",
        ":search",
        "@boost//:random",
    ],
)

cc_library(
    name = "search",
    srcs = glob([
        "search/src/**/*.cpp",
        "search/src/**/*.c",
    ]),
    hdrs = glob([
        "search/include/**/*.hpp",
        "search/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "search/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":kdtree",
        ":octree",
    ],
)

cc_library(
    name = "octree",
    srcs = glob([
        "octree/src/**/*.cpp",
        "octree/src/**/*.c",
    ]),
    hdrs = glob([
        "octree/include/**/*.hpp",
        "octree/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "octree/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        "@boost//:graph",
    ],
)

cc_library(
    name = "kdtree",
    srcs = glob([
        "kdtree/src/**/*.cpp",
        "kdtree/src/**/*.c",
    ]),
    hdrs = glob([
        "kdtree/include/**/*.hpp",
        "kdtree/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "kdtree/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
    ],
)

cc_library(
    name = "2d",
    srcs = ["2d/src/convolution_2d.cpp"],
    hdrs = glob([
        "2d/include/**/*.hpp",
        "2d/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "2d/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":filters",
    ],
)

cc_library(
    name = "filters",
    srcs = glob([
        "filters/src/**/*.cpp",
        "filters/src/**/*.c",
    ]),
    hdrs = glob([
        "filters/include/**/*.hpp",
        "filters/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "filters/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":sample_consensus",
        ":search",
        "@boost//:dynamic_bitset",
        "@boost//:random",
        "@boost//:unordered",
    ],
)

cc_library(
    name = "sample_consensus",
    srcs = glob([
        "sample_consensus/src/**/*.cpp",
        "sample_consensus/src/**/*.c",
    ]),
    hdrs = glob([
        "sample_consensus/include/**/*.hpp",
        "sample_consensus/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "sample_consensus/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":search",
        "@boost//:random",
        "@org_tuxfamily_eigen//:unsupported",
    ],
)

cc_library(
    name = "key_points",
    srcs = glob([
        "key_points/src/**/*.cpp",
        "key_points/src/**/*.c",
    ]),
    hdrs = glob([
        "key_points/include/**/*.hpp",
        "key_points/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "key_points/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
    ],
)

cc_library(
    name = "ml",
    srcs = glob([
        "ml/src/**/*.cpp",
        "ml/src/**/*.c",
    ]),
    hdrs = glob([
        "ml/include/**/*.hpp",
        "ml/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "ml/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
    ],
)

cc_library(
    name = "registration",
    srcs = glob(
        [
            "registration/src/**/*.cpp",
            "registration/src/**/*.c",
        ],
        exclude = [
            "registration/src/pairwise_graph_registration.cpp",
        ],
    ),
    hdrs = glob([
        "registration/include/**/*.hpp",
        "registration/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "registration/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":features",
        ":filters",
        ":sample_consensus",
        ":search",
        "@boost//:get_pointer",
        "@org_tuxfamily_eigen//:unsupported",
    ],
)

cc_library(
    name = "segmentation",
    srcs = glob(
        [
            "segmentation/src/**/*.cpp",
            "segmentation/src/**/*.c",
        ],
        exclude = [
        ],
    ),
    hdrs = glob([
        "segmentation/include/**/*.hpp",
        "segmentation/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
        "-Wno-uninitialized",
    ],
    strip_include_prefix = "segmentation/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":features",
        ":filters",
        ":geometry",
        ":ml",
        ":search",
        "@boost//:multi_array",
        "@boost//:ptr_container",
    ],
)

cc_library(
    name = "tracking",
    srcs = glob(
        [
            "tracking/src/**/*.cpp",
            "tracking/src/**/*.c",
        ],
        exclude = [
        ],
    ),
    hdrs = glob([
        "tracking/include/**/*.hpp",
        "tracking/include/**/*.h",
    ]),
    copts = [
        "-fopenmp",
    ],
    strip_include_prefix = "tracking/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":filters",
        ":search",
        "@boost//:random",
    ],
)

cc_library(
    name = "io",
    srcs = [
        "io/src/ascii_io.cpp",
        "io/src/auto_io.cpp",
        "io/src/compression.cpp",
        "io/src/debayer.cpp",
        "io/src/file_io.cpp",
        "io/src/hdl_grabber.cpp",
        "io/src/ifs_io.cpp",
        "io/src/image_grabber.cpp",
        "io/src/io_exception.cpp",
        "io/src/lzf.cpp",
        "io/src/lzf_image_io.cpp",
        "io/src/obj_io.cpp",
        "io/src/pcd_grabber.cpp",
        "io/src/pcd_io.cpp",
        "io/src/ply/ply_parser.cpp",
        "io/src/ply_io.cpp",
        "io/src/robot_eye_grabber.cpp",
        "io/src/vlp_grabber.cpp",
        "io/src/vtk_io.cpp",
    ],
    hdrs = glob([
        "io/include/**/*.hpp",
        "io/include/**/*.h",
    ]),
    strip_include_prefix = "io/include",
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        ":octree",
        "@boost//:asio",
        "@boost//:filesystem",
        "@boost//:interprocess",
        "@boost//:iostreams",
        "@boost//:property_tree",
    ],
)

cc_binary(
    name = "test_search",
    srcs = [
        "apps/src/test_search.cpp",
    ],
    copts = [
        "-fopenmp",
    ],
    deps = [
        "io",
        "search",
    ],
)
