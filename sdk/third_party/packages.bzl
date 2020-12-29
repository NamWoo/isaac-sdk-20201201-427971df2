"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load(
    "@com_nvidia_isaac_engine//bzl:deps.bzl",
    "isaac_http_archive",
    "isaac_new_local_repository",
)

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for various modules
def isaac_packages_workspace():
    isaac_http_archive(
        name = "robotis",
        build_file = clean_dep("//third_party:dynamixel.BUILD"),
        sha256 = "1233525218b59ee9b923124ca688feab7014362c1c9c7ad4a844927f8ec3dba5",
        url = "https://developer.nvidia.com/isaac/download/third_party/robotis_dynamixel_sdk-3-6-2-tar-gz",
        type = "tar.gz",
        strip_prefix = "DynamixelSDK-3.6.2",
        licenses = ["@robotis//:LICENSE"],
    )

    isaac_http_archive(
        name = "assimp",
        build_file = clean_dep("//third_party:assimp.BUILD"),
        sha256 = "60080d8ab4daaab309f65b3cffd99f19eb1af8d05623fff469b9b652818e286e",
        url = "https://developer.nvidia.com/isaac/download/third_party/assimp-4-0-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "assimp-4.0.1",
        licenses = ["@assimp//:LICENSE"],
    )

    isaac_http_archive(
        name = "apriltags",
        build_file = clean_dep("//third_party:apriltags.BUILD"),
        sha256 = "fdd71f1cfef895b9a744b032e759b826874efeb0154452ef8866943083c31531",
        url = "https://developer.nvidia.com/isaac/download/third_party/april_tags_v5_jp44_nano-tar-xz",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "apriltags3",
        build_file = clean_dep("//third_party:apriltags3.BUILD"),
        sha256 = "2759b044ff1dc9ef725e7c456b49283399ef78deee24754bc3617cbe369584f1",
        url = "https://developer.nvidia.com/isaac/download/third_party/apriltag-3_1_2-tar-gz",
        type = "tar.gz",
        strip_prefix = "apriltag-3.1.2",
        licenses = ["https://github.com/AprilRobotics/apriltag/blob/master/LICENSE.md"],
    )

    isaac_http_archive(
        name = "glfw",
        build_file = clean_dep("//third_party:glfw.BUILD"),
        sha256 = "e10f0de1384d75e6fc210c53e91843f6110d6c4f3afbfb588130713c2f9d8fe8",
        url = "https://developer.nvidia.com/isaac/download/third_party/glfw-3-2-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "glfw-3.2.1",
        licenses = ["@glfw//:COPYING.txt"],
    )

    isaac_http_archive(
        name = "gl3w",
        build_file = clean_dep("//third_party:gl3w.BUILD"),
        sha256 = "442801ac9f10258499259b0b7679d28a1c1bf17e4a92c7e774b44bd4ba37525c",
        url = "https://developer.nvidia.com/isaac/download/third_party/gl3w-4f1d558410b0938840dc3db98e741d71f382ba22-tar-gz",
        type = "tar.gz",
        strip_prefix = "gl3w-4f1d558410b0938840dc3db98e741d71f382ba22",
        licenses = ["@gl3w//:UNLICENSE"],
    )

    isaac_http_archive(
        name = "imgui",
        build_file = clean_dep("//third_party:imgui.BUILD"),
        sha256 = "dc48173f9b34c763005e63dccb4355653909b25e04d5bc28ea9351540c457d96",
        url = "https://developer.nvidia.com/isaac/download/third_party/imgui-1-66-tar-gz",
        type = "tar.gz",
        strip_prefix = "imgui-1.66",
        licenses = ["@imgui//:LICENSE.txt"],
    )

    isaac_http_archive(
        name = "gmapping_repo",
        url = "https://developer.nvidia.com/isaac/download/third_party/gmapping-6f2ac5a2a2a8637ee844b4096f288f50d27a24cb-tar-gz",
        type = "tar.gz",
        sha256 = "6744340ce03976dc74a41d715a9087bccb4b273aecfb1f9aba18d370f8e635a0",
        licenses = ["https://openslam-org.github.io/gmapping.html"],
    )

    native.bind(
        name = "gmapping",
        actual = "@gmapping_repo//:gmapping",
    )

    isaac_http_archive(
        name = "libi2c_aarch64",
        build_file = clean_dep("//third_party:libi2c.BUILD"),
        sha256 = "0371eb3a1f60a5515f5571e5fc07711eb5d82f575060096fae09dcd0821b7d39",
        url = "https://developer.nvidia.com/isaac/download/third_party/libi2c-0-aarch64_xavier-tar-xz",
        type = "tar.xz",
        strip_prefix = "libi2c",
        licenses = ["https://raw.githubusercontent.com/amaork/libi2c/master/LICENSE"],
    )

    isaac_http_archive(
        name = "vrworks_warp360",
        build_file = clean_dep("//third_party:warp360.BUILD"),
        sha256 = "fb698cc6c3a2c12b3f88817240b74bd1cc81b3eb5269d6f8fcf24edd87f8332d",
        url = "https://developer.nvidia.com/isaac/download/third_party/vrworks_warp360_jp44-tar-xz",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "libargus",
        build_file = clean_dep("//third_party:libargus.BUILD"),
        sha256 = "8db8df094efb31a6c945e69001d6f9de3ccdbe8752d4069ef05c79c14ec0af5b",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-2019-02-tar-gz",
        type = "tar.gz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_http_archive(
        name = "vicon_datastream",
        build_file = clean_dep("//third_party:vicon_datastream.BUILD"),
        sha256 = "f8e0d88ad53a99e3ef4de21891781c664fb333a7e656967fd1d4230d7538371e",
        url = "https://developer.nvidia.com/isaac/download/third_party/vicon-datastream-sdk-tar-gz",
        type = "tar.gz",
        licenses = ["https://www.vicon.com/products/software/datastream-sdk"],
    )

    isaac_http_archive(
        name = "elbrus_vo",
        build_file = clean_dep("//third_party:elbrus_vo.BUILD"),
        sha256 = "bc89a9ac4041fd9040ffcddc7d147e5df83727273314e9e637c908bb6636e43d",
        url = "https://developer.nvidia.com/isaac/download/third_party/elbrus_v6_6c-tar-xz",
        type = "tar.xz",
        strip_prefix = "elbrus",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "kinova_jaco",
        build_file = clean_dep("//third_party:kinova_jaco.BUILD"),
        sha256 = "a8fa1a09ec98a69ab508176c35e582ed41abb63da430c73b8371940c68739fdd",
        url = "https://developer.nvidia.com/isaac/download/third_party/kinova_ros-1-2-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "kinova-ros-1.2.1",
        licenses = ["https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/master/LICENSE"],
    )

    isaac_http_archive(
        name = "realsense",
        build_file = clean_dep("//third_party:realsense.BUILD"),
        sha256 = "5dafabd13fe3ed23ae6c1f6c7f0c902de580f3a60a8b646e9868f7edc962abf2",
        url = "https://developer.nvidia.com/isaac/download/third_party/librealsense-v2-29-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "librealsense-2.29.0",
        licenses = ["@realsense//:LICENSE"],
    )

    isaac_http_archive(
        name = "opencv_x86_64",
        build_file = clean_dep("//third_party:opencv.BUILD"),
        sha256 = "9123df1c86180a920aea4469499aec372e4567e970eb0de7353c4e3e3eb30ec8",
        url = "https://developer.nvidia.com/isaac/download/third_party/libopencv_4_1_2_x86_64-tar-xz",
        type = "tar.xz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_http_archive(
        name = "opencv_aarch64_jetpack44",
        build_file = clean_dep("//third_party:opencv_jetpack42.BUILD"),
        sha256 = "e88582b4dd4b226f9eb17b9b9b616381ac20e75d2f62db3c05044e2f5d0d06dd",
        url = "https://developer.nvidia.com/isaac/download/third_party/libopencv_4_1_jetpack44-tar-xz",
        type = "tar.xz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_http_archive(
        name = "libtensorflow_x86_64",
        build_file = clean_dep("//third_party:libtensorflow_x86_64.BUILD"),
        sha256 = "8549d06347f503ac991ba58834a7b7513d3729a59a17f39dddd273cad9ff3665",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtensorflow-1-15-3_cuda_10-2_cudnn_8_x86-64-tar-gz",
        type = "tar.gz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    isaac_http_archive(
        name = "libtensorflow_aarch64_jetpack44",
        build_file = clean_dep("//third_party:libtensorflow_aarch64_jetpack44.BUILD"),
        sha256 = "6f047425ceb0c274097b9baa9531fae1cc1cc7d734fafbc9865445ab27d6828c",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorflow-1-15-2-jp44-ea-aarch64-tar-xz",
        type = "tar.xz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    # libtorch for x86_64
    isaac_http_archive(
        name = "libtorch_x86_64",
        build_file = clean_dep("//third_party:libtorch_x86_64.BUILD"),
        sha256 = "886e1e046bea5866eaa4551acdf21b79e498da690c4458bff5107ef27cf01327",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_1_6_0_cuda_10_2_cudnn_8_capabilities_x86_64-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    # libtorch for aarch64_jetpack42
    isaac_http_archive(
        name = "libtorch_aarch64_jetpack44",
        build_file = clean_dep("//third_party:libtorch_aarch64_jetpack42.BUILD"),
        sha256 = "252093e4a791c3da38f0a1b55be644f1d67d6c33d7bfd1fa72f63ec07b8cd7ea",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_1_6_0_jp44-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    # Source: TensorRT-7.1.3.4.Ubuntu-18.04.x86_64-gnu.cuda-10.2.cudnn8.0.tar.gz
    isaac_http_archive(
        name = "tensorrt_x86_64",
        build_file = clean_dep("//third_party:tensorrt_x86_64.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt-7-1-3-cuda10-2_x86_64-tar-xz",
        sha256 = "b99556ae21582faa19e85f2486522abf9677a932b2c03cf3e0326459f676f256",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    # Source: SDKManager/JetPack_SDKs/4.3/L4T/78_19316_27599411/JETPACK_43_b78/NoDLA
    isaac_http_archive(
        name = "tensorrt_aarch64_jetpack44",
        build_file = clean_dep("//third_party:tensorrt_jetpack44.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_7_1_3_jp44_ga-tar-gz",
        sha256 = "9d9566f39d13afd95eda1d0297940084bc93e836abc475f6efb77490687f215e",
        type = "tar.gz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    # Source: SDKManager/JetPack_SDKs/4.3/L4T/78_19316_27599411/JETPACK_43_b78/DLA
    isaac_http_archive(
        name = "tensorrt_aarch64_jetpack44_dla",
        build_file = clean_dep("//third_party:tensorrt_jetpack44.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_7-1-0-4-1+cuda10-2_arm64-jetpack_441_b14-dla-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    isaac_http_archive(
        name = "libargus_aarch64_nano",
        build_file = clean_dep("//third_party:libargus_nano.BUILD"),
        sha256 = "96c122c248efe01fece983cb5a6f46dc9eed707b7954a9f30437252c77e0c4f5",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-jetpack44-tar-gz",
        type = "tgz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_http_archive(
        name = "hgmm_impl",
        sha256 = "310a5a76a382a064216411d35b9b7603135e55b4661e04b89518b4e255a61d20",
        url = "https://developer.nvidia.com/isaac/download/third_party/libhgmm_impl_jp44-tar-xz",
        build_file = clean_dep("//third_party:libhgmm_impl.BUILD"),
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "livox_sdk",
        sha256 = "1c62b3f85a548183100cc94730926b64df2feeb10883f7d3bd245708ef0340a5",
        url = "https://developer.nvidia.com/isaac/download/third_party/Livox-SDK-1-0-0-tar-gz",
        build_file = clean_dep("//third_party:livox_sdk.BUILD"),
        type = "tar.gz",
        strip_prefix = "Livox-SDK-1.0.0",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "lfll",
        url = "https://developer.nvidia.com/isaac/download/third_party/lfll-9d29453368c432e373acf712d51515505ef057b0-tar-gz",
        type = "tar.gz",
        sha256 = "716a1a786612cbce8b3e7ef69b42d7f96a071a0f9974392c193875ea1621eff3",
        build_file = clean_dep("//third_party:lfll.BUILD"),
        patches = [clean_dep("//third_party:lfll.patch")],
        licenses = ["https://github.com/nicopauss/LFLL/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "efll",
        url = "https://developer.nvidia.com/isaac/download/third_party/efll-640b8680b6535768f318172b0a28a5e4091d8f60-tar-gz",
        type = "tar.gz",
        sha256 = "0e4ab56295c3efb4ddd3611505d1a63c067dd79da8f41d71a9b5017275dbb924",
        build_file = clean_dep("//third_party:efll.BUILD"),
        licenses = ["https://github.com/zerokol/eFLL/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "tlt_parser",
        build_file = clean_dep("//third_party:tlt_parser.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/tlt_parser_20191121-zip",
        sha256 = "1205669b58d78a93b9f5da5656682ffd2b7146014d7501448638d5c1f08ab1a7",
        type = "zip",
        strip_prefix = "tlt_parser",
        licenses = ["//:LICENSE"],
    )

    # GStreamer is a pipeline-based multimedia framework that links together a wide variety of
    # media processing systems. It is a common Linux system component. The host system provides
    # access to the libraries with a variety of licenses depending on your usage. You should
    # review licenses per your usage of GStreamer components.
    isaac_http_archive(
        name = "gstreamer",
        build_file = clean_dep("//third_party:gstreamer.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/gstreamer-1-14-5-jetpack44-tar-xz",
        sha256 = "fc2c18a50f6d13a68f942d3d0c899a3fbf01ecdb809e5f3009b16369a5ef0157",
        type = "tar.xz",
        strip_prefix = "gstreamer-1.14.5",
        licenses = [
            "@gstreamer//:src/gstreamer/COPYING",
            "@gstreamer//:src/gst-plugins-base/COPYING",
        ],
    )

    # GLib is a set of low-level libraries useful for providing data structure handling for C,
    # portability wrappers, execution loops, and interfaces. It is a common Linux system
    # component.
    isaac_http_archive(
        name = "glib",
        build_file = clean_dep("//third_party:glib.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/glib-2-56-4-jetpack44-tar-xz",
        sha256 = "1587db1e59fe46fd17734ee17471382342d1414d05d4d53421f844dadb7e83ad",
        type = "tar.xz",
        strip_prefix = "glib-2.56.4",
        licenses = ["@glib//:src/COPYING"],
    )

    # Point Cloud Library http://pointclouds.org/ (BSD License)
    isaac_http_archive(
        name = "pcl",
        build_file = clean_dep("//third_party:pcl.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/pcl-1-9-1-tar-xz",
        sha256 = "dc78b9561aa4ba315cb37ad4e855fc3349ad8746b2e9cbecb7f15a2b03901584",
        type = "tar.xz",
        strip_prefix = "pcl-pcl-1.9.1",
        licenses = ["@pcl//:LICENSE.txt"],
    )

    isaac_http_archive(
        name = "flann",
        build_file = clean_dep("//third_party:flann.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/flann-06a49513138009d19a1f4e0ace67fbff13270c69-tar-gz",
        type = "tar.gz",
        sha256 = "a0da614ba918dc536f84173e7427420cb229a47144e695c47132d5202a9c9bf0",
        patches = [clean_dep("//third_party:flann.patch")],
        licenses = ["//:COPYING"],
    )

    # Deps for USD
    isaac_http_archive(
        name = "open_subdiv",
        build_file = clean_dep("//third_party:open_subdiv.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/OpenSubdiv-3_4_0-tar-gz",
        sha256 = "d932b292f83371c7518960b2135c7a5b931efb43cdd8720e0b27268a698973e4",
        type = "tar.gz",
        licenses = ["@open_subdiv//:LICENSE.txt"],
    )

    isaac_http_archive(
        name = "lcm",
        build_file = clean_dep("//third_party:lcm.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/lcm-1-4-0-tar-xz",
        sha256 = "2a96753c31b77b9d0afbaa40863974e641223505cd704449e8470b1dc4167b8d",
        type = "tar.xz",
        strip_prefix = "lcm-1.4.0",
        licenses = ["//:COPYING"],
    )

    isaac_http_archive(
        name = "laikago_sdk",
        build_file = clean_dep("//third_party:laikago_sdk.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/laikago_sdk_V1-1-3-tar-xz",
        sha256 = "2e7d3c15262c7b72f5ea75ed66e7b224cc2c373986ea37d730d1a90cdf5eebd3",
        type = "tar.xz",
        strip_prefix = "laikago_sdk",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "pybind11",
        build_file = clean_dep("//third_party:pybind11.BUILD"),
        sha256 = "97504db65640570f32d3fdf701c25a340c8643037c3b69aec469c10c93dc8504",
        url = "https://developer.nvidia.com/isaac/download/third_party/pybind11-2-5-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "pybind11-2.5.0",
        licenses = ["@pybind11//:LICENSE"],
    )

    isaac_new_local_repository(
        name = "python",
        build_file = clean_dep("//third_party:python.BUILD"),
        path = "/usr",
        licenses = ["https://docs.python.org/3/license.html"],
    )

    isaac_http_archive(
        name = "python_aarch64",
        build_file = clean_dep("//third_party:python_aarch64.BUILD"),
        sha256 = "0557f47820f90d0dc9371c991ecb361f0e5fbac753a416a80ed4ee7ad80906e6",
        url = "https://developer.nvidia.com/isaac/download/third_party/python3_aarch64_jp42-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.python.org/3/license.html"],
    )

    isaac_http_archive(
        name = "yaml-cpp",
        build_file = clean_dep("//third_party:yaml-cpp.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/yaml-cpp-yaml-cpp-0-6-3-zip",
        sha256 = "7c0ddc08a99655508ae110ba48726c67e4a10b290c214aed866ce4bbcbe3e84c",
        type = "zip",
        strip_prefix = "yaml-cpp-yaml-cpp-0.6.3",
        licenses = ["@yaml-cpp//:LICENSE"],
    )

    isaac_http_archive(
        name = "lula_x86_64",
        build_file = clean_dep("//third_party:lula.BUILD"),
        sha256 = "14aa2f193fd04780ffa0707955a769ead160fb15815488f6d6c2d8e0eae9b124",
        url = "https://developer.nvidia.com/isaac/download/third_party/lula-0-3-0-x86_64_with_deps-tar-gz",
        type = "tar.gz",
        strip_prefix = "lula-0.3.0-x86_64_with_deps",
        licenses = ["@lula_x86_64//:LICENSE"],
    )

    isaac_http_archive(
        name = "lula_aarch64",
        build_file = clean_dep("//third_party:lula.BUILD"),
        sha256 = "400f619259dccdcb5de3fdfc51c3b8c6fc78be3a68bd9028fa430b494cdbf2bd",
        url = "https://developer.nvidia.com/isaac/download/third_party/lula-0-3-0-aarch64_with_deps-tar-gz",
        type = "tar.gz",
        strip_prefix = "lula-0.3.0-aarch64_with_deps",
        licenses = ["@lula_aarch64//:LICENSE"],
    )
