"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:deps.bzl", "isaac_http_archive")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for various modules
def isaac_assets_workspace():
    isaac_http_archive(
        name = "isaac_assets",
        url = "https://developer.nvidia.com/isaac/download/third_party/isaac_assets-zip",
        build_file = clean_dep("//third_party/assets:isaac_assets.BUILD"),
        type = "zip",
        licenses = ["http://docs.nvidia.com/cuda/eula/index.html"],
    )

    isaac_http_archive(
        name = "torch_inference_test_assets",
        build_file = clean_dep("//third_party/assets:torch_inference_test_assets.BUILD"),
        sha256 = "24e10fbb2aae938b9dcfbaa8ceb1fc65b31de33733159c23a1e1d3c545cb8322",
        url = "https://developer.nvidia.com/isaac/download/third_party/torch_inference_test_assets-v2-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "voice_command_detection_model_carter",
        build_file = clean_dep("//third_party/assets:voice_command_detection_model_carter.BUILD"),
        sha256 = "57e1b0f70136f7008b467d02eb97d8f09da45e85ca6a8cb442aca9ea2f3d7b55",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_carter_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "voice_command_detection_model_kaya",
        build_file = clean_dep("//third_party/assets:voice_command_detection_model_kaya.BUILD"),
        sha256 = "80a8251c81735c88573e17933f553da2aead04771fea2dee76348eddc85d426d",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_kaya_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "resnet_object_detection_model",
        build_file = clean_dep("//third_party/assets:resnet_object_detection_model.BUILD"),
        sha256 = "c15f5536062a755ffe8dd5ee7425c07f382849c2d5d0d5f1a6505d0904730473",
        url = "https://developer.nvidia.com/isaac/download/third_party/resnet18_detector_dolly_20191122-zip",
        type = "zip",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "tennis_ball_resnet_object_detection_model",
        build_file = clean_dep("//third_party/assets:tennis_ball_resnet_object_detection_model.BUILD"),
        sha256 = "baf27155dcfad51ddf892dc83005c5e592e2cf3f10e927c7429d7045b736c5ea",
        url = "https://developer.nvidia.com/isaac/download/third_party/kaya_tennis_ball_resnet18_v3-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_estimation_cnn_model",
        build_file = clean_dep("//third_party/assets:industrial_dolly_pose_estimation_cnn_model.BUILD"),
        sha256 = "727ccbd88377733d7f4656afba68477268ac35ba5acb3d04d2a062f12cdf45f6",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_perception_models_bldgK_fof_v6-tar-gz",
        type = "tar.gz",
        strip_prefix = "industrial_dolly_perception_models_bldgK_fof_v6",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_estimation_data",
        build_file = clean_dep("//third_party/assets:industrial_dolly_pose_estimation_data.BUILD"),
        sha256 = "aebeae9d79119e2f428b24ccf5655447db258052f4a238a345f3dbbeb43189dd",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_pose_estimation_data_v4-tar-gz",
        type = "tar.gz",
        strip_prefix = "industrial_dolly_pose_estimation_data_v4",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_evaluation_data",
        build_file = clean_dep("//third_party/assets:industrial_dolly_pose_evaluation_data.BUILD"),
        sha256 = "76efcc08d26c6d5e7698d877df3226f44cf130c3286d91d3c5be05b0936a6a39",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_pose_evaluation_data_v1-tar-gz",
        type = "tar.gz",
        strip_prefix = "industrial_dolly_pose_evaluation_data",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_refinement_data",
        build_file = clean_dep("//third_party/assets:industrial_dolly_pose_refinement_data.BUILD"),
        sha256 = "d5fe51c9867076a356180356566b2022be545240945107bbdbc6e538433dc1e7",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_pose_refinement_data_v1-tar-xz",
        type = "tar.xz",
        strip_prefix = "industrial_dolly_pose_refinement_data_v1",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "sortbot_pose_estimation_models",
        build_file = clean_dep("//third_party/assets:sortbot_pose_estimation_models.BUILD"),
        sha256 = "635d3ab33619c61b530555876e5ac6e1be0043b0ae91ba3a4b7359cac348e519",
        url = "https://developer.nvidia.com/isaac/download/third_party/sortbot_perception_models_v1-tar-gz",
        type = "tar.gz",
        strip_prefix = "sortbot_perception_models_v1",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "sortbot_pose_estimation_data",
        build_file = clean_dep("//third_party/assets:sortbot_pose_estimation_data.BUILD"),
        sha256 = "a3c7fd59152b886f476434527b5dd608b48add5a3b6aafa5311a2ffa205398c7",
        url = "https://developer.nvidia.com/isaac/download/third_party/sortbot_pose_estimation_data_v2-tar-gz",
        type = "tar.gz",
        strip_prefix = "sortbot_pose_estimation_data_v2",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "openpose_model",
        build_file = clean_dep("//third_party/assets:openpose_model.BUILD"),
        sha256 = "134ac4553d34edf61b5cb91c5db4c124f87ce7462de3a08823c7c4fbee21ce1d",
        url = "https://developer.nvidia.com/isaac/download/third_party/ix-networks-openpose-20190815-tar-xz",
        type = "tar.xz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "openpose_trt_pose_model",
        build_file = clean_dep("//third_party/assets:openpose_model.BUILD"),
        sha256 = "da2e33a16a5d792e0d2983ed26a78be86909d8673e0b9b2035bf5bc2841c59f6",
        url = "https://developer.nvidia.com/isaac/download/third_party/trt-pose-20191107-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/NVIDIA-AI-IOT/trt_pose/blob/master/LICENSE.md"],
    )

    isaac_http_archive(
        name = "mobilenetv2",
        build_file = clean_dep("//third_party/assets:mobilenetv2.BUILD"),
        sha256 = "a20d0c8d698502dc6a620528871c97a588885df7737556243a3412b39fce85e0",
        url = "https://developer.nvidia.com/isaac/download/third_party/mobilenetv2-1-4-224-tgz",
        type = "tgz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/models/master/research/slim/nets/mobilenet/mobilenet.py"],
    )

    isaac_http_archive(
        name = "mobilenetv2_onnx",
        build_file = clean_dep("//third_party/assets:mobilenetv2_onnx.BUILD"),
        sha256 = "8ce2930074b6025c141fcfee9e2c63bb7183f5f19e27695931ce763956cab098",
        url = "https://rdk-public.s3.amazonaws.com/test_data/mobilenetv2-1_0_onnx.tar.xz",
        type = "tar.xz",
        licenses = ["https://raw.githubusercontent.com/onnx/models/master/models/image_classification/mobilenet/README.md"],
    )

    isaac_http_archive(
        name = "ml_test_data",
        build_file = clean_dep("//third_party/assets:ml_test_data.BUILD"),
        sha256 = "2916fe0330ed1c2392148fe1ba8f8353ae3b694aa1c50d28d8f3df8f318ad57e",
        url = "https://developer.nvidia.com/isaac/download/third_party/ml_test_data_1_3-tar-xz",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "tacotron2_model",
        build_file = clean_dep("//third_party/assets:tacotron2_model.BUILD"),
        sha256 = "ffb88e4734700521925fec5926a4b29336b261368f1b02d2d61f5bb3f6d95d40",
        url = "https://developer.nvidia.com/isaac/download/third_party/tacotron2_streaming_fp32-v1-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "waveglow_model",
        build_file = clean_dep("//third_party/assets:waveglow_model.BUILD"),
        sha256 = "a3a08e91470f8870a56e4fc4ff6fe479c31797f8d846200958f77733fa1d6cbb",
        url = "https://developer.nvidia.com/isaac/download/third_party/waveglow_randVect_noiseTrunc_fp16-v0-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_images",
        build_file = clean_dep("//third_party/assets:path_segmentation_images.BUILD"),
        sha256 = "da9e7e16613bd480290c7491373922e8dde247f255eee201f583c82c601a453c",
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_images_2019_11_14-tar-xz",
        type = "tar.xz",
        strip_prefix = "path_segmentation_images_2019_11_14",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_logs",
        build_file = clean_dep("//third_party/assets:path_segmentation_logs.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_logs_2020_06_19-tar-xz",
        sha256 = "874d674563b5dc348508f744909d4e3acda133366d2d3ff909bfed3001ee38f0",
        type = "tar.xz",
        strip_prefix = "path_segmentation_logs_2020_06_19",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_pretrained_models",
        build_file = clean_dep("//third_party/assets:path_segmentation_pretrained_models.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_pretrained_models_2_2019_11_14-tar-xz",
        sha256 = "0278861657c710d48a9a1ae75378e67612e9f0cae1e1ef49d4c422593f5f3c96",
        type = "tar.xz",
        strip_prefix = "path_segmentation_pretrained_models",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "dolly_docking_reinforcement_learning_policy",
        build_file = clean_dep("//third_party/assets:dolly_docking_reinforcement_learning_policy.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_docking_frozen_v0-tar-xz",
        sha256 = "36211abde5948ecc94b1d3412b264d64376cf5f919cb6ee12f7fd5a58f35a4a6",
        type = "tar.xz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "dolly_detection_test_data",
        build_file = clean_dep("//third_party/assets:dolly_detection_test_data.BUILD"),
        sha256 = "e245dc734dd8ad609d4159b2db915a4d78941d2a9d3bd2a3b8947008b2e948fa",
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_detection_test_data_v1-tar-xz",
        type = "tar.xz",
        strip_prefix = "dolly_detection_test_data",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "sidewalk_segmentation_test_data",
        build_file = clean_dep("//third_party/assets:sidewalk_segmentation_test_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/sidewalk_segmentation_test_data_20191018-tar-xz",
        sha256 = "1d9c1f268d6b779d23251aea8ccadba4cf1882d4a4370edc01f1c478988ca888",
        type = "tar.xz",
        strip_prefix = "sidewalk_segmentation_test_data",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "crop_and_downsample_test_data",
        build_file = clean_dep("//third_party/assets:crop_and_downsample_test_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/crop_and_downsample_test_images_2019_11_20-tar-xz",
        sha256 = "0b7f834fd3be3ed93a823d0c6b480d75482eb3213d8d8b20c0d3776559cc3d91",
        type = "tar.xz",
        strip_prefix = "crop_and_downsample_test_images_2019_11_20",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "pose2_grid_graphs_factory",
        build_file = clean_dep("//third_party/assets:pose2_grid_graphs_factory.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/pose2_grid_graphs_factory-zip",
        sha256 = "d62e04c45458858ceed2b3c5217bdbeacb71f05eff2f16a4cb45eeec6774d1fa",
        type = "zip",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "dope_ycb_data",
        build_file = clean_dep("//third_party/assets:dope_ycb_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/dope_ycb_data_v2-tar-xz",
        sha256 = "0d4eacd2dff9f7bf520d3abd4cacb6f7a469184466c80e9864149c49da25579d",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "block_pose_estimation_data",
        build_file = clean_dep("//third_party/assets:block_pose_estimation_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/block_pose_estimation_data_v1-tar-gz",
        sha256 = "71ec3744b86f5ba3a9eecf8213bfb3751fc64eb54f162ff5c8d6ab0bf2fb7c60",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "jetbot_ball_detection_resnet_model",
        build_file = clean_dep("//third_party/assets:jetbot_ball_detection_resnet_model.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/jetbot_ball_detection_resnet18_v1-tar-xz",
        sha256 = "edf552db56fb4fabbb4e2aa3cec54ec5ce745f2a5e3a860acf754369e9573c76",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "detect_net_cvat_sample_data",
        build_file = clean_dep("//third_party/assets:detect_net_cvat_sample_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/detect_net_cvat_sample_v1-tar-xz",
        sha256 = "25db5f528af3e62dae171dda7068218eb08d8fb629285bcebc388de2704733ef",
        type = "tar.xz",
        strip_prefix = "detect_net_cvat_sample_v1",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "industrial_dolly_sortbot_sample_images",
        build_file = clean_dep("//third_party/assets:industrial_dolly_sortbot_sample_images.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_sortbot_sample_images_v1-tar-xz",
        sha256 = "e4119906653e110eb0bd392d62e7e7651d980c14796bac133e9195c4f6bd1229",
        type = "tar.xz",
        strip_prefix = "industrial_dolly_sortbot_sample_images_v1",
        licenses = ["//:LICENSE"],
    )
