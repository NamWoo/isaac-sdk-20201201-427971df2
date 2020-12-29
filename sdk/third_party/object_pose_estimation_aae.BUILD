"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# The archive is a forked version of the github source code
# https://github.com/DLR-RM/AugmentedAutoencoder.git by Sundermeyer.
# Minor modifications to the code involve adding identity layers, using
# tf.reshape in the last layer instead of tf.flatten to make it compatible
# for using TensorRT for inference and adding outputs from the model to
# tf.Summary for visualization.
py_binary(
    name = "py_aae",
    srcs = [
        "auto_pose/ae/ae.py",
        "auto_pose/ae/ae_factory.py",
        "auto_pose/ae/codebook.py",
        "auto_pose/ae/dataset.py",
        "auto_pose/ae/decoder.py",
        "auto_pose/ae/encoder.py",
        "auto_pose/ae/pysixd_stuff/transform.py",
        "auto_pose/ae/pysixd_stuff/view_sampler.py",
        "auto_pose/ae/queue.py",
        "auto_pose/ae/utils.py",
        "auto_pose/test/aae_retina_pose_estimator.py",
        "setup.py",
    ],
    main = "auto_pose/ae/ae.py",
    visibility = ["//visibility:public"],
)
