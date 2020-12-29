'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import math
import numpy as np
import tensorflow as tf

from decoder import Decoder
from encoder import Encoder
from encoder_bbox import EncoderBbox
from pose_estimation_cnn import PoseEstimationCNN
from regression_rotation import RegressionRotation
from regression_translation import RegressionTranslation


def quaternion_to_numpyarray(q):
    """
    Returns the numpy Quaternion as numpy array
    """
    return np.array([q.w, q.x, q.y, q.z])


def decode_translation(center_depth, pinhole_focal, pinhole_center, image_dim):
    """
    Decodes translation output of the tensorflow model to translation vector (x, y, z) in metres.

    Args:
        center_depth: Center and depth of the object - translation output from the model
        pinhole_focal: Tuple of focal length of camera in pixels along x and y axes
        pinhole_center: Tuple of pinhole center of camera in pixels along x and y axes
        image_dim: Tuple of the image dimensions [row, cols]
    """
    translation = center_depth
    # Get back object center to pixel coordinates (undo the pixel coordinate scaling)
    pixel = center_depth[:, :2] * 0.5 * image_dim[0] + 0.5 * np.asarray(image_dim)
    translation[:, :2] =  np.multiply(np.transpose(np.tile(center_depth[:, 2], (2,1))), \
        (pixel - pinhole_center)/pinhole_focal)
    # Axes swap to align with Isaac's coordinate system
    translation[:, [0, 1]] = translation[:, [1, 0]]
    return translation


def build_encoder(x,
                  latent_space_size,
                  num_filter,
                  kernel_size_encoder,
                  strides,
                  batch_norm,
                  is_training=True):
    """
    Builds encoder that encode the input color image
    """
    encoder = Encoder(x,
                      latent_space_size,
                      num_filter,
                      kernel_size_encoder,
                      strides,
                      batch_norm,
                      is_training=is_training)
    return encoder


def build_encoder_bbox(input_bbox,
                       encoder_image,
                       num_fc_layers_encoder_bbox,
                       batch_norm,
                       is_training=True):
    """
    Builds encoder_bbox that encodes bounding box parameters and concatenates
    the latent code of the encoder image with the latent code of the bounding box params.
    """
    encoder_bbox = EncoderBbox(input_bbox,
                               encoder_image._z,
                               num_fc_layers_encoder_bbox,
                               batch_norm,
                               is_training=is_training)
    return encoder_bbox


def build_regression_rotation(encoder_bbox,
                              input_rotation,
                              num_fc_layers_rotation,
                              loss,
                              quaternion_mag_loss_weight,
                              batch_norm,
                              is_training=True):
    """
    Builds regression network that decodes latent vector to rotation of the object in quaternion.
    """
    regression_rotation = RegressionRotation(encoder_bbox._concat_latent_code,
                                             input_rotation,
                                             num_fc_layers_rotation,
                                             loss,
                                             batch_norm,
                                             quaternion_mag_loss_weight,
                                             is_training=is_training)
    return regression_rotation


def build_regression_translation(encoder_bbox,
                                 input_translation,
                                 num_fc_layers_translation,
                                 loss,
                                 batch_norm,
                                 is_training=True):
    """
    Builds regression network that decodes latent vector to object center and depth.
    """
    regression_translation = RegressionTranslation(encoder_bbox._concat_latent_code,
                                                   input_translation,
                                                   num_fc_layers_translation,
                                                   loss,
                                                   batch_norm,
                                                   is_training=is_training)
    return regression_translation


def build_decoder(reconstruction_target,
                  latent_code,
                  num_filter,
                  kernel_size_decoder,
                  strides,
                  loss,
                  bootstrap_ratio,
                  auxiliary_mask,
                  batch_norm,
                  is_training=True):
    """
    Builds CNN network that decodes the segmentation image from latent code.

    Latent_code is the output from encoder_bbox.
    Thus, the code supports two different neural network architectures for pose prediction,
    one where decoder learns to construct image from fused latent code of input image and bbox,
    second where decoder learns to construct image from translation and rotation parameter
    predictions of the regression layers.
    """
    decoder = Decoder(reconstruction_target,
                      latent_code._concat_latent_code,
                      list(reversed(num_filter)),
                      kernel_size_decoder,
                      list(reversed(strides)),
                      loss,
                      bootstrap_ratio,
                      auxiliary_mask,
                      batch_norm,
                      is_training=is_training)
    return decoder


def build_pose_estimation_cnn(encoder, encoder_bbox, regression_translation, regression_rotation,
                              decoder, norm_regularize, losses_weights):
    """
    Builds the full 3D pose estimation model and computes the total weighted loss
    """
    pose_estimation_cnn = PoseEstimationCNN(encoder, encoder_bbox, regression_translation,
                                            regression_rotation, decoder, norm_regularize,
                                            losses_weights)
    return pose_estimation_cnn


def build_train_op(pose_estimation_cnn, learning_rate, optimizer_name="Adam"):
    """
    Sets up the training operation with SGD optimizer
    """
    optimizer = eval('tf.train.{}Optimizer'.format(optimizer_name))
    optim = optimizer(learning_rate)
    train_op = tf.contrib.training.create_train_op(pose_estimation_cnn._loss,
                                                   optim,
                                                   global_step=pose_estimation_cnn._global_step)
    return train_op
