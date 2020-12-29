'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
"""
Pose CNN Decoder training script.

Trains a neural network which takes in RGB images of different orientations of
an object and generates the segmentation mask of the object, translation and rotation parameters.
The network architecture used is based on AugmentedAutoencoder.

This script is setup to stream images from Isaac SDK for training.
"""

from isaac import *
from pathlib import Path
from PIL import Image

import gc
import json
import math
import numpy as np
import os
import packages.ml
import packages.object_pose_estimation.apps.pose_cnn_decoder.training
import quaternion
import shutil
import sys
import tensorflow as tf
import time

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"  # so the IDs match nvidia-smi
os.environ["CUDA_VISIBLE_DEVICES"] = "0"  # Limit the GPU usage to gpu #0

from tensorflow.python.client import timeline
from tensorflow.python.tools import freeze_graph

from packages.object_pose_estimation.apps.pose_cnn_decoder.evaluation.pose_evaluation_utils import *
from pose_estimation_cnn_utils import *

# Command line flags:
flags = tf.app.flags
FLAGS = flags.FLAGS
flags.DEFINE_string(
    'training_config_path',
    'packages/object_pose_estimation/apps/pose_cnn_decoder/training/training_config.json',
    'Relative path to the config file containing training configurations')

# Op names.
COLOR_IMAGE = 'rgb_image'
RECONSTRUCT_IMAGE = 'reconst_img'
INPUT_TRANSLATION = 'input_translation'
INPUT_ROTATION = 'input_rotation'
INPUT_DETECTION = 'input_detection'


def get_generator(bridge, config):
    """
    Create a training sample generator.

    Args:
      bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
      A generator function which yields a single training example.
    """
    def _generator():
        # Counters for offline training
        start_idx = 0
        end_idx = 0
        # Indefinitely yield samples. Requires the training scene to be running on navsim.
        while True:
            # Try to acquire a sample.
            if (config['online_training']):
                # If none are available, then wait for a bit so we do not spam the app.
                samples = bridge.acquire_samples(config['batch_size'])
            else:
                # Acquire samples from saved offline data directories
                start_idx = end_idx
                end_idx = start_idx + config['batch_size']
                samples = get_offline_dataset(config, start_idx, end_idx, training=True)

            if not samples:
                time.sleep(1)
                continue
            if len(samples[0][1]) == 0:
                continue

            for i in range(len(samples)):
                encoder_img = np.squeeze(samples[i][0])
                # Continue if it is empty tensor
                if (encoder_img.shape[0] == 0):
                    continue

                # Decoder image (input dimension - rows x cols)
                decoder_img = np.expand_dims(np.squeeze(samples[i][1]), axis=2)
                if (decoder_img.shape[0] == 0):
                    continue

                # Object center and depth labels
                # cx, cy in normalized pixel coordinates of the offset in image coordinate system
                # d is the depth in meters // distance from the camera
                object_center_and_depth = np.squeeze(samples[i][2])
                if (object_center_and_depth.shape[0] == 0):
                    continue
                if (object_center_and_depth.shape[0] != 3):
                    raise Exception("Object center and depth size should be 3!")

                # Input rotation as quaternions
                input_rotation = np.squeeze(samples[i][3])
                if (input_rotation.shape[0] != 4):
                    raise Exception("Object rotation input size should be 4!")
                input_rotation /= np.linalg.norm(input_rotation)
                # Handles rotational symmetries of the object only in the Z direction for now
                input_symm_rotations = np.zeros((config['num_rotation_symmetry'], 4))
                input_symm_rotations[0, :] = input_rotation
                if (config['rotation_loss'] == 'PLoss'):
                    # Changing order to q.x,q.y, q.z, q.w for using tensorflow's quaternion
                    # notation
                    input_symm_rotations[0, :] = quaternion_wxyz_to_xyzw_order(
                        input_symm_rotations[0, :])
                for i_rot_symm in range(1, config['num_rotation_symmetry']):
                    symm_rotation_angle = 2 * math.pi * i_rot_symm / config[
                        'num_rotation_symmetry']
                    rotation_z = quaternion.from_rotation_vector([0, 0, symm_rotation_angle])
                    quat_rotate = quaternion.from_float_array(input_rotation) * rotation_z
                    input_symm_rotations[i_rot_symm, :] = quaternion_to_numpyarray(quat_rotate)
                    if (config['rotation_loss'] == 'PLoss'):
                        # Changing order to q.x,q.y, q.z, q.w for using tensorflow's quaternion
                        # notation
                        input_symm_rotations[i_rot_symm, :] = quaternion_wxyz_to_xyzw_order(
                            input_symm_rotations[i_rot_symm, :])

                # Input detection: [bbox_size.x, bbox_size.y, bbox_center.x, bbox_center.y]
                # Detection values are normalized so that x pixel values are in range [-1, 1].
                input_detection = np.squeeze(samples[i][4])
                # Continue if there are no detections
                if (input_detection.shape[0] == 0):
                    continue
                if (input_detection.shape[0] != 4):
                    raise Exception("Detection size should be 4!")

                yield {
                    COLOR_IMAGE: encoder_img,
                    RECONSTRUCT_IMAGE: decoder_img,
                    INPUT_TRANSLATION: object_center_and_depth,
                    INPUT_ROTATION: input_symm_rotations,
                    INPUT_DETECTION: input_detection
                }

    return _generator


def quaternion_wxyz_to_xyzw_order(quat):
    """
    Changes to {q.x, q.y, q.z, q.w} quaternion order from Isaac's {q.w, q.x, q.y, q.z}
    """
    output_quaternion = [quat[1], quat[2], quat[3], quat[0]]
    return output_quaternion


def get_training_dataset(bridge, config):
    """Create a tf.data dataset which yields batches of samples for training.

    Args:
      bridge: the isaac sample accumulator node which we will acquire samples from

    Returns:
      A tf.data dataset which yields batches of training examples.
    """
    dataset = tf.data.Dataset.from_generator(
        get_generator(bridge, config), {
            COLOR_IMAGE: tf.float32,
            RECONSTRUCT_IMAGE: tf.float32,
            INPUT_TRANSLATION: tf.float32,
            INPUT_ROTATION: tf.float32,
            INPUT_DETECTION: tf.float32,
        }, {
            COLOR_IMAGE: tf.TensorShape([int(128), int(128), int(3)]),
            RECONSTRUCT_IMAGE: tf.TensorShape([int(128), int(128), int(1)]),
            INPUT_TRANSLATION: tf.TensorShape([int(3)]),
            INPUT_ROTATION: tf.TensorShape([int(config['num_rotation_symmetry']),
                                            int(4)]),
            INPUT_DETECTION: tf.TensorShape([int(4)]),
        })
    dataset = dataset.batch(batch_size=config['batch_size'])
    dataset = dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)
    return dataset


def get_offline_dataset(config, start_idx, end_idx, training=False):
    """
    Retrieves and returns batch_size number of data samples from a directory.

    Args:
      config: Dictionary of training configuration parameters
      start_idx: starting index of the data files of type int to retrieve the data batch
      end_idx: Ending index of the data files of type int to retrieve the data batch

    Returns:
      Tuple of image batch, detection input batch and ground truth pose label batch
    """
    if (training):
        data_dir = config['training_data_dir']
    else:
        data_dir = config['validation_data_dir']
    batch_size = end_idx - start_idx
    samples = []

    for i_data in range(start_idx, end_idx):
        if (training):
            # Infinitely loop through the data for training
            i_data %= config['num_training_data']
        sample = []
        # Expects images to be in 'images' sub-directory
        img = np.array(Image.open(os.path.join("{}/images/{}.png".format(data_dir, str(i_data)))),
                       np.float32)
        # Assumes [0, 255] range if pixel values are greater than 1 and rescales to [0, 1] range
        if np.max(np.max(img)) > 1.0:
            img = img / 255.0
        sample.append(img)
        if (training):
            sample.append(
                np.array(
                    Image.open(
                        os.path.join("{}/decoder_labels/{}.png".format(data_dir, str(i_data)))),
                    np.float32))
        # Expects the pose labels to be in 'pose_labels' sub-directory
        pose_label = np.loadtxt(os.path.join("{}/pose_labels/{}.txt".format(
            data_dir, str(i_data))),
                                delimiter='\n')
        # Splitting into translation and rotation vectors
        sample.append(pose_label[4:])
        sample.append(pose_label[:4])
        # Expects input detections to be in 'bbox_inputs' sub-directory
        sample.append(
            np.loadtxt(os.path.join("{}/bbox_inputs/{}.txt".format(data_dir, str(i_data))),
                       delimiter='\n'))
        samples.append(sample)

    return samples


def make_dir(path):
    """Clears a directory and creates it on disk.

    Args:
      path: The path on disk where the directory should be located.
    """

    # Catch an exception when trying to delete the path because it may not yet exist.
    try:
        shutil.rmtree(path)
    except:
        pass

    # Return if the directory already exists
    if os.path.exists(path):
        return
    # Create the directory. If it already exists, the error will get excepted
    os.makedirs(path)


def main():
    """ The entry point of the application.
    The function initializes the app, configures the settings, creates the network,
    trains and evaluates the model.
    """
    # Read the config json file
    config = {}
    config_path = os.fspath(Path(FLAGS.training_config_path).resolve())
    with open(config_path) as f:
        config = json.load(f)
    os.environ["CUDA_VISIBLE_DEVICES"] = config[
        'gpu_visible_device']  # Limit the GPU usage to specified

    app = None
    if (config['online_training']):
        # Create the application.
        app_filename = os.fspath(Path(config['app_filename']).resolve())
        app = Application(app_filename=app_filename)
        # Configure the application for the object spawned in sim
        robot_prefab = config['robot_prefab']
        scenario_mgr = app.nodes['data.simulation.scenario_manager'].components[
            'scenario_manager']
        # Select the simulation scene and object to spawn from configuration
        scenario_mgr.config.scene = config['scene']
        scenario_mgr.config.robot_prefab = config['robot_prefab']

        if (not config['decoding_full_object']):
            # Modify the graph to use the bounding box from encoder instead of decoder
            encoder_bounding_boxes = app.nodes['data.simulation.interface'].components['subgraph']
            encoder_crop = app.nodes['data.encoder_crop_downsample'].components[
                'ImageDetectionExtraction']
            app.connect(encoder_bounding_boxes, "encoder_bounding_boxes", encoder_crop,
                        "detections")
            decoder_crop = app.nodes['data.decoder_crop_downsample'].components[
                'ImageDetectionExtraction']
            app.connect(encoder_bounding_boxes, "encoder_bounding_boxes", decoder_crop,
                        "detections")
            detection_convertor = app.nodes['data.detection_convertor'].components[
                'BoundingBoxEncoder']
            app.connect(encoder_bounding_boxes, "encoder_bounding_boxes", detection_convertor,
                        "detection")

        # Startup the bridge to get data.
        node = app.nodes["pose_estimation_training_samples"]
        assert node is not None
        bridge = packages.ml.SampleAccumulator(node._node)
        app.start()
    else:
        # Offline training scenario
        bridge = None

    try:
        # Get the dataset batch
        dataset = get_training_dataset(bridge, config)
        iterator = dataset.make_one_shot_iterator()
        data_dict = iterator.get_next()
        # Encoder input image
        input_image = data_dict[COLOR_IMAGE]
        # Adding Gaussian noise to input image
        # This acts as data augmentation tool and also stabilizes the weights of the network
        # at the beginning of the training.
        if (config['add_noise_to_image']):
            noise = tf.random_normal(shape=tf.shape(input_image),
                                     mean=0.0,
                                     stddev=0.01,
                                     dtype=tf.float32)
            input_image = tf.add(input_image, noise)

        # Reconstructed image to compute decoder loss
        reconstruct_image = data_dict[RECONSTRUCT_IMAGE]
        if (config['add_noise_to_image']):
            noise = tf.random_normal(shape=tf.shape(reconstruct_image),
                                     mean=0.0,
                                     stddev=0.01,
                                     dtype=tf.float32)
            reconstruct_image = tf.add(reconstruct_image, noise)

        # Object poses
        input_rotation = data_dict[INPUT_ROTATION]
        input_translation = data_dict[INPUT_TRANSLATION]
        # Object Detection
        input_bbox = data_dict[INPUT_DETECTION]

        # Create the autoencoder network
        encoder = build_encoder(input_image,
                                config['latent_space_size'],
                                config['num_filter'],
                                config['kernel_size_encoder'],
                                config['strides'],
                                config['batch_norm'],
                                is_training=True)
        encoder_bbox = build_encoder_bbox(input_bbox,
                                          encoder,
                                          config['num_fc_layers_encoder_bbox'],
                                          config['batch_norm'],
                                          is_training=True)
        regression_translation = build_regression_translation(encoder_bbox,
                                                              input_translation,
                                                              config['num_fc_layers_translation'],
                                                              config['translation_loss'],
                                                              config['batch_norm'],
                                                              is_training=True)
        regression_rotation = build_regression_rotation(encoder_bbox,
                                                        input_rotation,
                                                        config['num_fc_layers_rotation'],
                                                        config['rotation_loss'],
                                                        config["quaternion_mag_loss_weight"],
                                                        config['batch_norm'],
                                                        is_training=True)
        decoder = build_decoder(reconstruct_image,
                                encoder_bbox,
                                config['num_filter'],
                                config['kernel_size_decoder'],
                                config['strides'],
                                config['decoder_loss'],
                                config['bootstrap_ratio'],
                                config['auxiliary_mask'],
                                config['batch_norm'],
                                is_training=True)
        pose_estimation_cnn = build_pose_estimation_cnn(encoder, encoder_bbox,
                                                        regression_translation,
                                                        regression_rotation, decoder,
                                                        config['norm_regularize'],
                                                        config['losses_weights'])
        train_op = build_train_op(pose_estimation_cnn, config['learning_rate'])
        global_step = tf.compat.v1.placeholder(tf.int32)

        # Setup logging summaries and checkpoints..
        tf.compat.v1.summary.image('Input_image', input_image, max_outputs=1)
        tf.compat.v1.summary.image('GT reconstruct_image', reconstruct_image, max_outputs=1)
        saver = tf.compat.v1.train.Saver()

        # Setup logging folders.
        ckpt_dir = os.path.join(Path(config['train_logdir']).resolve(), 'ckpts')
        ckpt_file = os.path.join(Path(ckpt_dir, 'model'))
        # Create directories if not loading from existing checkpoint
        if not config['checkpoint']:
            make_dir(ckpt_dir)

        # Create tensorflow session configuration
        session_config = tf.compat.v1.ConfigProto()
        session_config.gpu_options.per_process_gpu_memory_fraction = config['gpu_memory_usage']
        session_config.gpu_options.allow_growth = True

        # Wait until we get enough samples
        if (config['online_training']):
            while True:
                sample_number = bridge.get_sample_count()
                if sample_number >= config['batch_size']:
                    break
                time.sleep(config['sleep_duration'])
                print("waiting for samples samples: {}".format(sample_number))
        print("Starting training...")

        # Training loop.
        train_var_num = np.sum(
            [np.prod(v.get_shape().as_list()) for v in tf.compat.v1.trainable_variables()])
        with tf.compat.v1.Session(config=session_config) as sess:
            merged_loss_summary = tf.summary.merge_all()
            summary_writer = tf.summary.FileWriter(ckpt_dir, sess.graph)
            output_node_names = ",".join(
                ['translation_output', 'rotation_output_1', 'decoder_output'])
            sess.run(tf.global_variables_initializer())
            tf.train.write_graph(sess.graph, ckpt_dir, 'graph.pb', as_text=False)

            if config['checkpoint']:
                saver.restore(sess, os.path.join(ckpt_dir, config['checkpoint']))
                print("Checkpoint Loaded - {}".format(config['checkpoint']))

                # Assumes the number of steps is at the end of the
                # checkpoint file (separated by a '-')
                init_step = int(config['checkpoint'].split("-")[-1]) + 1
            else:
                init_step = 0

            if (config['validation']):
                # Paramters for validation
                num_valid_steps = int(
                    np.ceil(config['num_validation_data'] / config['batch_size']))
                if (num_valid_steps > 0):
                    predicted_poses = np.ones((config['num_validation_data'], 7))
                    gt_poses = np.ones((config['num_validation_data'], 7))
                    image_dim = [float(config['image_dim'][0]), float(config['image_dim'][1])]
                    pinhole_center = [
                        float(config['pinhole_center'][0]),
                        float(config['pinhole_center'][1])
                    ]
                    pinhole_focal = [
                        float(config['pinhole_focal'][0]),
                        float(config['pinhole_focal'][1])
                    ]
                    validation_set_median_metrics = []
                    validation_set_accuracy = []
                    median_metric_file = os.path.join("{}-{}-validation_median_errors.txt".format(
                        ckpt_file, init_step))
                    accuracy_file = os.path.join("{}-{}-validation_accuracy.txt".format(
                        ckpt_file, init_step))

            # Start training
            # Finalize the graph to make sure it is not modified during training
            for step in range(init_step, config['training_steps']):
                if step % config['summary_every'] == 0:
                    loss = sess.run(merged_loss_summary)
                    summary_writer.add_summary(loss, step)
                else:
                    sess.run(train_op)
                # Save checkpoint
                if step % config['save_every'] == 0 or step == config['training_steps'] - 1:
                    saver.save(sess, ckpt_file, global_step=step)
                # Save validation metrics if needed
                if (config['validation'] and (step % config['validation_every'] == 0
                                              or step == config['training_steps'] - 1)):
                    start_idx = 0
                    end_idx = 0
                    for valid_step in range(num_valid_steps):
                        start_idx = end_idx
                        end_idx = min(start_idx + config['validation_batch_size'],
                                      config['num_validation_data'])
                        # Run inference on a batch of validation data
                        samples = np.asarray(get_offline_dataset(config, start_idx, end_idx),
                                             dtype=object)
                        val_input_image = np.stack(samples[:, 0], axis=0)
                        val_input_bbox = np.stack(samples[:, 3], axis=0)
                        # Decode translation from object center and depth
                        gt_poses[start_idx:end_idx,
                                 4:] = decode_translation(np.stack(samples[:, 1],
                                                                   axis=0), pinhole_focal,
                                                          pinhole_center, image_dim)
                        # Rotation component of the pose
                        gt_poses[start_idx:end_idx + 1, :4] = np.stack(samples[:, 2], axis=0)

                        prediction = sess.run(
                            [regression_translation.output, regression_rotation.output],
                            feed_dict={
                                encoder.x: val_input_image,
                                encoder_bbox.input: val_input_bbox
                            })
                        tensor_proto = tf.compat.v1.make_tensor_proto(prediction[0])
                        predicted_center_depth = tf.make_ndarray(tensor_proto)
                        predicted_poses[start_idx:end_idx+1, 4:] = \
                            decode_translation(predicted_center_depth,
                                               pinhole_focal, pinhole_center, image_dim)
                        tensor_proto = tf.compat.v1.make_tensor_proto(prediction[1])
                        predicted_poses[start_idx:end_idx, :4] = tf.make_ndarray(tensor_proto)

                    # Numpy array of absolute error in translation in the depth region of interest
                    translation_error = compute_roi_abs_translation_error(
                        predicted_poses, gt_poses, depth_roi=config['depth_roi'])
                    # Numpy array of rotation error in the depth region of interest
                    rotation_error = compute_roi_rotation_error(predicted_poses,
                                                                gt_poses,
                                                                config['symmetry_axes'][0],
                                                                config['num_rotation_symmetry'],
                                                                depth_roi=config['depth_roi'])
                    # Compute the validation error metrics
                    median_metrics = compute_median_pose_errors(rotation_error, translation_error)
                    median_metrics = np.concatenate(([step], median_metrics))
                    validation_set_median_metrics.append(median_metrics)
                    accuracy = compute_accuracy(translation_error, rotation_error,
                                                config['translation_err_threshold'],
                                                config['rotation_err_threshold'])
                    validation_set_accuracy.append([step, accuracy])
                    np.savetxt(median_metric_file, validation_set_median_metrics)
                    np.savetxt(accuracy_file, validation_set_accuracy)
                gc.collect()

        # Saving model.pb file of the last iteration
        # Note: This should not be called inside another tf Session
        frozen_file = os.path.join("{}-{}-frozen.pb".format(ckpt_file,
                                                            config['training_steps'] - 1))
        freeze_graph.freeze_graph(input_graph=os.path.join(ckpt_dir, 'graph.pb'),
                                  input_saver="",
                                  input_binary=True,
                                  input_checkpoint=os.path.join("{}-{}".format(
                                      ckpt_file, config['training_steps'] - 1)),
                                  output_node_names=output_node_names,
                                  restore_op_name="save/restore_all",
                                  filename_tensor_name="save/Const:0",
                                  output_graph=frozen_file,
                                  clear_devices=True,
                                  initializer_nodes="")
        print("Saved frozen model at {}.".format(frozen_file))
        if (app is not None):
            app.stop()

    except KeyboardInterrupt:
        print("Exiting due to keyboard interrupt")
        if (app is not None):
            app.stop()


if __name__ == '__main__':
    main()
