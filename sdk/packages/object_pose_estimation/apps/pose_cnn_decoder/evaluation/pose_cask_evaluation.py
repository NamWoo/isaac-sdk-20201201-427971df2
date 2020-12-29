'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from isaac import Cask
from pathlib import Path
from matplotlib import pyplot as plt

import argparse
import datetime
import json
import matplotlib.animation as animation
import numpy as np
import os
import quaternion
import shutil
import uuid

from packages.cask.apps import multi_cask_processing
from pose_evaluation_utils import *


def get_image_cask_uuid_raci_metadata(roots):
    """
    Retrieve the image_cask_uuids for a list of logs from their associated metadata.
    Returns None if the metadata json files are not found to extract uuids.
    """
    image_cask_uuids = []
    for root in roots:
        metadata_path = root + "_md.json"
        metadata = {}
        if (not os.path.isfile(metadata_path)):
            return None
        with open(metadata_path) as f:
            metadata = json.load(f)
        image_cask_uuids.append(metadata["Image_Cask_File"])
    return image_cask_uuids


def make_clean_dir(path):
    """Clears a directory and creates it on disk.

    Args:
      path: The path on disk where the directory should be located.
    """

    # Clean up by deleting the directory if it already exists
    if os.path.exists(path):
        shutil.rmtree(path)
    # Create the directory
    os.makedirs(path)


def get_message_idx(cask, acqtime, start_idx):
    """
    Returns index of cask channel with matching timestamp. Returns -1, if the message was not found.
    cask - cask channel
    acqtime - acquisition timestamp to search for in cask
    start_idx - starting index to search from in the list of cask messages
    """

    for i in range(start_idx, len(cask)):
        if cask[i].acqtime == acqtime:
            return i
    # Return -1 if matching time stamp is not found.
    return -1


def main(args):
    """ The entry point of the application.
    The function reads the json file containing the ground truth and predicted pose labels,
    computes evaluation metrics and save the plots of translationa and rotation errors.
    """

    # Create uuid
    eval_uuid = uuid.uuid4()

    # ----------------------- LOAD CONFIG --------------------------------------
    # load evaluation config
    config = {}
    config_path = os.fspath(Path(args.config).resolve())
    with open(config_path) as f:
        config = json.load(f)

    # read cask paths
    image_cask_dir_path = args.image_cask_dir
    gt_cask_dir_path = args.gt_cask_dir
    pred_cask_dir_path = args.predicted_cask_dir

    image_cask_list = multi_cask_processing.load_roots(image_cask_dir_path, "")
    gt_poses_cask_list = multi_cask_processing.load_roots(gt_cask_dir_path, "")
    predicted_poses_cask_list = multi_cask_processing.load_roots(pred_cask_dir_path, "")

    pred_cask_root_list = [os.path.split(root)[0] for root in predicted_poses_cask_list]
    gt_cask_root_list = [os.path.split(root)[0] for root in gt_poses_cask_list]
    pred_image_cask_uuids = get_image_cask_uuid_raci_metadata(pred_cask_root_list)
    assert (pred_image_cask_uuids
            is not None), "Unable to find the metadata files for prediction casks."

    # Check for image cask uuids in the ground truth cask list (for simulation) or
    # roots of the cask list (for real data ground truth casks)
    gt_image_cask_uuids = get_image_cask_uuid_raci_metadata(gt_poses_cask_list)
    if (gt_image_cask_uuids is None):
        gt_image_cask_uuids = get_image_cask_uuid_raci_metadata(gt_cask_root_list)
    assert (gt_image_cask_uuids
            is not None), "Unable to find the metadata files for ground truth casks."

    # List of predicted poses
    images = []
    predicted_poses = []
    gt_poses = []
    predicted_detections = []

    # Read all the json objects into a string
    if (not (len(predicted_poses_cask_list) == len(gt_poses_cask_list)
             or not (len(predicted_poses_cask_list) == len(image_cask_list)))):
        raiseError("Length of image, ground truth and predicted labels list must be equal")

    # ------------------------- AGGREGATE IMAGE, PREDICTIONS, LABELS DATA --------------------
    for image_cask_path in image_cask_list:
        _, image_cask_tail = os.path.split(image_cask_path)

        try:
            # get associated gt cask and pred casks
            gt_cask_idx = gt_image_cask_uuids.index(image_cask_tail)
            pred_cask_idx = pred_image_cask_uuids.index(image_cask_tail)
        except (ValueError):
            print("Associated ground truth or predicted cask is not \
                available for image cask {}".format(image_cask_path))
            continue

        gt_cask_path = gt_poses_cask_list[gt_cask_idx]
        pred_cask_path = predicted_poses_cask_list[pred_cask_idx]

        #read image cask
        images_channel = Cask(image_cask_path)[config['image_channel']]

        #read ground truth cask
        gt_poses_channel = Cask(gt_cask_path)[config['gt_pose_channel']]

        # read predictions cask
        pred_poses_channel = Cask(pred_cask_path)[config['pred_pose_channel']]

        if (args.use_2d_detections):
            # read predcited detections cask if the channel is not empty
            pred_detections_channel = Cask(pred_cask_path)[config['pred_detection_channel']]

        # Reset indices before synching casks
        prev_image_idx = 0
        prev_gt_pose_idx = 0
        prev_pred_detection_idx = 0

        # ----------------------- SYNC CASKS --------------------------------------
        # Sync image, gt pose message, and predicted pose message channels by acqtime
        for prediction_pose in pred_poses_channel:
            #TODO:[Sravya] - Extend the evaluation to multiple objects in an image scenario
            if (len(prediction_pose.proto.poses) == 0 or len(prediction_pose.proto.poses) > 1):
                continue
            acqtime = prediction_pose.acqtime
            gt_pose_idx = get_message_idx(gt_poses_channel, acqtime, prev_gt_pose_idx)
            image_idx = get_message_idx(images_channel, acqtime, prev_image_idx)

            # Skip the message if the matching timestamp is not found in either casks
            if (image_idx != -1 and gt_pose_idx != -1):
                if (len(gt_poses_channel[gt_pose_idx].proto.poses) == 0):
                    continue
                images.append(images_channel[image_idx].tensor)
                gt_poses.append(pose_capnp_to_list(gt_poses_channel[gt_pose_idx].proto.poses[0]))
                predicted_poses.append(pose_capnp_to_list(prediction_pose.proto.poses[0]))
                prev_image_idx = image_idx
                prev_gt_pose_idx = gt_pose_idx

                if (args.use_2d_detections):
                    detection_idx = get_message_idx(pred_detections_channel, acqtime,
                                                    prev_pred_detection_idx)
                    prev_pred_detection_idx = detection_idx
                    # If detection sample is missing, add a zero size bbox instead of
                    # ignoring the sample.
                    if ((detection_idx < 0)
                            or (not pred_detections_channel[detection_idx].proto.boundingBoxes)):
                        predicted_detections.append([0, 0, 0, 0])
                        continue
                    bbox = pred_detections_channel[detection_idx].proto.boundingBoxes[0]
                    predicted_detections.append([bbox.min.x, bbox.min.y, bbox.max.x, bbox.max.y])

    # --------- Compute and plot evaluation metrics and outliers --------
    # Parse json objects and populate the pose statistics
    predicted_poses = np.asarray(predicted_poses)
    gt_poses = np.asarray(gt_poses)
    num_samples = gt_poses.shape[0]
    if (num_samples == 0):
        return
    if (args.use_2d_detections):
        predicted_detections = np.asarray(predicted_detections)

    # Setup folders to save eval metrics/plots.
    results_dir = args.results_dir
    make_clean_dir(results_dir)

    # Translation error along x, y a nd z axes in meters in the depth region of interest.
    translation_err = compute_roi_abs_translation_error(predicted_poses, gt_poses)
    # Rotation error between quaternions of ground truth and predicted pose (in deg)
    rotation_err = compute_roi_rotation_error(predicted_poses, gt_poses, config['symmetry_axis'],
                                              config['num_rotation_symmetry'])

    # ----------------------- COMPUTE AND OUTPUT KPIS --------------------------------------
    # Indices in the depth range of interest
    roi = np.where((gt_poses[:, 6] > config['depth_roi'][0])
                   & (gt_poses[:, 6] <= config['depth_roi'][1]))[0].tolist()
    # Median metrics in the region of interest in terms of distance from camera
    median_metrics = compute_median_pose_errors(rotation_err[roi], translation_err[roi, :])
    median_metrics = [round(elem, 3) for elem in median_metrics]
    translation_metrics_txt = "Median translation error in the depth roi [{}, {}] in m = {}\n".\
        format(config['depth_roi'][0], config['depth_roi'][1], median_metrics[1:])
    rotation_metrics_txt = "Median rotation error in the depth roi [{}, {}] = {} deg \n".\
        format(config['depth_roi'][0], config['depth_roi'][1], median_metrics[0])

    # Mean Precision in Pose Estimation metric
    # Reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6130367
    accuracy = compute_accuracy(translation_err[roi, :], rotation_err[roi],
                                config['translation_err_threshold'],
                                config['rotation_err_threshold'])
    accuracy = round(accuracy, 2)

    # Check if the KPIs meet the minimum thresholds and determine pass/fail
    kpi_pass = (accuracy >= config['KPI_accuracy_threshold']
                and median_metrics[0] <= config['rotation_err_threshold']
                and median_metrics[1] <= config['translation_err_threshold'][0]
                and median_metrics[2] <= config['translation_err_threshold'][1]
                and median_metrics[3] <= config['translation_err_threshold'][2])

    # Write KPI metrics to JSON file
    evaluation_report = config
    evaluation_report["translation_x_err_threshold"] = config['translation_err_threshold'][0]
    evaluation_report["translation_y_err_threshold"] = config['translation_err_threshold'][1]
    evaluation_report["translation_z_err_threshold"] = config['translation_err_threshold'][2]
    evaluation_report["date"] = str(datetime.datetime.now().strftime("%Y-%m-%d"))
    evaluation_report["num_frames"] = num_samples
    evaluation_report["KPI_roi_median_translation_x_error"] = median_metrics[1]
    evaluation_report["KPI_roi_median_translation_y_error"] = median_metrics[2]
    evaluation_report["KPI_roi_median_translation_z_error"] = median_metrics[3]
    evaluation_report["KPI_roi_median_rotation_error_deg"] = median_metrics[0]
    evaluation_report["KPI_roi_accuracy"] = accuracy
    evaluation_report["KPI_pass"] = bool(kpi_pass)

    # Write the JSON to file
    json_output_path = os.path.join(results_dir,
                                    "pose_estimation_evaluation_" + str(eval_uuid) + ".json")
    with open(json_output_path, 'w') as f:
        json.dump(evaluation_report, f, indent=2)

    # Save outlier images (above the error thresholds above) as animation
    # List of indices common across roi, translation and rotation positive indices
    positive_ind = (np.where((gt_poses[:, 6] > config['depth_roi'][0])
                             & (gt_poses[:, 6] <= config['depth_roi'][1])
                             & (translation_err[:, 0] < config['translation_err_threshold'][0])
                             & (translation_err[:, 1] < config['translation_err_threshold'][1])
                             & (translation_err[:, 2] < config['translation_err_threshold'][2])
                             & (rotation_err < config['rotation_err_threshold'])))[0].tolist()
    outlier_ind = [x for x in roi if x not in positive_ind]
    non_relevant_ind = [x for x in range(num_samples) if x not in outlier_ind]
    for index in sorted(non_relevant_ind, reverse=True):
        del images[index]
    camera_matrix = get_camera_matrix(config["camera_focal"], config["image_center"])
    save_animation_path = os.path.join("{}/{}.mp4".format(results_dir,
                                                          'outlier_images_with_predicted_poses'))
    for i in range(len(images)):
        # Update the image pixels with 3D bbox of the pose
        images[i] = visualize_3Dboundingbox(config["object_size"], config["object_center"],
                                            camera_matrix, predicted_poses[outlier_ind[i], :],
                                            images[i])
        if (args.use_2d_detections):
            images[i] = visualize_2Dboundingbox(predicted_detections[outlier_ind[i], :],
                                                images[i])
    # Save the outlier images with 2D and 3D predicted bounding boxes as animation
    save_animation_from_image_list(images, save_animation_path)

    # Box-whisker plot to analyze the distribution of errors at various camera distances)
    # Bins of errors based on distance from camera
    [rotation_err_bins, trans_err_x_bins, trans_err_y_bins, trans_err_z_bins,
     dist_values_bins] = compute_pose_error_depth_bins(rotation_err, translation_err, gt_poses,
                                                       config['box_plot_camera_distance_bins'])

    plt.figure()
    plt.boxplot(trans_err_x_bins)
    plt.ylim([0, config['translation_box_plot_y_limit'][0]])
    plt.title('Absolute of x position error for different camera distance intervals')
    plt.xlabel('Distance from camera in m')
    plt.ylabel('Absolute x position error in m')
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'box_plot_abs_x_err')))

    plt.figure()
    plt.boxplot(trans_err_y_bins)
    plt.ylim([0, config['translation_box_plot_y_limit'][1]])
    plt.title('Absolute of y position error for different camera distance intervals')
    plt.xlabel('Distance from camera in m')
    plt.ylabel('Absolute y position error in m')
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'box_plot_abs_y_err')))

    plt.figure()
    plt.boxplot(trans_err_z_bins)
    plt.ylim([0, config['translation_box_plot_y_limit'][2]])
    plt.title('Absolute of z error for different camera distance intervals')
    plt.xlabel('Distance from camera in m')
    plt.ylabel('Absolute z position error in m')
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'box_plot_abs_z_err')))

    plt.figure()
    plt.boxplot(rotation_err_bins)
    plt.ylim([0, config['rotation_box_plot_y_limit']])
    plt.title('Rotation error in deg for different camera distance intervals')
    plt.xlabel('distance from camera in m')
    plt.ylabel('rotation error angle in deg')
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'box_plot_rotation_err')))

    # Rotation angle threshold vs accuracy
    # Ref: PoseCNN: https://arxiv.org/pdf/1711.00199.pdf
    z_min = config['depth_roi'][0]
    z_max = config['depth_roi'][1]
    rot_threshold_bins = np.linspace(0, 180, 180)

    # Total number of samples within the camera distances range
    rotation_accuracy = []
    for rot_threshold in rot_threshold_bins:
        positive_ind = (np.where((gt_poses[:, 6] > z_min) & (gt_poses[:, 6] <= z_max)
                                 & (rotation_err < rot_threshold)))[0].tolist()
        rotation_accuracy.append(len(positive_ind) / len(roi))

    # Translation threshold vs accuracy
    # Ref: PoseCNN: https://arxiv.org/pdf/1711.00199.pdf
    # Plotting accuracy till translation error threshold of 30% body diameter
    body_diameter = np.linalg.norm(config['object_size'])
    trans_threshold_bins = np.linspace(0, 0.3 * body_diameter, 100)
    # Total number of samples within the camera distances range
    translation_accuracy = []
    for trans_threshold in trans_threshold_bins:
        positive_ind = (np.where((gt_poses[:, 6] > z_min) & (gt_poses[:, 6] <= z_max)
                                 & (translation_err[:, 0] < trans_threshold)
                                 & (translation_err[:, 1] < trans_threshold)
                                 & (translation_err[:, 2] < trans_threshold)))[0].tolist()
        translation_accuracy.append(len(positive_ind) / len(roi))

    # Plot of accuracy vs rotation threshold
    plt.figure()
    plt.plot(rot_threshold_bins, rotation_accuracy)
    plt.ylim([0, 1.0])
    plt.xlabel('Rotationangle threshold (in deg)')
    plt.ylabel('Accuracy')
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'rotation_accuracy')))

    # Plot of accuracy vs translation threshold
    plt.figure()
    plt.plot(trans_threshold_bins, translation_accuracy)
    plt.xlabel('Translation threshold (in m)')
    plt.ylabel('Accuracy')
    plt.ylim([0, 1.0])
    plt.savefig(os.path.join("{}/{}.png".format(results_dir, 'translation_accuracy')))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Pose Estimation Cask Evaluation')
    parser.add_argument(
        '--config',
        dest='config',
        default=
        "packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation/evaluation_config.json",
        help='Config file to load.')

    parser.add_argument(
        '--use_2d_detections',
        dest='use_2d_detections',
        default=True,
        help='If set to true, predicted 2D detections are assumed to be one of the channels of the \
            predicted poses cask. The detections are visualized for the outliers.')

    parser.add_argument(
        '--results_dir',
        dest='results_dir',
        default="/tmp/pose_evaluation_results",
        help=
        'Path to store the evaluation results. The directory is created if not already present.')

    parser.add_argument('--image_cask_dir',
                        dest='image_cask_dir',
                        default="/tmp/data/raw",
                        help='Path to the image cask directory')

    parser.add_argument('--gt_cask_dir',
                        dest='gt_cask_dir',
                        default="/tmp/data/ground_truth",
                        help='Path to the ground truth pose cask directory')

    parser.add_argument(
        '--predicted_cask_dir',
        dest='predicted_cask_dir',
        default="/tmp/data/predictions",
        help='Path to the predicted pose cask directory. It is expected to contain 2D detection\
            as well if use_2d_detections is set to True')

    args, _ = parser.parse_known_args()
    main(args)