'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

'''

from isaac import Cask
from pathlib import Path

import argparse
import json
import os
import numpy as np
import datetime
import uuid

from packages.cask.apps import multi_cask_processing
from detect_net_evaluation_utils import match_bboxes, matches_to_confusion_matrix
from detect_net_evaluation_utils import plot_bboxes, save_sample
"""
Object detection evaluation application. Reads and sycs multiple sets of image, ground truth,
and predictions casks to computes various metrics for evaluation. Each cask has an associated
metadata in accordance with RACI Workflow design. Outliers are found based on configuarble
parameters and can be visualized. The results of the evaluation areoutput to a JSON file report
at a location specified by the user.
"""


def get_image_cask_uuid_raci_metadata(roots):
    """
    Retrieve the image_cask_uuids for a list of logs from their associated metadata.
    """
    image_cask_uuids = []
    for root in roots:
        metadata_path = root + "_md.json"
        metadata = {}
        with open(metadata_path) as f:
            metadata = json.load(f)
        image_cask_uuids.append(metadata["Image_Cask_File"])
    return image_cask_uuids


def get_message_idx(cask_channel, acqtime, start_idx):
    """
    Returns index of cask channel with matching timestamp. Returns -1, if the message was not found.
    cask_channel - cask channel
    acqtime - acquisition timestamp to search for in cask
    start_idx - starting index to search from in the list of cask messages
    """

    for i in range(start_idx, len(cask_channel)):
        if cask_channel[i].acqtime == acqtime:
            break
    if i == len(cask_channel) or cask_channel[i].acqtime != acqtime:
        return -1
    return i


def main(args):
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
    pred_cask_dir_path = args.pred_cask_dir
    results_dir = args.results_dir

    image_cask_list = multi_cask_processing.load_roots(image_cask_dir_path, "")
    gt_cask_list = multi_cask_processing.load_roots(gt_cask_dir_path, "")
    pred_cask_list = multi_cask_processing.load_roots(pred_cask_dir_path, "")
    pred_cask_root_list = [os.path.split(root)[0] for root in pred_cask_list]

    gt_image_cask_uuids = get_image_cask_uuid_raci_metadata(gt_cask_list)
    pred_image_cask_uuids = get_image_cask_uuid_raci_metadata(pred_cask_root_list)

    # Image, ground truth, and prediction data for full dataset
    images = []
    gt_detections = []
    pred_detections = []

    # load other evaluation config params
    iou_thresholds = config['iou_thresholds']
    labels = config['labels']
    num_labels = len(labels)
    num_thresholds = len(iou_thresholds)

    # ----------------------- READ DATASET FROM SET OF CASKS ---------------------
    for image_cask_path in image_cask_list:
        _, image_cask_tail = os.path.split(image_cask_path)

        gt_cask_idx = None
        pred_cask_idx = None

        try:
            # get associated gt cask and pred casks
            gt_cask_idx = gt_image_cask_uuids.index(image_cask_tail)
            pred_cask_idx = pred_image_cask_uuids.index(image_cask_tail)
        except (ValueError):
            print("Associated ground truth or predicted cask is not \
                available for image cask {}".format(image_cask_path))
            continue

        gt_cask_path = gt_cask_list[gt_cask_idx]
        pred_cask_path = pred_cask_list[pred_cask_idx]

        #read image cask
        image_cask = Cask(image_cask_path)
        images_channel = image_cask[config['image_channel']]

        #read ground truth cask
        gt_cask = Cask(gt_cask_path)
        gt_detections_channel = gt_cask[config['gt_channel']]

        # read predictions cask
        pred_cask = Cask(pred_cask_path)
        pred_detections_channel = pred_cask[config['pred_channel']]

        # Sync image, gt detection message, and predicted detection message channels by acqtime
        prev_image_idx = 0
        prev_gt_idx = 0
        for prediction in pred_detections_channel:
            image_idx = get_message_idx(images_channel, prediction.acqtime, prev_image_idx)
            gt_idx = get_message_idx(gt_detections_channel, prediction.acqtime, prev_gt_idx)
            if (image_idx != -1 and gt_idx != -1):
                images.append(images_channel[image_idx])
                gt_detections.append(gt_detections_channel[gt_idx])
                pred_detections.append(prediction)
                prev_image_idx = image_idx
                prev_gt_idx = gt_idx

    num_samples = len(images)    #also same as len(gt_detections) and len(pred_detections)

    # ----------------------- COMPUTE METRICS --------------------------------------
    # use bboxes matched with specified IOU threshold to determine outliers
    outlier_iou_area_threshold = config['outlier_iou_area_threshold']

    # matches with IOU under the area min have large bbox error
    large_bbox_iou_area_min = config['large_bbox_iou_area_min']

    # Lists to collect outlier indices
    large_bbox_outliers_idxs = []
    false_positive_idxs = []
    false_negative_idxs = []

    # Lists to collect (and later average) area/height/width IOUs
    all_ious_area = []
    all_ious_height = []
    all_ious_width = []

    # Compute confusion matrices: one per IOU threshold
    confusion_matrices = {iou_threshold: np.empty((0)) for iou_threshold in iou_thresholds}
    for idx in range(num_samples):
        gt_bboxes = gt_detections[idx].proto.boundingBoxes
        gt_predictions = gt_detections[idx].proto.predictions
        pred_bboxes = pred_detections[idx].proto.boundingBoxes
        pred_predictions = pred_detections[idx].proto.predictions

        # match gt bboxes with predicted bboxes for each IOU
        for iou_threshold in iou_thresholds:
            gt_matched_idxs, pred_matched_idxs, matched_ious_width, matched_ious_height, \
                matched_ious_area = match_bboxes(gt_bboxes, pred_bboxes, iou_threshold)
            all_ious_width += matched_ious_width
            all_ious_height += matched_ious_height
            all_ious_area += matched_ious_area

            # collect outliers with large bbox error
            if iou_threshold == outlier_iou_area_threshold and not all(
                    i > large_bbox_iou_area_min for i in matched_ious_area):
                large_bbox_outliers_idxs.append(idx)

            # compute confusion matrices
            confusion_matrix_sample = matches_to_confusion_matrix(
                gt_matched_idxs, pred_matched_idxs, gt_predictions, pred_predictions, labels)

            # collect false positive idxs
            if iou_threshold == outlier_iou_area_threshold and len(pred_bboxes) != len(
                    pred_matched_idxs):
                false_positive_idxs.append(idx)

            # collect false negative idxs
            if iou_threshold == outlier_iou_area_threshold and len(gt_bboxes) != len(
                    gt_matched_idxs):
                false_negative_idxs.append(idx)

            # accumulate confusion matrices across samples
            if not confusion_matrices[iou_threshold].any():
                confusion_matrices[iou_threshold] = confusion_matrix_sample
            else:
                confusion_matrices[iou_threshold] += confusion_matrix_sample

    # Compute precision and recall metrics for each class and each IoU from the confusion matrix
    per_class_precisions = np.zeros((num_labels, num_thresholds))
    per_class_recalls = np.zeros((num_labels, num_thresholds))
    for iou_idx in range(num_thresholds):
        confusion_matrix = confusion_matrices[iou_thresholds[iou_idx]]
        precisions = np.diagonal(confusion_matrix) / np.sum(confusion_matrix, axis=0)
        recalls = np.diagonal(confusion_matrix) / np.sum(confusion_matrix, axis=1)
        per_class_precisions[:, iou_idx] = precisions[:-1]
        per_class_recalls[:, iou_idx] = recalls[:-1]

    # compute average IOUs for area as well as height and width dimensions
    mean_iou_width = sum(all_ious_width) / len(all_ious_width)
    mean_iou_height = sum(all_ious_height) / len(all_ious_height)
    mean_iou_area = sum(all_ious_area) / len(all_ious_area)

    # ----------------------- COMPUTE KPIS FROM METRICS  ---------------------------

    # COCO 2017 challenge mAP is computed over all IOUs over all classes
    # https://cocodataset.org/#detection-eval
    KPI_mAP = np.mean(per_class_precisions)
    KPI_mAR = np.mean(per_class_recalls)

    # PASCAL VOC2007 challenge mAP is computed =over a single IOU over all classes
    # http://host.robots.ox.ac.uk/pascal/VOC/
    mAP_per_IOU = np.mean(per_class_precisions, axis=0)
    mAR_per_IOU = np.mean(per_class_recalls, axis=0)
    KPI_mAP_lowest_IOU = mAP_per_IOU[0]
    KPI_mAR_lowest_IOU = mAR_per_IOU[0]

    # Check if the KPIs meet the minimum thresholds and determine pass/fail
    kpi_pass = (KPI_mAP >= config["KPI_threshold_mAP"] and KPI_mAR >= config["KPI_threshold_mAR"] \
        and KPI_mAP_lowest_IOU >= config["KPI_threshold_mAP_lowest_IOU"] \
        and KPI_mAR_lowest_IOU >= config["KPI_threshold_mAR_lowest_IOU"])

    # ----------------------- OUTPUT ALL RESULTS --------------------------------------
    evaluation_report = config
    evaluation_report["date"] = str(datetime.datetime.now().strftime("%Y-%m-%d"))

    # Construct JSON output
    results = {}
    results["num_frames"] = num_samples

    # Write object detection evaluation metrics
    statistics = []
    for class_idx in range(num_labels):
        per_class_stats = {}
        per_class_stats["class_name"] = labels[class_idx]
        per_class_stats["precisions"] = per_class_precisions[class_idx, :].tolist()
        per_class_stats["recalls"] = per_class_recalls[class_idx, :].tolist()
        statistics.append(per_class_stats)
    results["per_class_stats"] = statistics

    # Write confusion matrices and mean IOUs
    conf_matrices_stats = []
    for iou_idx in range(num_thresholds):
        conf_matrix = {}
        conf_matrix["confusion_matrix"] = confusion_matrices[iou_thresholds[iou_idx]].tolist()
        if iou_thresholds[iou_idx] == outlier_iou_area_threshold:
            conf_matrix["mean_iou_width"] = mean_iou_width
            conf_matrix["mean_iou_height"] = mean_iou_height
            conf_matrix["mean_iou_area"] = mean_iou_area
        conf_matrices_stats.append(conf_matrix)
    results["confusion_matrices"] = conf_matrices_stats

    # Write outlier indices
    outliers = {}
    outliers["large_bbox_error_idxs"] = large_bbox_outliers_idxs
    outliers["false_positive_idxs"] = false_positive_idxs
    outliers["false_negative_idxs"] = false_negative_idxs
    results["outliers"] = outliers

    # Write results and KPIs
    evaluation_report["results"] = results
    evaluation_report["KPI_mAP"] = KPI_mAP
    evaluation_report["KPI_mAR"] = KPI_mAR
    evaluation_report["KPI_mAP_lowest_IOU"] = KPI_mAP_lowest_IOU
    evaluation_report["KPI_mAR_lowest_IOU"] = KPI_mAR_lowest_IOU
    evaluation_report["KPI_pass"] = bool(kpi_pass)

    # Write the JSON to file
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    json_output_path = os.path.join(results_dir,
                                    "object_detection_evaluation_" + str(eval_uuid) + ".json")
    with open(json_output_path, 'w') as f:
        json.dump(evaluation_report, f, indent=2)

    # ----------------------- VISUALIZE OUTLIERS  --------------------------------------
    if args.save_outliers:
        large_bbox_outliers_dir = os.path.join(results_dir, "large_bbox_error_outliers")
        if not os.path.exists(large_bbox_outliers_dir):
            os.makedirs(large_bbox_outliers_dir)
        for outlier_idx in large_bbox_outliers_idxs:
            save_sample(outlier_idx, images[outlier_idx], gt_detections[outlier_idx], \
                pred_detections[outlier_idx], "large bbox error outlier", large_bbox_outliers_dir)

        false_positives_dir = os.path.join(results_dir, "false_positives")
        if not os.path.exists(false_positives_dir):
            os.makedirs(false_positives_dir)
        for outlier_idx in false_positive_idxs:
            save_sample(outlier_idx, images[outlier_idx], gt_detections[outlier_idx], \
                pred_detections[outlier_idx], "false positive outlier", false_positives_dir)

        false_negatives_dir = os.path.join(results_dir, "false_negatives")
        if not os.path.exists(false_negatives_dir):
            os.makedirs(false_negatives_dir)
        for outlier_idx in false_negative_idxs:
            save_sample(outlier_idx, images[outlier_idx], gt_detections[outlier_idx], \
                pred_detections[outlier_idx], "false negative outlier", false_negatives_dir)

    print("Evalulation complete. Results saved to: " + json_output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='DetectNet Cask Evaluation')
    parser.add_argument(
        '--config',
        dest='config',
        default="packages/detect_net/evaluation/detect_net_cask_evaluation.config.json",
        help='Config file to load.')
    parser.add_argument(
        '--results_dir',
        dest='results_dir',
        default="/tmp/data/results",
        help='Path to store the evaluation results.')
    parser.add_argument(
        '--image_cask_dir',
        dest='image_cask_dir',
        default="/tmp/data/raw",
        help='Path to the image cask directory')
    parser.add_argument(
        '--gt_cask_dir',
        dest='gt_cask_dir',
        default="/tmp/data/ground_truth",
        help='Path to the ground truth detections cask directory')
    parser.add_argument(
        '--pred_cask_dir',
        dest='pred_cask_dir',
        default="/tmp/data/predictions",
        help='Path to the predicted bounding box cask directorys')
    parser.add_argument(
        '--save_outliers',
        dest='save_outliers',
        default=false,
        help='If enabled, saves outlier images as PNGs in resuls directory')

    args, _ = parser.parse_known_args()
    main(args)
