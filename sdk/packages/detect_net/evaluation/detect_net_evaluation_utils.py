'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

"""
Utility functions for Detect Net evaluation.
"""


def compute_intersection_over_union(gt_bbox, pred_bbox):
    """ Compute the intersection over union for a ground truth bounding box and a prediction
        bounding box

        Params:
            gt_bbox (RectangleProto): single ground truth bbox
            pred_bbox (RectangleProto): single prediction bbox
        Returns:
            iou_width (double): intersection over union for width dimension
            iou_height (double): intersection over union for height dimension
            iou_area (double): intersection over union area
    """
    intersection_width = min(gt_bbox.max.y, pred_bbox.max.y) - max(gt_bbox.min.y, pred_bbox.min.y)
    intersection_height = min(gt_bbox.max.x, pred_bbox.max.x) - max(gt_bbox.min.x, pred_bbox.min.x)
    intersection_area = intersection_width * intersection_height

    gt_bbox_width = gt_bbox.max.y - gt_bbox.min.y
    gt_bbox_height = gt_bbox.max.x - gt_bbox.min.x
    gt_bbox_area = gt_bbox_width * gt_bbox_height

    pred_bbox_width = pred_bbox.max.y - pred_bbox.min.y
    pred_bbox_height = pred_bbox.max.x - pred_bbox.min.x
    pred_bbox_area = pred_bbox_width * pred_bbox_height

    union_width = gt_bbox_width + pred_bbox_width - intersection_width
    union_height = gt_bbox_height + pred_bbox_height - intersection_height
    union_area = gt_bbox_area + pred_bbox_area - intersection_area

    iou_width = intersection_width / union_width
    iou_height = intersection_height / union_height
    iou_area = intersection_area / union_area

    return iou_width, iou_height, iou_area


def match_bboxes(gt_bboxes, pred_bboxes, iou_threshold):
    """ Match GT and prediction boxes according to an IOU threshold for a given image sample

        Params:
            gt_bboxes (List of RectangleProto messages): ground truth bboxes for one sample
            pred_bboxes (List of RectangleProto messages): prediction bboxes for one sample

        Returns:
            (gt_matched_idxs, pred_matched_idxs, matched_ious):
                gt_matched_idxs (List of int): Matched indices into gt_bboxes list
                pred_matched_idxs (List of int): Matched indices into pred_bboxes list
                matched_ious_width (List of double): IOU width values for each match
                matched_ious_height (List of double): IOU height values for each match
                matched_ious_area (List of double): IOU area values for each match
    """

    gt_matched_idxs = []
    pred_matched_idxs = []
    matched_ious_width = []
    matched_ious_height = []
    matched_ious_area = []

    for gt_idx in range(len(gt_bboxes)):
        for pred_idx in range(len(pred_bboxes)):
            if pred_idx in pred_matched_idxs: continue
            iou_width, iou_height, iou_area = compute_intersection_over_union(
                gt_bboxes[gt_idx], pred_bboxes[pred_idx])
            if iou_area > iou_threshold:
                gt_matched_idxs.append(gt_idx)
                pred_matched_idxs.append(pred_idx)
                matched_ious_width.append(iou_width)
                matched_ious_height.append(iou_height)
                matched_ious_area.append(iou_area)

    return gt_matched_idxs, pred_matched_idxs, matched_ious_width, matched_ious_height, \
        matched_ious_area


def matches_to_confusion_matrix(gt_matched_idxs, pred_matched_idxs, gt_predictions,
                                pred_predictions, labels):
    """ Compute confusion matrix for a given image sample based matches between ground truth
        and predicted detections

        Params:
            gt_matched_idxs (List of int): Matched indices into gt_bboxes list
            pred_matched_idxs (List of int): Matched indices into pred_bboxes list
            gt_predictions (List of PredictionProto messages): prediction messages for ground truth
            pred_predictions (List of PredictionProto messages): prediction messages for predictions
            labels (List of str): List of N labels

        Returns:
            confusion_matrix_sample (np.array): MxM array, where:
                M = N (number of labels) + 1 (background class)
    """

    #Initialize confusion matrix with axis 0 = actual class, axis 1 = predicted class
    num_labels = len(labels)
    background_class_idx = len(labels)
    confusion_matrix_sample = np.zeros((num_labels + 1, num_labels + 1), dtype=np.int32)

    #Label to index mapping
    labels_map = {k: v for v, k in enumerate(labels)}
    labels_map["background"] = background_class_idx

    num_matches = len(gt_matched_idxs)
    num_gt_predictions = len(gt_predictions)
    num_pred_predictions = len(pred_predictions)

    # unmatched ground truth: false negatives for background class
    for idx in range(num_gt_predictions):
        if idx not in gt_matched_idxs:
            gt_label = gt_predictions[idx].label
            gt_label_idx = labels_map[gt_label.lower()]
            pred_label_idx = background_class_idx
            confusion_matrix_sample[gt_label_idx, pred_label_idx] += 1

    # unmatched predictions: false positives for background class
    for idx in range(num_pred_predictions):
        if idx not in pred_matched_idxs:
            gt_label_idx = background_class_idx
            pred_label = pred_predictions[idx].label
            pred_label_idx = labels_map[pred_label.lower()]
            confusion_matrix_sample[gt_label_idx, pred_label_idx] += 1

    # matched predictions
    for match_idx in range(num_matches):
        gt_label = gt_predictions[gt_matched_idxs[match_idx]].label
        pred_label = pred_predictions[pred_matched_idxs[match_idx]].label
        gt_label_idx = labels_map[gt_label.lower()]
        pred_label_idx = labels_map[pred_label.lower()]
        confusion_matrix_sample[gt_label_idx, pred_label_idx] += 1

    return confusion_matrix_sample


def plot_bboxes(ax, bboxes, color):
    """ Plot bboxes onto a matplotlib subplot
        Params:
            ax (Matplotlib subplot): axis of the subplot to plot bboxes
            bboxes (List of RectangleProto messages): bboxes for a given image sample
            color (str): color to draw boxes
    """
    for bbox in bboxes:
        col_min = bbox.min.y
        row_min = bbox.min.x
        width = bbox.max.y - col_min
        height = bbox.max.x - row_min
        bbox = patches.Rectangle(
            (col_min, row_min), width, height, linewidth=1, edgecolor=color, facecolor='none')
        ax.add_patch(bbox)


def save_sample(idx, image, gt_detection, pred_detection, title, directory):
    """ Display one sample image with ground truth detections and predicted detections
        Params:
            image (TensorProto message): image to display
            gt_detection (Detections2Proto message): ground truth detection
            pred_detection (Detections2Proto message): predicted detection
    """
    fig, ax = plt.subplots(1)
    ax.imshow(image.tensor)
    plot_bboxes(ax, gt_detection.proto.boundingBoxes, 'orange')
    plot_bboxes(ax, pred_detection.proto.boundingBoxes, 'green')
    plt.title(title)
    fig.savefig(os.path.join(directory, str(idx) + ".png"))
    plt.clf()
    plt.close()
