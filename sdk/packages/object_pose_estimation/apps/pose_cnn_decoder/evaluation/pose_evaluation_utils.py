'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from packages.object_pose_estimation.apps.pose_cnn_decoder.evaluation.cuboid import Cuboid3d
from matplotlib import pyplot as plt

import cv2
import math
import matplotlib.animation as animation
import numpy as np
import quaternion


def save_animation_from_image_list(image_list, save_animation_path):
    """
    Saves a stream of images received as animation.

    Args:
        image_list: ndarray of image tensors to be displayed in stream
        save_animation_path: Path to save the animation of stream of images
    """
    fig = plt.figure()
    im = plt.imshow(image_list[0])

    def updatefig(array, *args):
        im.set_array(array)
        return im,

    ani = animation.FuncAnimation(fig, func=updatefig, frames=image_list, interval=100, blit=True)
    ani.save(save_animation_path, writer='ffmpeg')


def compute_median_pose_errors(rotation_err, translation_err):
    """
    Returns the median error of the rotation (index 0) and translation errors (indices 1-3)
    as list of 4 values.

    Args:
        translation_err: Translation error of dimensions [num_data x 3]
        rotation_err: Rotation errors of dimensions [num_data x 1]
    """
    median_trans_err = np.median(translation_err, axis=0)
    median_rot_err = np.median(rotation_err)
    return [median_rot_err, median_trans_err[0], median_trans_err[1], median_trans_err[2]]


def compute_accuracy(translation_err, rotation_err, translation_err_threshold,
                     rotation_err_threshold):
    """
    Returns the accuracy metric of the model which is fraction of data within the specified
    error thresholds
    Ref: PoseCNN-https://arxiv.org/pdf/1711.00199.pdf

    Args:
        translation_err: Translation error of dimensions [num_data x 3]
        rotation_err: Rotation errors of dimensions [num_data x 1]
        translation_err_threshold: Tuple of size 3 specifying thresholds for the translation errors
        rotation_err_threshold: Scalar specifying threshold for the rotation error in degrees.
    """
    positive_ind = (np.where((translation_err[:, 0] <= translation_err_threshold[0]) & \
        (translation_err[:, 1] <= translation_err_threshold[1]) & \
        (translation_err[:, 2] <= translation_err_threshold[2]) & \
        (rotation_err < rotation_err_threshold)))[0].tolist()
    return float(len(positive_ind)) / translation_err.shape[0]


def quaternion_to_ndarray(q):
    """
    Converts the numpy Quaternion to ndarray

    Args:
        q: Numpy quaternion in order (q.w, q.x, q.y, q.z)
    """
    return np.array([q.w, q.x, q.y, q.z])


def compute_roi_abs_translation_error(predicted_pose, gt_pose, depth_roi=None):
    """
    Returns absolute error of the translation x, y an d z positions, given
    ndarrays of predicted and ground truth poses.
    If region of interest (ROI) for depth/distance from camera is given as input,
    only errors for samples inside ROI are returned.

    Args:
        predicted_pose: ndarray of predcited pose of dimensions [num_samples, 3]
        gt_pose: ndarray of ground truth pose of dimensions [num_samples, 3]
        depth_roi: Tuple of minimum and maximum depth values for depth.
    """
    region_of_interest = range(predicted_pose.shape[0])
    if (depth_roi is not None):
        roi_index = np.where((gt_pose[:,6] > depth_roi[0]) & (gt_pose[:,6] \
            <= depth_roi[1]))[0].tolist()
        if (len(roi_index) > 0):
            # If no sample in region of interest, compute error over full region
            region_of_interest = roi_index

    return np.abs(gt_pose[region_of_interest, 4:7] - predicted_pose[region_of_interest, 4:7])


def compute_roi_rotation_error(predicted_pose,
                               gt_pose,
                               symm_axis,
                               num_rotation_symmetry,
                               depth_roi=None):
    """
    Returns rotation error between two input quaternions in degrees by taking all
    the symmetric rotations of the object into account.
    If region of interest for depth/distance from camera is given as input,
    only errors for samples inside ROI are returned.

    Args:
        predicted_pose: ndarray of predcited pose of dimensions [num_samples, 3]
        gt_pose: ndarray of ground truth pose of dimensions [num_samples, 3]
        symm_axis: Rotation axis of symmetry of the object
        num_rotation_symmetry: Number of rotations of symmetry in the given symmetry axis.
        depth_roi: Tuple of minimum and maximum depth values for depth.
    """
    rotation_err = []
    for i in range(predicted_pose.shape[0]):
        # Get all symmetric ground truth rotations with default axis of symmetry
        # TODO(Sravya): Extend the capability to handle multiple axes of symmetry
        gt_symm_rotations = compute_symmetry_rotations(gt_pose[i, :4], symm_axis,\
            num_rotation_symmetry)
        rotation_err.append(compute_rotation_err(gt_symm_rotations, predicted_pose[i, :4]))
    # Convert rotation errors from radians to degrees
    rotation_err = np.asarray(rotation_err) * 180 / np.pi
    region_of_interest = range(predicted_pose.shape[0])
    if (depth_roi is not None):
        roi_index = np.where((gt_pose[:,6] > depth_roi[0]) & (gt_pose[:,6] \
            <= depth_roi[1]))[0].tolist()
        if (len(roi_index) > 0):
            # If no sample in region of interest, compute error over full region
            region_of_interest = roi_index
    return rotation_err[region_of_interest]


def compute_symmetry_rotations(ref, symmetry_axis, num_rotation_symmetry):
    """
    Returns the list of all symmetric rotations of input rotation
    given the symmetry axis and the number of rotation symmetries.

    Args:
        ref: input rotation in quaternion order (q.w, q.x, q.y, q.z)
        symmetry_axis: 0 for x-axis, 1 for y-axis and 2 for z-axis
        num_rotation_symmetry: Number of rotation symmetries about an axis
    """
    symm_rotations = np.zeros((num_rotation_symmetry, 4))
    unit_rotation = np.zeros((3))
    unit_rotation[num_rotation_symmetry] = 1
    for i_rot_symm in range(num_rotation_symmetry):
        symm_rotation_angle = 2 * math.pi * i_rot_symm / num_rotation_symmetry
        rotation_z = quaternion.from_rotation_vector((symm_rotation_angle * \
            unit_rotation).tolist())
        quat_rotate = quaternion.from_float_array(ref) * rotation_z
        symm_rotations[i_rot_symm, :] = quaternion_to_ndarray(quat_rotate)
    return symm_rotations


def compute_rotation_err(ground_truth, predictions):
    """
    Returns the minimum of the quaternion rotation error of the predicted rotation
    across all ground truth rotations

    Args:
        ground_truth: ndarray of all ground truth rotations as quaternions including
        object symmetries.
        predictions: ndarray of quaternion rotation predicted by the model
    """
    num_rotations = ground_truth.shape[0]
    rotation_err = 2 * np.pi
    for i in range(num_rotations):
        rotation_err = np.minimum(
            rotation_err, compute_angle_between_rotations(ground_truth[i, :], predictions))
    return rotation_err


def compute_angle_between_rotations(rotation_lhs, rotation_rhs):
    """
    Returns the angle in rad between two input rotations as quaternions

    Args:
        rotation_lhs, rotation_rhs: Ndarrays of the two quaternion rotations in order
        (q.w, q.x, q.y, q.z)
    """
    rotation_lhs /= np.linalg.norm(rotation_lhs)
    rotation_rhs /= np.linalg.norm(rotation_rhs)
    return np.arccos(np.clip(2 * np.square(np.dot(rotation_lhs, rotation_rhs)) - 1.0, -1.0, 1.0))


def compute_pose_error_depth_bins(rotation_err, translation_err, gt_poses, dist_bins):
    """
    Returns the rotation and translation errors, ground truth depth values in bins/intervals
    determined by the input argument depth_bins
    Ref: PoseCNN-https://arxiv.org/pdf/1711.00199.pdf

    Args:
        translation_err: Translation error of dimensions [num_data x 3]
        rotation_err: Rotation errors of dimensions [num_data x 1]
        dist_bins: list of camera distances in ascending order.
        Bins are formed between two adjacent values in the list.
    """
    trans_err_x_bins = []
    trans_err_y_bins = []
    trans_err_z_bins = []
    dist_values_bins = []
    rotation_err_bins = []
    for i in range(len(dist_bins)):
        if i == 0:
            z_min = 0
        else:
            z_min = dist_bins[i - 1]
        z_max = dist_bins[i]
        roi = np.where((gt_poses[:, 6] > z_min) & (gt_poses[:, 6] <= z_max))
        trans_err_x_bins.append(translation_err[roi, 0].squeeze().tolist())
        trans_err_y_bins.append(translation_err[roi, 1].squeeze().tolist())
        trans_err_z_bins.append(translation_err[roi, 2].squeeze().tolist())
        rotation_err_bins.append(rotation_err[roi].squeeze().tolist())
        dist_values_bins.append(gt_poses[roi, 6].squeeze().tolist())
    return (rotation_err_bins, trans_err_x_bins, trans_err_y_bins, trans_err_z_bins,
            dist_values_bins)


def pose_capnp_to_list(pose):
    """
    Reads Detections3Proto capnp message and returns pose as python list of 7 values.
    First four values are the quaternion for rotation and next three are the translation values.

    Args:
        pose: Detections3Proto capnp message
    """
    return [pose.rotation.q.w, pose.rotation.q.x, pose.rotation.q.y, pose.rotation.q.z, \
        pose.translation.x, pose.translation.y, pose.translation.z]


def get_camera_matrix(focal, center):
    """
    Returns camera intrinsics matrix from focal and image centers
    f = [fx, fy]: focal lengths along x and y axes
    c = [cx, cy]: Image centers along x and y axes
    """
    matrix_camera = np.zeros((3, 3))
    matrix_camera[0, 0] = focal[0]
    matrix_camera[1, 1] = focal[1]
    matrix_camera[0, 2] = center[0]
    matrix_camera[1, 2] = center[1]
    matrix_camera[2, 2] = 1
    return matrix_camera


def draw_cuboid(vert, img):
    """
    Draws the cuboid on an input image with given vertices in pixels and returns the updated image

    Args:
        vert: Ndarray of the 8 vertices of the cuboid
        img: Ndarray of image that the cubiod is added to.
    """
    img = cv2.line(img, tuple(np.int_(vert[1])), tuple(np.int_(vert[0])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[0])), tuple(np.int_(vert[3])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[3])), tuple(np.int_(vert[2])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[2])), tuple(np.int_(vert[1])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[5])), tuple(np.int_(vert[4])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[4])), tuple(np.int_(vert[7])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[7])), tuple(np.int_(vert[6])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[6])), tuple(np.int_(vert[5])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[2])), tuple(np.int_(vert[6])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[1])), tuple(np.int_(vert[5])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[3])), tuple(np.int_(vert[7])), (0, 255, 0), 4)
    img = cv2.line(img, tuple(np.int_(vert[0])), tuple(np.int_(vert[4])), (0, 255, 0), 4)
    return img


def visualize_3Dboundingbox(object_size, object_center, camera_matrix, pose, img):
    """
    Returns the input image with added 3D bounding box visualization

    Args:
        Object size, object_center: Tuple of size 3 each
        camera_matrix: camera intrinsics matrix,
        pose: Ndarray of 7 elements [q.w, q.x, q.y, q.z, x, y, z],
        img: input image to draw the 3D bounding box on.
    """
    _cuboid3d = Cuboid3d(object_size, object_center)
    cuboid3d_points = np.array(_cuboid3d.get_vertices())
    rotation_matrix = quaternion.as_rotation_matrix(np.quaternion(pose[0], pose[1], \
        pose[2], pose[3]))
    # Reference: https://www.programcreek.com/python/example/89450/cv2.Rodrigues
    rvec = cv2.Rodrigues(rotation_matrix)[0]
    tvec = pose[4:]
    dist_coeffs = np.zeros((4, 1))
    # Compute the pixel coordinates of the 3D points
    projected_points, _ = cv2.projectPoints(cuboid3d_points, rvec, tvec, camera_matrix,\
        dist_coeffs)
    projected_points = np.squeeze(projected_points)
    # Draw line to form 3D bounding box from project points
    img = draw_cuboid(projected_points, img)
    return img


def visualize_2Dboundingbox(detection, img):
    """
    Returns 2D bounding box on the image
    Args:
    detection: [bbox.min.x, bbox.min.y, bbox.max.x, bbox.max.y]
    img: input image to draw the 2D bounding box on.
    """
    if (np.linalg.norm(detection) == 0):
        return img
    img = cv2.rectangle(img, (int(detection[1]), int(detection[0])), (int(detection[3]),\
        int(detection[2])), (255,255,255), 2)
    return img