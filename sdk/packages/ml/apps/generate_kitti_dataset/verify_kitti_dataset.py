'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import argparse
import logging
import math
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
import time
"""
This script visualizes a subset of the generated kitti dataset from
packages/ml/apps/generate_kitti_dataset in order to verify the integrity
of the generated data. To check that boxes and images are properly synced
and data is valid for training, this app samples a subset of the generated
training data and crops the images according to their bounding box labels.
The cropped images are saved in the <path_to_dataset>/training/verify
directory for visual inspection.
"""


def main():
    parser = argparse.ArgumentParser(description='Verify kitti dataset')
    parser.add_argument('--dataset_path',
                        dest='dataset_path',
                        default="/tmp/unity3d_kitti_dataset",
                        help='Path to KITTI Dataset folder')
    parser.add_argument('--num_images',
                        dest='num_images',
                        type=int,
                        default=20,
                        help='Number of images to sample for verification')

    args = parser.parse_args()

    # Get paths to training image and label directories
    image_dir_path = os.path.join(args.dataset_path, "training/image_2/")
    label_dir_path = os.path.join(args.dataset_path, "training/label_2/")
    assert os.path.isdir(image_dir_path) and os.path.isdir(label_dir_path), \
        "Please provide path to valide KITTI dataset generated using"\
            " packages/ml/apps/generate_kitti_dataset:generate_kitti_dataset"

    # Create "verify" directory to save cropped images
    verify_dir_path = os.path.join(args.dataset_path, "training/verify/")
    if (not os.path.isdir(verify_dir_path)):
        try:
            os.mkdir(verify_dir_path)
        except OSError as error:
            print(error)

    # Sample a subset of the generated training images
    images = os.listdir(image_dir_path)
    images = images[:min(args.num_images, len(images))]

    # For each sample image, crop according to first label
    # (if available) and save the image
    for image in images:
        # Open image
        image_path = os.path.join(image_dir_path, image)
        try:
            im = Image.open(image_path)
        except:
            print("Image {} could not be opened".format(image_path))
        im_arr = np.array(im)

        # Open label file and read first line
        label_path = "{}.txt".format(label_dir_path + os.path.splitext(image)[0])
        try:
            with open(label_path, 'r') as label_file:
                line = label_file.readline()
        except:
            print("Label file {} could not be opened".format(label_path))

        # if no labels in file, skip this sample
        if (line == ""):
            continue
        kitti_label = line.split(' ')
        col_min = int(math.floor(float(kitti_label[4])))
        row_min = int(math.floor(float(kitti_label[5])))
        width = int(math.ceil(float(kitti_label[6]) - float(kitti_label[4])))
        height = int(math.ceil(float(kitti_label[7]) - float(kitti_label[5])))

        # crop image according to label and save
        plt.imshow(im_arr[row_min:row_min + height, col_min:col_min + width, :])
        plt.savefig(os.path.join(verify_dir_path, image))


if __name__ == '__main__':
    main()
