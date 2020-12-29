/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// Computes the number of overlapping pixels between the specified ground truth and prediction class
// Inputs : Ground truth image, predicted segmentation, ground truth class and prediction class
int ComputeOverlap(const TensorConstView3f& ground_truth, const TensorConstView3f& prediction,
                   const int ground_truth_class, const int prediction_class);

// Computes a confusion matrix with true positive, true negative, false positive and false negative
// Inputs : Ground truth, network prediction
Matrix2i ComputeConfusionMatrix(const TensorConstView3f& ground_truth,
                                const TensorConstView3f& prediction);

// Calculates true positive rate, also known as recall or sensitivity
// It gives us an idea as to how much of the positive pixels were predicted positive
// Formula for the rate: true_positive / true_positive + false_negative
double ComputeTruePositiveRate(const Matrix2i& confusion_matrix);

// Calculates true negative rate, also known as specificity or selectivity
// It gives us an idea as to how much of the negative pixels were predicted negative
// Formula for the rate: true_negative / true_negative + false_positive
double ComputeTrueNegativeRate(const Matrix2i& confusion_matrix);

// Calculates false positive rate, also known as fall-out rate
// It tells us what portion of the true negative pixels were wrongly predicted positive
// Formula for the rate: false_positive / true_negative + false_positive
double ComputeFalsePositiveRate(const Matrix2i& confusion_matrix);

// Calculates false negative rate, also known as miss rate
// It tells us what portion of the true positive pixels were wrongly predicted negative
// Formula for the rate: false_negative / true_positive + false_negative
double ComputeFalseNegativeRate(const Matrix2i& confusion_matrix);

// Computes the precision for the prediction
// It gives us an idea as to what fraction of the predicted positive pixels were actually positive
// Formula for the rate: true_positive / true_positive + false_positive
double ComputePrecision(const Matrix2i& confusion_matrix);

// Computes the intersection-over-union score for the prediction
// It gives us an idea as to how close the predicted positive mask is to the ground truth
// Formula for the score: true_positive / false_positive + true_positive + false_negative
double IntersectionOverUnion(const Matrix2i& confusion_matrix);

// Computes the pixel accuracy for the prediction
// It gives us an overall accuracy measure over both positive and negative classes
// Formula for the score: true_positive + true negative /
//                        false_positive + true_positive + false_negative + true negative
double PixelAccuracy(const Matrix2i& confusion_matrix);

}  // namespace isaac
