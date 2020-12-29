/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

int ComputeOverlap(const TensorConstView3f& ground_truth, const TensorConstView3f& prediction,
                   const int ground_truth_class, const int prediction_class) {
  // Compute the number of overlapping pixels between the ground truth class and prediction class
  ASSERT(ground_truth.dimensions()[0] == prediction.dimensions()[0] &&
         ground_truth.dimensions()[1] == prediction.dimensions()[1] &&
         ground_truth.dimensions()[2] == prediction.dimensions()[2],
         "Dimensions of ground truth and predicted classes must match");
  int overlap_count = 0;
  for (int class_index = 0; class_index < ground_truth.dimensions()[2]; class_index++) {
    for (int row = 0; row < ground_truth.dimensions()[0]; row++) {
      for (int col = 0; col < ground_truth.dimensions()[1]; col++) {
        float ground_truth_value = ground_truth(row, col, class_index);
        float prediction_value = prediction(row, col, class_index);
        if (static_cast<int>(ground_truth_value) == ground_truth_class &&
            static_cast<int>(prediction_value) == prediction_class) {
          overlap_count++;
        }
      }
    }
  }
  return overlap_count;
}

Matrix2i ComputeConfusionMatrix(const TensorConstView3f& ground_truth,
                                const TensorConstView3f& prediction) {
  Matrix2i confusion_matrix;
  // True positive
  confusion_matrix(0, 0) = ComputeOverlap(ground_truth, prediction, 1, 1);
  // False positive
  confusion_matrix(0, 1) = ComputeOverlap(ground_truth, prediction, 0, 1);
  // False negative
  confusion_matrix(1, 0) = ComputeOverlap(ground_truth, prediction, 1, 0);
  // True negative
  confusion_matrix(1, 1) = ComputeOverlap(ground_truth, prediction, 0, 0);
  return confusion_matrix;
}

double ComputeTruePositiveRate(const Matrix2i& confusion_matrix) {
  // It can also be calculated as 1 - false negative rate
  const int divisor = confusion_matrix(0, 0) + confusion_matrix(1, 0);
  if (divisor == 0) {
    return 0.0;
  }
  return static_cast<double>(confusion_matrix(0, 0)) / static_cast<double>(divisor);
}

double ComputeTrueNegativeRate(const Matrix2i& confusion_matrix) {
  // It can also be calculated as 1 - false positive rate
  const int divisor = confusion_matrix(1, 1) + confusion_matrix(0, 1);
  if (divisor == 0) {
    return 0.0;
  }
  return static_cast<double>(confusion_matrix(1, 1)) / static_cast<double>(divisor);
}

double ComputeFalsePositiveRate(const Matrix2i& confusion_matrix) {
  // It can also be calculated as 1 - true negative rate
  const int divisor = confusion_matrix(1, 1) + confusion_matrix(0, 1);
  if (divisor == 0) {
    return 0.0;
  }
  return static_cast<double>(confusion_matrix(0, 1)) / static_cast<double>(divisor);
}

double ComputeFalseNegativeRate(const Matrix2i& confusion_matrix) {
  // It can also be calculated as 1 - true positive rate
  const int divisor = confusion_matrix(0, 0) + confusion_matrix(1, 0);
  if (divisor == 0) {
    return 0.0;
  }
  return static_cast<double>(confusion_matrix(1, 0)) / static_cast<double>(divisor);
}

double ComputePrecision(const Matrix2i& confusion_matrix) {
  const int divisor = confusion_matrix(0, 0) + confusion_matrix(0, 1);
  if (divisor == 0) {
    return 0.0;
  }
  return static_cast<double>(confusion_matrix(0, 0)) / static_cast<double>(divisor);
}

double IntersectionOverUnion(const Matrix2i& confusion_matrix) {
  const int positive = confusion_matrix(0, 0) + confusion_matrix(1, 0);
  const int true_positive = confusion_matrix(0, 0);
  if (positive == 0 && true_positive == 0) {
    return 1.0;
  } else {
    const int divisor = positive + confusion_matrix(0, 1);
    if (divisor == 0) {
      return 0.0;
    }
    return static_cast<double>(confusion_matrix(0, 0)) / static_cast<double>(divisor);
  }
}

double PixelAccuracy(const Matrix2i& confusion_matrix) {
  const int divisor = confusion_matrix(0, 0) + confusion_matrix(1, 1) +
                      confusion_matrix(0, 1) + confusion_matrix(1, 0);
  if (divisor == 0) {
    return 0.0;
  }
  const int dividend = confusion_matrix(0, 0) + confusion_matrix(1, 1);
  const double accuracy = static_cast<double>(dividend) / static_cast<double>(divisor);
  return accuracy;
}

}  // namespace isaac
