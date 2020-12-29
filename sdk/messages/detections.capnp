#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xecd282d812d9c328;

using import "geometry.capnp".BoxProto;
using import "geometry.capnp".RectangleProto;
using import "graph.capnp".GraphProto;
using import "math.capnp".Pose3dProto;
using import "math.capnp".Pose2dProto;
using import "math.capnp".Vector2iProto;
using import "math.capnp".Vector2dProto;
using import "tensor.capnp".TensorProto;

# A prediction gives a class name with a confidence level from 0 to 1
struct PredictionProto {
  # Name or label for the detection
  label @0: Text;
  # A general value to indicate how confident we are about the detection.
  # This could for example be provided by a perception algorithm like a
  # neural network bounding box detector.
  confidence @1: Float64;
}

# A message containing detections made by sensor(s), in the 2D image space
# Each detection has a bounding box and/or 2D pose, label, and confidence
struct Detections2Proto {
  # List of predictions made
  predictions @0: List(PredictionProto);
  # List of 2D bounding boxes where we detected objects
  boundingBoxes @1: List(RectangleProto);
  # List of 2D poses of objects detected. Optional field.
  poses @2: List(Pose2dProto);
}

# A message containing detections made by sensor(s), in the 3D space
# Each detection has a 3D pose, label, confidence, and 3D bounding box
struct Detections3Proto {
  # List of predictions made
  predictions @0: List(PredictionProto);
  # List of 3D bounding boxes in the objects' frame
  boundingBoxes @2: List(BoxProto);
  # List of 3D poses of objects detected relative to the sensor frame
  poses @1: List(Pose3dProto);
}

# Defines a skeleton with 2D joint locations. Used for example to describe
# a human skeleton detected on an image.
struct Skeleton2Proto {
  # A graph defining the topology of the skeleton.
  graph @0: GraphProto;
  # Information about a joint in the skeleton
  struct Joint2Proto {
    # Location of the joint, for example image pixel coordinates.
    position @0: Vector2dProto;
    # A label and confidence, describing the prediction of the joint
    label @1: PredictionProto;
  }
  # Detailed information for every joint in the skeleton. The number of
  # joints must be identical to the number of nodes in the graph.
  joints @1: List(Joint2Proto);
  # A label and confidence, describing the prediction of the skeleton
  label @2: PredictionProto;
}

# Defines a list of Skeleton2Proto messages.
struct Skeleton2ListProto {
  # List of skeleton models
  skeletons @0: List(Skeleton2Proto);
}

# A message to contain the evaluation results of an object detection algorithm. Contains the
# confusion matrices for the classes in a dataset accumulated over a certain number of images in
# the dataset.
struct ConfusionMatrixProto {
  # Number of samples these metrics were accumulated over. If 1, then the confusion matrices hold
  # the results for a single sample. If greater than 1, then the counts that are contained in the
  # confusion matrices are summed over accumulated samples.
  numSamples @0: Int64;

  # List of threshold values used to compute the metrics for each class. Each threshold
  # corresponds to one confusion matrix in the confusionMatrices Tensor.
  # The metric on which the threshold is applied is dependent on the task. In case of object
  # detection, the threshold is typically applied for the IoU (Intersection over Union) score
  # between two bounding boxes.
  thresholds @1: List(Float64);

  # Tensor with dimensions (actual class * predicted class * IoU), where a slice of the tensor
  # across a single IoU is a confusion matrix. (See https://en.wikipedia.org/wiki/Confusion_matrix)
  # Each confusion matrix is calculated using the corresponding IoU threshold, which determines the
  # tolerance to bounding box error. A lower IoU corresponds to greater tolerance to bbox error.
  confusionMatrices @2: TensorProto;
}
