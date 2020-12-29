/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/serialization/json.hpp"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"

namespace isaac {
namespace kinematic_tree {

// Available types of motor
enum MotorType {
  kConstantMotor,  // Constant transformation motor
  kRevoluteMotor,  // Revolute joint motor
  kInvalid = -1    // Invalid motor type, returned from an invalid string in JSON
};

// Mapping between each MotorType type and an identifying string.
NLOHMANN_JSON_SERIALIZE_ENUM(MotorType, {
  {MotorType::kInvalid, nullptr},
  {MotorType::kConstantMotor, "constant"},
  {MotorType::kRevoluteMotor, "revolute"}
});

// Loads a manipulator model from a json. Returns whether or not the model was loaded successfully.
// The "xx.kinematic.json" JSON format used the following structure:
//
//  {
//    "links": [
//      {
//        "name": "base"
//      },
//      {
//        "name" : "axis1",
//        "parent": "base",
//        "motor": {
//          "type": "constant",
//          "properties": {
//            "pose": {
//              "translation": [0.0, 0.0, 0.1373],
//              "rotation": {
//                "axis": [0.0, 0.0, 1.0],
//                "angle_degrees": 90
//              }
//            }
//          }
//        }
//      },
//      {
//        "name" : "joint1",
//        "parent": "axis1",
//        "motor": {
//          "type": "revolute",
//          "properties": {
//            "axis": [0, 0, 1]
//          }
//        }
//      }
//    ]
//  }
//
// "links" contains an array of links. Each link contains "name" (str), "parent" (str,
// unless it's the root link), and optionally "motor" (json object).
//
// "motor" is described by "type" (see json to MotorType map above) and "properties".
// For ConstantMotor, "properties" include "pose" (Pose3d) for 3d pose of the motor.
// For RevoluteMotor, "properties" include "axis" (Vector3d) for the rotation axis, and optionally
// "limits" (Vector2d) for joint limit.
//
// See json_formatter.hpp for supported serialization format of Pose3d, Vector3d and Vector2d
//
bool FromJson(const Json& json, KinematicTree& model);

}  // namespace kinematic_tree
}  // namespace isaac
