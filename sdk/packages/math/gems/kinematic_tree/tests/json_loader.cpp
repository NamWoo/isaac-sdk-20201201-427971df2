/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/kinematic_tree/json_loader.hpp"

#include "gtest/gtest.h"

#include "packages/math/gems/kinematic_tree/kinematic.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace kinematic_tree {

TEST(JsonLoader, ConstantMotors) {
  const char kinematic_json_text[] =
      R"???({
    "links": [
      {
        "name": "base"
      },
      {
        "name" : "axis1",
        "parent": "base",
        "motor": {
          "type": "constant",
          "properties": {
            "pose": [1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
          }
        }
      }
    ]
  })???";
  Json json = nlohmann::json::parse(kinematic_json_text);
  KinematicTree model;
  EXPECT_TRUE(FromJson(json, model));
  EXPECT_EQ(model.getNumberOfLinks(), 2);
  EXPECT_EQ(model.getMotorStateDimensions(), 0);
  VectorXd state(0);
  ISAAC_EXPECT_POSE_NEAR(ForwardKinematic(model, state, "axis1").toPose3(),
      Pose3d::Translation(Vector3d::Constant(1.0)), 1e-9);
}

TEST(JsonLoader, Kinova) {
  const std::string data_path = "packages/math/gems/kinematic_tree/tests/";
  Json json = serialization::LoadJsonFromFile(data_path + "kinova.kinematic.json");
  KinematicTree model;
  EXPECT_TRUE(FromJson(json, model));
  EXPECT_EQ(model.getNumberOfLinks(), 16);
  EXPECT_EQ(model.getMotorStateDimensions(), 7);
  // validate home position of kinova
  const Vector7d joint_angles = Vector7d((double[]){4.94, 2.834, 0.0, 0.76, 4.63, 4.49, 5.03});
  const Pose3d home_expect {SO3d::FromQuaternion(Quaterniond(0.5511, 0.6452, 0.3175, 0.4234)),
                       Vector3d(0.2116, -0.2648, 0.5054)};
  const Pose3d home_fk = ForwardKinematic(model, joint_angles, "end_effector").toPose3();
  ISAAC_EXPECT_POSE_NEAR(home_fk, home_expect, 1e-2);

}

}  // namespace kinematic_tree
}  // namespace isaac