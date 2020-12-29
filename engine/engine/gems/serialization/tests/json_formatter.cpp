/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "engine/gems/math/test_utils.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

namespace {
// Threshold to be used with ISAAC_EXPECT_POSE_NEAR and ISAAC_EXPECT_SO_NEAR macros
constexpr double kPoseThreshold = 1e-9;
}  // namsepace

TEST(Serialization, Double) {
  const double actual = -1.2;
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), actual);
}

TEST(Serialization, DoubleTryGetFailure1) {
  Json json;
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, DoubleTryGetFailure2) {
  Json json;
  Set(json, false);
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, DoubleFromInt) {
  const int actual = -1;
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), actual);
}

TEST(Serialization, DoubleFromString) {
  const std::string actual = "this is a string";
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, Pair) {
  using T = std::pair<double, int>;
  T val{2.2, -1};

  Json json;
  Set(json, val);

  auto maybe = TryGet<T>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

TEST(Serialization, Map) {
  std::map<std::string, std::vector<std::string>> val;
  val["a"] = {"1", "2", "3"};
  val["b"];

  Json json;
  Set(json, val);

  auto maybe = TryGet<decltype(val)>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

TEST(Serialization, CircleD) {
  geometry::CircleD val{{4.4, 5.5}, 6.6};

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 5.5}, 6.6};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::CircleD>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, CircleDVerbose) {
  geometry::CircleD val{{4.4, 5.5}, 6.6};

  Json json;
  json["center"] = {4.4, 5.5};
  json["radius"] = 6.6;

  auto maybe = TryGet<geometry::CircleD>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, SphereF) {
  geometry::SphereF val{{-2.2, 4.4, 5.5}, 6.6};

  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::SphereF>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, SphereFVerbose) {
  geometry::SphereF val{{-2.2, 4.4, 5.5}, 6.6};

  Json json;
  json["center"] = {-2.2, 4.4, 5.5};
  json["radius"] = 6.6;

  auto maybe = TryGet<geometry::SphereF>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, RectangleD) {
  geometry::RectangleD val = geometry::RectangleD::FromOppositeCorners({4.4, 5.5}, {6.6, 7.7});

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 6.6}, {5.5, 7.7}};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::RectangleD>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->min(), val.min());
  EXPECT_EQ(maybe->max(), val.max());
}

TEST(Serialization, RectangleDVerbose) {
  geometry::RectangleD val = geometry::RectangleD::FromOppositeCorners({4.4, 7.7}, {6.6, 5.5});

  // JSON defined by bounding box
  Json json_from_bounds;
  json_from_bounds["bounds"] = {{4.4, 6.6}, {5.5, 7.7}};

  auto maybe_from_bounds = TryGet<geometry::RectangleD>(json_from_bounds);
  ASSERT_TRUE(maybe_from_bounds);
  EXPECT_EQ(maybe_from_bounds->min(), val.min());
  EXPECT_EQ(maybe_from_bounds->max(), val.max());

  // JSON defined by opposite corners
  Json json_from_corners;
  json_from_corners["corner_1"] = {4.4, 7.7};
  json_from_corners["corner_2"] = {6.6, 5.5};

  auto maybe_from_corners = TryGet<geometry::RectangleD>(json_from_corners);
  ASSERT_TRUE(maybe_from_corners);
  EXPECT_EQ(maybe_from_corners->min(), val.min());
  EXPECT_EQ(maybe_from_corners->max(), val.max());

  // JSON defined by center and dimensions
  Json json_from_dimensions;
  json_from_dimensions["center"] = {5.5, 6.6};
  json_from_dimensions["dimensions"] = {2.2, 2.2};

  auto maybe_from_dimensions = TryGet<geometry::RectangleD>(json_from_dimensions);
  ASSERT_TRUE(maybe_from_dimensions);
  EXPECT_EQ(maybe_from_dimensions->min(), val.min());
  EXPECT_NEAR(maybe_from_dimensions->max()[0], val.max()[0], 1e-15);
  EXPECT_EQ(maybe_from_dimensions->max()[1], val.max()[1]);
}

TEST(Serialization, BoxF) {
  geometry::BoxF val = geometry::BoxF::FromOppositeCorners({4.4, 7.7, 2.2}, {6.6, 5.5, 3.3});

  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::BoxF>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->min(), val.min());
  EXPECT_EQ(maybe->max(), val.max());
}

TEST(Serialization, BoxFVerbose) {
  geometry::BoxF val = geometry::BoxF::FromOppositeCorners({4.4, 7.7, 2.2}, {6.6, 5.5, 3.3});

  // JSON defined by bounding box
  Json json_from_bounds;
  json_from_bounds["bounds"] = {{4.4, 6.6}, {5.5, 7.7}, {2.2, 3.3}};

  auto maybe_from_bounds = TryGet<geometry::BoxF>(json_from_bounds);
  ASSERT_TRUE(maybe_from_bounds);
  EXPECT_EQ(maybe_from_bounds->min(), val.min());
  EXPECT_EQ(maybe_from_bounds->max(), val.max());

  // JSON defined by opposite corners
  Json json_from_corners;
  json_from_corners["corner_1"] = {4.4, 7.7, 2.2};
  json_from_corners["corner_2"] = {6.6, 5.5, 3.3};

  auto maybe_from_corners = TryGet<geometry::BoxF>(json_from_corners);
  ASSERT_TRUE(maybe_from_corners);
  EXPECT_EQ(maybe_from_corners->min(), val.min());
  EXPECT_EQ(maybe_from_corners->max(), val.max());

  // JSON defined by center and dimensions
  Json json_from_dimensions;
  json_from_dimensions["center"] = {5.5, 6.6, 2.75};
  json_from_dimensions["dimensions"] = {2.2, 2.2, 1.1};

  auto maybe_from_dimensions = TryGet<geometry::BoxF>(json_from_dimensions);
  ASSERT_TRUE(maybe_from_dimensions);
  EXPECT_EQ(maybe_from_dimensions->min(), val.min());
  EXPECT_NEAR(maybe_from_dimensions->max()[0], val.max()[0], 1e-5);
  EXPECT_EQ(maybe_from_dimensions->max()[1], val.max()[1]);
}

TEST(Serialization, LineSegment2d) {
  geometry::LineSegment2d val = geometry::LineSegment2d({4.4, 5.5}, {6.6, 7.7});

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 5.5}, {6.6, 7.7}};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::LineSegment2d>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->a(), val.a());
  EXPECT_EQ(maybe->b(), val.b());
}

TEST(Serialization, LineSegment2dVerbose) {
  geometry::LineSegment2d val = geometry::LineSegment2d({4.4, 5.5}, {6.6, 7.7});

  // JSON defined by points
  Json json_from_points;
  json_from_points["point_1"] = {4.4, 5.5};
  json_from_points["point_2"] = {6.6, 7.7};

  auto maybe_from_points = TryGet<geometry::LineSegment2d>(json_from_points);
  ASSERT_TRUE(maybe_from_points);
  EXPECT_EQ(maybe_from_points->a(), val.a());
  EXPECT_EQ(maybe_from_points->b(), val.b());
}

TEST(Serialization, LineSegment3f) {
  geometry::LineSegment3f val = geometry::LineSegment3f({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});
  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::LineSegment3f>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->a(), val.a());
  EXPECT_EQ(maybe->b(), val.b());
}

TEST(Serialization, LineSegment3fVerbose) {
  geometry::LineSegment3f val = geometry::LineSegment3f({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

  // JSON defined by points
  Json json_from_points;
  json_from_points["point_1"] = {1.0, 2.0, 3.0};
  json_from_points["point_2"] = {4.0, 5.0, 6.0};

  auto maybe_from_points = TryGet<geometry::LineSegment3f>(json_from_points);
  ASSERT_TRUE(maybe_from_points);
  EXPECT_EQ(maybe_from_points->a(), val.a());
  EXPECT_EQ(maybe_from_points->b(), val.b());
}

TEST(Serialization, Pixel3ub) {
  Pixel3ub val{120, 150, 200};

  Json json;
  Set(json, val);

  auto maybe_rgb = TryGet<Pixel3ub>(json);
  ASSERT_TRUE(maybe_rgb);
  EXPECT_EQ(maybe_rgb.value().size(), 3);
  EXPECT_EQ(maybe_rgb.value()[0], val[0]);
  EXPECT_EQ(maybe_rgb.value()[1], val[1]);
  EXPECT_EQ(maybe_rgb.value()[2], val[2]);

  // Add alpha channel if not supplied in Json, default value 255 (opaque)
  auto maybe_rgba = TryGet<Pixel4ub>(json);
  ASSERT_TRUE(maybe_rgba);
  EXPECT_EQ(maybe_rgba.value().size(), 4);
  EXPECT_EQ(maybe_rgba.value()[0], val[0]);
  EXPECT_EQ(maybe_rgba.value()[1], val[1]);
  EXPECT_EQ(maybe_rgba.value()[2], val[2]);
  EXPECT_EQ(maybe_rgba.value()[3], 255);
}

TEST(Serialization, Pixel4ub) {
  Pixel4ub val{120, 150, 200, 100};

  Json json;
  Set(json, val);

  // Drop alpha channel when reading Pixel3ub
  auto maybe_rgb = TryGet<Pixel3ub>(json);
  ASSERT_TRUE(maybe_rgb);
  EXPECT_EQ(maybe_rgb.value().size(), 3);
  EXPECT_EQ(maybe_rgb.value()[0], val[0]);
  EXPECT_EQ(maybe_rgb.value()[1], val[1]);
  EXPECT_EQ(maybe_rgb.value()[2], val[2]);

  auto maybe_rgba = TryGet<Pixel4ub>(json);
  ASSERT_TRUE(maybe_rgba);
  EXPECT_EQ(maybe_rgba.value().size(), 4);
  EXPECT_EQ(maybe_rgba.value()[0], val[0]);
  EXPECT_EQ(maybe_rgba.value()[1], val[1]);
  EXPECT_EQ(maybe_rgba.value()[2], val[2]);
  EXPECT_EQ(maybe_rgba.value()[3], val[3]);
}

TEST(Serialization, Vector) {
  using T = Vector<double, 2>;
  T val{12.3, -45.6};

  Json json;
  Set(json, val);

  auto maybe = TryGet<T>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

TEST(Serialization, VectorXd) {
  VectorXd val = VectorXd::Random(5);

  Json json;
  Set(json, val);

  auto maybe = TryGet<VectorXd>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

enum class TestEnum {
  kNone,
  kOne,
  kTwo,
  kThree
};

NLOHMANN_JSON_SERIALIZE_ENUM(TestEnum, {
  {TestEnum::kNone, nullptr},
  {TestEnum::kOne, "one cake"},
  {TestEnum::kTwo, "two flake"},
  {TestEnum::kThree, "and three"},
});

TEST(Serialization, Enum) {
  TestEnum x = TestEnum::kOne;

  Json json;
  Set(json, x);

  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), x);
}

TEST(Serialization, EnumFailure1) {
  Json json;
  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(*maybe, TestEnum::kNone);
}

TEST(Serialization, EnumFailure2) {
  Json json;
  Set(json, 1.3);
  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(*maybe, TestEnum::kNone);
}

TEST(Serialization, SO3) {
  const SO3d actual = SO3d::FromQuaternion(Quaterniond{-0.393, -0.469, -0.725, 0.314});
  Json json;
  Set(json, actual);
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3EmptyFailure) {
  Json json;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

// Make sure we are backward compatible
TEST(Serialization, SO3BackwardCompatible) {
  const SO3d actual = SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0});
  Json json = {0.0, 0.0, 0.0, 1.0};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3BackwardCompatibleFailure1) {
  Json json = {0.0, 0.0, 1.0};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3BackwardCompatibleFailure2) {
  Json json = {0.0, 0.0, 1.0, 1.0, 0.0};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3BackwardCompatibleFailure3) {
  Json json = {0.0, 0.0, 1.0, "a"};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3Quaternion) {
  const SO3d actual = SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0});
  Json json;
  json["qw"] = 0.0;
  json["qx"] = 0.0;
  json["qy"] = 0.0;
  json["qz"] = 1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3QuaternionFailure1) {
  Json json;
  json["qw"] = 0.0;
  json["qx"] = 0.0;
  json["qz"] = 1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3QuaternionFailure2) {
  Json json;
  json["qw"] = 0.0;
  json["qx"] = 0.0;
  json["qy"] = "a";
  json["qz"] = 1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3AxisAngleInRadians) {
  const double angle_radians = 2.0 / 3.0 * Pi<double>;
  const SO3d actual = SO3d::FromAxisAngle({0.0, 2.0, -0.5}, angle_radians);
  Json json;
  json["axis"] = {0.0, 2.0, -0.5};
  json["angle_radians"] = angle_radians;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3AxisAngleInDegrees) {
  const double angle_radians = 2.0 / 3.0 * Pi<double>;
  const SO3d actual = SO3d::FromAxisAngle({0.0, 2.0, -0.5}, angle_radians);
  Json json;
  json["axis"] = {0.0, 2.0, -0.5};
  json["angle_degrees"] = RadToDeg(angle_radians);
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

// Only "angle_radians" or "angle_degrees" need to be set
TEST(Serialization, SO3AxisAngleFailure) {
  Json json;
  json["axis"] = {0.0, 2.0, -0.5};
  json["angle_radians"] = -1.0;
  json["angle_degrees"] = -1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3AngleAxisFailure1) {
  Json json;
  json["axis"] = {0.0, 2.0, -0.5};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3AngleAxisFailure2) {
  Json json;
  json["angle_radians"] = -1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3Euler1) {
  const SO3d actual = SO3d::FromEulerAnglesRPY(0.2, 0.4, -0.5);
  Json json;
  json["roll_radians"] = 0.2;
  json["pitch_radians"] = 0.4;
  json["yaw_radians"] = -0.5;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3Euler2) {
  const SO3d actual = SO3d::FromEulerAnglesRPY(0.2, 0.0, -0.5);
  Json json;
  json["roll_degrees"] = RadToDeg(0.2);
  json["yaw_radians"] = -0.5;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3Euler3) {
  const SO3d actual = SO3d::FromEulerAnglesRPY(0.2, 0.4, -0.0);
  Json json;
  json["roll_radians"] = 0.2;
  json["pitch_radians"] = 0.4;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3Euler4) {
  const SO3d actual = SO3d::FromEulerAnglesRPY(0.0, 0.0, -0.5);
  Json json;
  json["yaw_radians"] = -0.5;
  json["unrelated"] = 0.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_SO_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, SO3EulerFailure1) {
  Json json;
  json["yaw_radians"] = "a";
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, SO3EulerFailure2) {
  Json json;
  json["pitch_radians"] = {0.1, 0.2};
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

// Need to set yaw or (axis+angle). Can't set both.
TEST(Serialization, SO3Failure1) {
  Json json;
  json["yaw_radians"] = -0.5;
  json["axis"] = {0.0, 2.0, -0.5};
  json["angle_radians"] = -1.0;
  auto maybe = TryGet<SO3d>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, Pose3) {
  const Pose3d actual =
      Pose3d{SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0}), Vector3d{25.0, 20.0, 0.0}};
  Json json;
  Set(json, actual);
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

// Make sure we are backward compatible
TEST(Serialization, Pose3BackwardCompatible) {
  const Pose3d actual =
      Pose3d{SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0}), Vector3d{25.0, 20.0, 0.0}};
  Json json = {0.0, 0.0, 0.0, 1.0, 25.0, 20.0, 0.0};
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

// More rotation formats are tested by SO3 tests above
TEST(Serialization, Pose3QuaternionAndTranslation) {
  const Pose3d actual =
      Pose3d{SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0}), Vector3d{25.0, 20.0, 0.0}};
  Json json;
  json["translation"] = {25.0, 20.0, 0.0};
  json["rotation"] = {
    { "qw", 0.0 },
    { "qx", 0.0 },
    { "qy", 0.0 },
    { "qz", 1.0 }
  };
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

// This also tests "angle_degrees". Note that "angle_radians" is tested above.
TEST(Serialization, Pose3AngleAxisAndTranslation) {
  const Pose3d actual =
      Pose3d{SO3d::FromAngleAxis(Pi<double>, {1.0, 0.0, 0.0}), Vector3d{25.0, 20.0, 0.0}};
  Json json;
  json["translation"] = {25.0, 20.0, 0.0};
  json["rotation"] = {
    { "angle_degrees", 180},
    { "axis", {1.0, 0.0, 0.0}}
  };
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, Pose3Translation) {
  const Pose3d actual = Pose3d::Translation(25.0, 20.0, 0.0);
  Json json;
  json["translation"] = {25.0, 20.0, 0.0};
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, Pose3Quaternion) {
  const Pose3d actual =
      Pose3d{SO3d::FromQuaternion(Quaterniond{0.0, 0.0, 0.0, 1.0}), Vector3d::Zero()};
  Json json;
  json["rotation"] = {
    {"qw", 0.0},
    {"qx", 0.0},
    {"qy", 0.0},
    {"qz", 1.0}
  };
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

TEST(Serialization, Pose3Identity) {
  const Pose3d actual = Pose3d::Identity();
  Json json;
  auto maybe = TryGet<Pose3d>(json);
  ASSERT_TRUE(maybe);
  ISAAC_EXPECT_POSE_NEAR(maybe.value(), actual, kPoseThreshold);
}

// Use the to_json/from_json from nlohmann
// https://github.com/nlohmann/json#basic-usage
TEST(Serialization, ToJsonFromJsonVector2d) {
  using T = Vector2d;
  const T val{-11.1, 2.2};

  Json json = val;
  auto back = json.get<T>();
  EXPECT_EQ(back, val);
}

}  // namespace serialization
}  // namespace isaac
