/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/geometry/types.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

TEST(Cuboid, rectangle) {
  {
    RectangleD rect = RectangleD::FromBoundingCuboid({Vector2d(-5.0, -1.0), Vector2d(-2.0, 12.0)});
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
  {
    RectangleD rect = RectangleD::FromBoundingCuboid({Vector2d(-5.0, -1.0), Vector2d(-2.0, 12.0)});
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
  {
    RectangleD rect = RectangleD::FromOppositeCorners(Vector2d(-5.0, -2.0), Vector2d(-1.0, 12.0));
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
  {
    RectangleD rect = RectangleD::FromOppositeCorners(Vector2d(-5.0, 12.0), Vector2d(-1.0, -2.0));
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
  {
    RectangleD rect = RectangleD::FromSizes(Vector2d(-3.0, 5.0), Vector2d(4.0, 14.0));
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
  {
    RectangleD rect = RectangleD::FromSizes(Vector2d(-3.0, 5.0), {4.0, 14.0});
    EXPECT_TRUE(rect.isInside(Vector2d(-3.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-6.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, -4.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(0.0, 0.0)));
    EXPECT_FALSE(rect.isInside(Vector2d(-3.0, 20.0)));
  }
}

TEST(Sphere, circle) {
  CircleD circle{Vector2d(-5.0, -2.0), 3.0};
  EXPECT_TRUE(circle.isInside(Vector2d(-5.0, -2.0)));
  EXPECT_TRUE(circle.isInside(Vector2d(-3.0, -1.0)));
  EXPECT_TRUE(circle.isInside(Vector2d(-2.0, -2.0)));
  EXPECT_FALSE(circle.isInside(Vector2d(-1.9, -2.0)));
}

TEST(Casting, cast) {
  {
    CircleF ref{Vector2f(-5.0f, -2.0f), 3.0f};
    CircleF casted = ref.cast<double>().cast<float>();
    EXPECT_NEAR((ref.center - casted.center).squaredNorm(), 0.0f, 1e-7f);
    EXPECT_NEAR(ref.radius, casted.radius, 1e-7f);
  }
  {
    RectangleF ref = RectangleF::FromOppositeCorners(Vector2f(-5.0, -2.0), Vector2f(-1.0, 12.0));
    RectangleF casted = ref.cast<double>().cast<float>();
    EXPECT_NEAR((ref.min() - casted.min()).squaredNorm(), 0.0f, 1e-7f);
    EXPECT_NEAR((ref.max() - casted.max()).squaredNorm(), 0.0f, 1e-7f);
  }
  {
    Line2f ref = Line2f::FromPoints(Vector2f(-5.0, -2.0), Vector2f(-1.0, 12.0));
    Line2f casted = ref.cast<double>().cast<float>();
    EXPECT_NEAR((ref.origin() - casted.origin()).squaredNorm(), 0.0f, 1e-7f);
    EXPECT_NEAR((ref.direction() - casted.direction()).squaredNorm(), 0.0f, 1e-7f);
  }
  {
    Ray2f ref = Ray2f::FromPoints(Vector2f(-5.0, -2.0), Vector2f(-1.0, 12.0));
    Ray2f casted = ref.cast<double>().cast<float>();
    EXPECT_NEAR((ref.origin() - casted.origin()).squaredNorm(), 0.0f, 1e-7f);
    EXPECT_NEAR((ref.direction() - casted.direction()).squaredNorm(), 0.0f, 1e-7f);
  }
  {
    LineSegment2f ref = LineSegment2f::FromPoints(Vector2f(-5.0, -2.0), Vector2f(-1.0, 12.0));
    LineSegment2f casted = ref.cast<double>().cast<float>();
    EXPECT_NEAR((ref.a() - casted.a()).squaredNorm(), 0.0f, 1e-7f);
    EXPECT_NEAR((ref.b() - casted.b()).squaredNorm(), 0.0f, 1e-7f);
  }
}

}  // namespace geometry
}  // namespace isaac
