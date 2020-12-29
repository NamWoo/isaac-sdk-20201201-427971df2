/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/geometry/line.hpp"
#include "engine/gems/geometry/line_segment.hpp"
#include "engine/gems/geometry/line_utils.hpp"
#include "engine/gems/geometry/polyline.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

// Closest point to Line

TEST(ClosestPointToLine, line_2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  Line2d line = Line2d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(0.0, 0.0)), Vector2d(-2.0, 2.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(50.0, 50.0)), Vector2d(48.0, 52.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(14.0, 16.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(24.0, 26.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0), 1e-9);
}

TEST(ClosestPointToLine, line_3d) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d end(23.0, 27.0, 32.0);
  Line3d line = Line3d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(0.0, 0.0, 1.0)),
                      Vector3d(-4.0, 0.0, 5.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(50.0, 50.0, 51.0)),
                      Vector3d(46.0, 50.0, 55.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(14.0, 16.0, 22.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(24.0, 26.0, 32.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
}

TEST(ClosestPointToLine, line_2d_ray) {
  Vector2d start(13.0, 17.0);
  Vector2d dir(-7.0, -7.0);
  Line2d line = Line2d::FromDirection(start, dir);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(0.0, 0.0)), Vector2d(-2.0, 2.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(50.0, 50.0)), Vector2d(48.0, 52.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(14.0, 16.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(24.0, 26.0)), Vector2d(23.0, 27.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0), 1e-9);
}

TEST(ClosestPointToLine, line_3d_ray) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d dir(-42.0, -42.0, -42.0);
  Line3d line = Line3d::FromDirection(start, dir);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(0.0, 0.0, 1.0)),
                      Vector3d(-4.0, 0.0, 5.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(50.0, 50.0, 51.0)),
                      Vector3d(46.0, 50.0, 55.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(14.0, 16.0, 22.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(24.0, 26.0, 32.0)),
                      Vector3d(23.0, 27.0, 32.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(line, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
}

// Closest point to Ray

TEST(ClosestPointToLine, Ray_2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  Ray2d ray = Ray2d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(0.0, 0.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(50.0, 50.0)), Vector2d(48.0, 52.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(14.0, 16.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(24.0, 26.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0), 1e-9);
}

TEST(ClosestPointToLine, Ray_3d) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d end(23.0, 27.0, 32.0);
  Ray3d ray = Ray3d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(0.0, 0.0, 1.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(50.0, 50.0, 51.0)),
                      Vector3d(46.0, 50.0, 55.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(14.0, 16.0, 22.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(24.0, 26.0, 32.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
}

TEST(ClosestPointToLine, Ray_2d_ray) {
  Vector2d start(13.0, 17.0);
  Vector2d dir(17.0, 17.0);
  Ray2d ray = Ray2d::FromDirection(start, dir);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(0.0, 0.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(50.0, 50.0)), Vector2d(48.0, 52.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(14.0, 16.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(24.0, 26.0)), Vector2d(23.0, 27.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0), 1e-9);
}

TEST(ClosestPointToLine, Ray_3d_ray) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d dir(13.0, 13.0, 13.0);
  Ray3d ray = Ray3d::FromDirection(start, dir);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(0.0, 0.0, 1.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(50.0, 50.0, 51.0)),
                      Vector3d(46.0, 50.0, 55.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(14.0, 16.0, 22.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(24.0, 26.0, 32.0)),
                      Vector3d(23.0, 27.0, 32.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(ray, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
}

// Closest point to LineSegment

TEST(ClosestPointToLine, linesegment_2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  LineSegment2d segment = LineSegment2d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector2d(0.0, 0.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector2d(50.0, 50.0)), end, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector2d(14.0, 16.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector2d(24.0, 26.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(
      ClosestPointToLine(segment, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(
      ClosestPointToLine(segment, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0), 1e-9);
}

TEST(ClosestPointToLine, linesegment_3d) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d end(23.0, 27.0, 32.0);
  LineSegment3d segment = LineSegment3d::FromPoints(start, end);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(0.0, 0.0, 1.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(50.0, 50.0, 51.0)), end, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(14.0, 16.0, 22.0)), start, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(24.0, 26.0, 32.0)), end, 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);

  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(ClosestPointToLine(segment, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0), 1e-9);
}

// Distance from point to Line

TEST(SquaredDistancePointToLine, line_2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  Line2d line = Line2d::FromPoints(start, end);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(0.0, 0.0)),
              (Vector2d(0.0, 0.0) - Vector2d(-2.0, 2.0)).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(50.0, 50.0)),
              (Vector2d(50.0, 50.0) - Vector2d(48.0, 52.0)).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(14.0, 16.0)),
              (Vector2d(14.0, 16.0) - start).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(24.0, 26.0)),
              (Vector2d(24.0, 26.0) - end).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(18.0, 20.0)), 2.0, 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(line, Vector2d(22.0, 16.0)), 50.0, 1e-9);
}

// Distance from point to Ray

TEST(SquaredDistancePointToLine, Ray_2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  Ray2d ray = Ray2d::FromPoints(start, end);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(0.0, 0.0)),
              (Vector2d(0.0, 0.0) - start).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(50.0, 50.0)),
              (Vector2d(50.0, 50.0) - Vector2d(48.0, 52.0)).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(14.0, 16.0)),
              (Vector2d(14.0, 16.0) - start).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(24.0, 26.0)),
              (Vector2d(24.0, 26.0) - end).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(18.0, 20.0)), 2.0, 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(ray, Vector2d(22.0, 16.0)), 50.0, 1e-9);
}

// Distance from point to LineSegment

TEST(SquaredDistancePointToLine, linesegment_3d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  LineSegment2d segment = LineSegment2d::FromPoints(start, end);
  LineSegment2d point_segment = LineSegment2d::FromPoints(start, start);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(0.0, 0.0)),
              (Vector2d(0.0, 0.0) - start).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(50.0, 50.0)),
              (Vector2d(50.0, 50.0) - end).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(14.0, 16.0)),
              (Vector2d(14.0, 16.0) - start).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(24.0, 26.0)),
              (Vector2d(24.0, 26.0) - end).squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(18.0, 20.0)), 2.0, 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(segment, Vector2d(22.0, 16.0)), 50.0, 1e-9);
  EXPECT_NEAR(
        SquaredDistancePointToLine(point_segment, Vector2d(0.0, 0.0)), start.squaredNorm(), 1e-9);
  EXPECT_NEAR(SquaredDistancePointToLine(point_segment, start), 0.0, 1e-9);
}

// Lines intersection

TEST(AreLinesIntersecting, line_2_line) {
  Vector2d start(13.0, 17.0);
  Vector2d dir(1.0, 2.0);
  const Line2d ref = Line2d::FromDirection(start, dir);
  { // Same line but different starting position and ray
    const Line2d line = Line2d::FromDirection(start + 42.0 * dir, -3.0 * dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  { // Parallel
    const Line2d line = Line2d::FromDirection(SO2d::FromAngle(0.0001) * start, -3.0 * dir);
    EXPECT_FALSE(AreLinesIntersecting(ref, line));
    EXPECT_FALSE(AreLinesIntersecting(line, ref));
  }
  { // Slightly different direction
    const Line2d line = Line2d::FromDirection(Vector2d(42.0, 442.0), SO2d::FromAngle(0.0001) * dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
}

TEST(AreLinesIntersecting, Ray_2_ray) {
  Vector2d start(13.0, 17.0);
  Vector2d dir(1.0, 2.0);
  const Ray2d ref = Ray2d::FromDirection(start, dir);
  {
    const Ray2d line = Ray2d::FromDirection(start, -dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start + 0.0001 * dir, dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start - 0.0001 * dir, dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start + 0.0001 * dir, -dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start - 0.0001 * dir, -dir);
    EXPECT_FALSE(AreLinesIntersecting(ref, line));
    EXPECT_FALSE(AreLinesIntersecting(line, ref));
  }
  { // Parallel
    const Ray2d line = Ray2d::FromDirection(SO2d::FromAngle(0.0001) * start, dir);
    EXPECT_FALSE(AreLinesIntersecting(ref, line));
    EXPECT_FALSE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start + 10.0 * dir, SO2d::FromAngle(1.0) * dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start + 10.0 * dir + Vector2d(1.0, 0.0),
                                                SO2d::FromAngle(1.0) * dir);
    EXPECT_TRUE(AreLinesIntersecting(ref, line));
    EXPECT_TRUE(AreLinesIntersecting(line, ref));
  }
  {
    const Ray2d line = Ray2d::FromDirection(start + 10.0 * dir + Vector2d(-1.0, 0.0),
                                                SO2d::FromAngle(1.0) * dir);
    EXPECT_FALSE(AreLinesIntersecting(ref, line));
    EXPECT_FALSE(AreLinesIntersecting(line, ref));
  }
}

TEST(AreLinesIntersecting, segment_2_segment) {
  { // identical
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(seg1, seg2));
    EXPECT_TRUE(AreLinesIntersecting(seg2, seg1));
  }
  { // Share one point
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(20.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(seg1, seg2));
    EXPECT_TRUE(AreLinesIntersecting(seg2, seg1));
  }
  { // Disjoint one point
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(11.0, 0.0), Vector2d(20.0, 0.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  { // Parallel one point
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(0.0, 10.0), Vector2d(10.0, 10.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  { // Overlap one point
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(9.0, 0.0), Vector2d(20.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(seg1, seg2));
    EXPECT_TRUE(AreLinesIntersecting(seg2, seg1));
  }
  { // included one point
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(-9.0, 0.0), Vector2d(20.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(seg1, seg2));
    EXPECT_TRUE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(-1.0, 1.0), Vector2d(-1.0, -1.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(11.0, 1.0), Vector2d(11.0, -1.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(5.0, 10.0), Vector2d(5.0, 1.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(5.0, -10.0), Vector2d(5.0, -1.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(5.0, -10.0), Vector2d(5.0, 1.0));
    EXPECT_TRUE(AreLinesIntersecting(seg1, seg2));
    EXPECT_TRUE(AreLinesIntersecting(seg2, seg1));
  }
  {
    const LineSegment2d seg1 = LineSegment2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg2 = LineSegment2d::FromPoints(Vector2d(5.0, -10.0), Vector2d(5.0, -10.0));
    EXPECT_FALSE(AreLinesIntersecting(seg1, seg2));
    EXPECT_FALSE(AreLinesIntersecting(seg2, seg1));
  }
}

TEST(AreLinesIntersecting, line_2_ray) {
  { // included
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const Ray2d ray = Ray2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(line, ray));
    EXPECT_TRUE(AreLinesIntersecting(ray, line));
  }
  { // Parallel
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const Ray2d ray = Ray2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_FALSE(AreLinesIntersecting(line, ray));
    EXPECT_FALSE(AreLinesIntersecting(ray, line));
  }
  { // Intersect
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const Ray2d ray = Ray2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, 2.0));
    EXPECT_TRUE(AreLinesIntersecting(line, ray));
    EXPECT_TRUE(AreLinesIntersecting(ray, line));
  }
  { // Miss
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const Ray2d ray = Ray2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    EXPECT_FALSE(AreLinesIntersecting(line, ray));
    EXPECT_FALSE(AreLinesIntersecting(ray, line));
  }
  { // Same as above but reverse Ray and line so they do intersect
    const Line2d line = Line2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    const Ray2d ray = Ray2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    EXPECT_TRUE(AreLinesIntersecting(line, ray));
    EXPECT_TRUE(AreLinesIntersecting(ray, line));
  }
}

TEST(AreLinesIntersecting, line_2_segment) {
  { // included
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(line, seg));
    EXPECT_TRUE(AreLinesIntersecting(seg, line));
  }
  { // Parallel
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_FALSE(AreLinesIntersecting(line, seg));
    EXPECT_FALSE(AreLinesIntersecting(seg, line));
  }
  { // Intersect
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, 2.0));
    EXPECT_TRUE(AreLinesIntersecting(line, seg));
    EXPECT_TRUE(AreLinesIntersecting(seg, line));
  }
  { // Miss
    const Line2d line = Line2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    EXPECT_FALSE(AreLinesIntersecting(line, seg));
    EXPECT_FALSE(AreLinesIntersecting(seg, line));
  }
  { // Same as above but reverse seg and line so they do intersect
    const Line2d line = Line2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    EXPECT_TRUE(AreLinesIntersecting(line, seg));
    EXPECT_TRUE(AreLinesIntersecting(seg, line));
  }
}

TEST(AreLinesIntersecting, Ray_2_segment) {
  { // included
    const Ray2d ray = Ray2d::FromPoints(Vector2d(0.0, 0.0), Vector2d(10.0, 0.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_TRUE(AreLinesIntersecting(ray, seg));
    EXPECT_TRUE(AreLinesIntersecting(seg, ray));
  }
  { // Parallel
    const Ray2d ray = Ray2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(10.0, 0.0), Vector2d(0.0, 0.0));
    EXPECT_FALSE(AreLinesIntersecting(ray, seg));
    EXPECT_FALSE(AreLinesIntersecting(seg, ray));
  }
  { // Intersect
    const Ray2d ray = Ray2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, 2.0));
    EXPECT_TRUE(AreLinesIntersecting(ray, seg));
    EXPECT_TRUE(AreLinesIntersecting(seg, ray));
  }
  { // Miss
    const Ray2d ray = Ray2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    EXPECT_FALSE(AreLinesIntersecting(ray, seg));
    EXPECT_FALSE(AreLinesIntersecting(seg, ray));
  }
  { // Same as above but reverse seg and Ray
    const Ray2d ray = Ray2d::FromPoints(Vector2d(5.0, 0.0), Vector2d(5.0, -2.0));
    const LineSegment2d seg = LineSegment2d::FromPoints(Vector2d(0.0, 1.0), Vector2d(10.0, 1.0));
    EXPECT_FALSE(AreLinesIntersecting(ray, seg));
    EXPECT_FALSE(AreLinesIntersecting(seg, ray));
  }
}

TEST(ClosestPointToSegement, 2d) {
  Vector2d start(13.0, 17.0);
  Vector2d end(23.0, 27.0);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(0.0, 0.0)), start);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(50.0, 50.0)), end);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(14.0, 16.0)), start);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(24.0, 26.0)), end);

  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(18.0, 20.0)), Vector2d(17.0, 21.0));
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector2d(22.0, 16.0)), Vector2d(17.0, 21.0));
}

TEST(ClosestPointToSegement, 3d) {
  Vector3d start(13.0, 17.0, 22.0);
  Vector3d end(23.0, 27.0, 32.0);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(0.0, 0.0, 0.0)), start);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(50.0, 50.0, 50.0)), end);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(14.0, 16.0, 22.0)), start);
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(24.0, 26.0, 32.0)), end);

  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(18.0, 20.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0));
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(22.0, 16.0, 26.0)),
            Vector3d(17.0, 21.0, 26.0));

  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(18.0, 24.0, 22.0)),
            Vector3d(17.0, 21.0, 26.0));
  EXPECT_EQ(ClosestPointToSegement(start, end, Vector3d(22.0, 26.0, 16.0)),
            Vector3d(17.0, 21.0, 26.0));
}

TEST(DistanceSquaredPointToSegement, simple) {
  const Pose2d pose = Pose2d::FromXYA(1.2, 2.3, 0.456);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(-5.0, 0.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(-5.0, 5.0)), 50.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(0.0, 5.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(2.0, 5.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(5.0, 5.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(8.0, 5.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(10.0, 5.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(15.0, 5.0)), 50.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(15.0, 0.0)), 25.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(0.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(5.0, 0.0)), 0.0, 1e-9);
  EXPECT_NEAR(DistanceSquaredPointToSegement(
      pose * Vector2d(0.0, 0.0), pose * Vector2d(10.0, 0.0),
      pose * Vector2d(10.0, 0.0)), 0.0, 1e-9);
}

}  // namespace geometry
}  // namespace isaac
