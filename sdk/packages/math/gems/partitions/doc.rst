..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

Mappings
========

Coordinate mappings:

- RegularPartition: A regular partition of 1-dimensional Cartesian space.
- RegularIntervalPartition: A regular partition of an interval.
- RingPartition: A regular partition of a circle, i.e. an interval which loops around.
- RegularPlanePartition: A regular partition of 2-dimensional Cartesian space (i.e. a grid).
- PosedRegularPlanePartition: Similar to RegularPlanePartition, but rotated and translated with respect to the origin.
- PolarPartition: Mapping between Cartesian space and polar coordinates.


Storage:

- SingleStore: Uses a single image to store data. This is useful for a small dense map.
- MultiStore: Uses a sparse grid of images to store data. Useful for large maps which have many holes.
