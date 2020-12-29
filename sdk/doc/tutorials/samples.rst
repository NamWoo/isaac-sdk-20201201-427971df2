..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _tutorials_samples:

Sample Applications
----------------------------------

Small sample applications to demonstrate various features of Isaac can be found in the folders
:samp:`//apps/samples` and :samp:`//apps/tutorials`.
Note that many samples have hardware requirements.

Samples come with a BUILD file and a single JSON file.
The JSON file contains a compute graph, which defines the functionality of the application.
For example, a camera sample application has a camera node and a viewer node.
The camera node acquires images and sends them to the viewer node for visualization.
The JSON file may also contain a configuration section for specification of various configuration
parameters.
Sometimes you might have to adapt the configuration based on your hardware setup or platform.

How to run an application on the desktop is explained in the :ref:`running_an_app` section.
Deploying an application to a Jetson device and running it on the Jetson is explained in the
:ref:`deployment_device` section.

The following sections describe a selection of samples to get you started.


ping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A tutorial that introduces the concept of apps and codelets. It shows how to create a simple
application with a codelet that ticks periodically. You can find a step-by-step tutorial for this
application in the :ref:`cplusplus_ping` section of this documentation.

Platforms: Desktop, Jetson TX2, Jetson Xavier, Jetson Nano

Hardware requirements: none


ping_pong
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A development of the basic ping tutorial that shows how messages can be passed from one node to
another node.

Platforms: Desktop, Jetson TX/2, Jetson Xavier, Jetson Nano

Hardware requirements: none

jupyter_notebook
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A basic tutorial that introduces using Isaac within a Jupyter notebook. It shows how to start and
stop an application, send and receive a message, and edit configuration parameters.

Platforms: Desktop, Jetson TX/2, Jetson Xavier, Jetson Nano

Hardware requirements: none

proportional_control_cpp
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An intermediate tutorial that explains how to write a basic proportional controller for a
differential robot base.

Platforms: Jetson TX2, Jetson Xavier, Jetson Nano

Hardware requirements: Segway


v4l2_camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A basic sample application that connects to a camera using video for Linux (V4L2). The camera image
can be seen in Sight. (You must add a window in Sight to see the image. See
:ref:`sight-windows-menu` for more information.) Make sure to set a valid resolution in the
configuration section in the application file.

Platforms: Desktop, Jetson TX2, Jetson Xavier, Jetson Nano

Hardware: Requires a camera (for example, a Realsense or a ZED camera). A normal webcam may also
work.


april_tags
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Demonstrates the GPU accelerated AprilTag fiducial detection GEM. A connected Realsense camera and
a printed AprilTag are required. (The AprilTag can be displayed on your screen, if necessary.) In
case you do not have a Realsense camera, you can switch to a different camera similar to that used
in the v4l2_camera tutorial.

Platforms: Desktop, Jetson TX2, Jetson Xavier, Jetson Nano

Hardware: Requires a camera and an AprilTag fiducial.



realsense_camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A basic sample application that connects to a Realsense camera. The camera image can be seen in
Sight. Make sure to set a valid resolution in the configuration section in the application file.

Platforms: Desktop, Jetson TX2, Jetson Xavier, Jetson Nano

Hardware: Requires a Realsense camera.


zed_camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A basic sample application that connects to a ZED camera. The camera image can be seen in Sight.
Make sure to set a valid resolution in the configuration section in the application file.

Platforms: Desktop, Jetson TX2, Jetson Xavier, Jetson Nano

Hardware: Requires a ZED camera.


image_undistortion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A sample application that corrects distortion in images captured with a ZED camera. The effects of
the correction can be seen in Sight. (You must add a window in Sight to see the image. See
:ref:`sight-windows-menu` for more information.)

Platforms: Desktop, Jetson Xavier

Hardware: Requires a camera.


hgmm_matching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A sample application that demonstrates point cloud matching using a Realsense camera.

Platforms: Desktop, Jetson Xavier

Hardware: Requires a Realsense camera.

.. _navigation_rosbridge:

navigation_rosbridge
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A sample application that demonstrates creating a bridge between Isaac and ROS by using
`roscpp <http://wiki.ros.org/roscpp>`_, C++ API of ROS.

As :ref:`ros_bridge` section explains, Isaac provides two methods to bridge to ROS:

1. Recommended usage: Using converters provided in "ros_bridge" package,
2. Advanced usage: Using roscpp directly, as this sample illustrates.

This sample application connects to ROS, but does not have any Isaac node other than the bridge.
So, it is not useful when it is run as is. It is meant to be a sample on how to create
a bridge. Users can implement a bridge codelet like this one and add it to
their application graph.

Platforms: Desktop, Jetson Xavier

Hardware requirements: none
