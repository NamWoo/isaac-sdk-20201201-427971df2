..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _kaya_software:

Running Isaac SDK on Kaya
=========================

Once you have built your own :ref:`kaya_hardware` robot, follow the steps on this page to run some
sample applications on it.

Installation and Setup
----------------------

Complete these prerequisite steps before running applications on Kaya:

1. Install the Jetson operating system on your Jetson Nano as described in
   the :ref:`get-started-nano` guide.

2. Obtain the IP address of the robot as described in the :ref:`get-started-nano` guide.

3. Follow the :ref:`setup-isaac` guide to install Isaac SDK, along with all of its dependencies,
   on the Jetson Nano.

4. Follow the steps in the :ref:`deployment_device` section to register your SSH key with Kaya.

The Joystick Application
---------------------------------

The following steps deploy a simple joystick application that uses a paired PS4 controller to
control Kaya. The same steps can be used for deploying and running other applications on your
robot.

1. To run the :code:`//apps/kaya:joystick` joystick application on Kaya, deploy the package from
   your desktop to Kaya with the following command:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ ./../engine/engine/build/deploy.sh --remote_user <username_on_robot> -p //apps/kaya:joystick-pkg -d jetpack44 -h <robot_ip>

   where :code:`<robot_ip>` is the IP address of the robot and :code:`<username_on_robot>` is your
   username on Kaya.

   .. note:: If a username is not specified with the :code:`--remote_user` option, the username
      defaults to :code:`nvidia`.

   The :code:`-d jetpack44` option specifies that you are building and deploying to a Jetson device
   with Jetpack version 4.4.

2. Log in to Kaya:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ ssh <username_on_robot>@<robot_ip>

3. Go to the deployment folder on Kaya and run the application:

   .. code-block:: bash

      bob@jetson:~/$ cd deploy/<bob>/joystick-pkg/
      bob@jetson:~/deploy/<bob>/joystick-pkg$ ./apps/kaya/joystick

   Where :code:`<bob>` is your username on the host system.

By default, the L1 button on the controller functions as a deadman trigger. The robot will follow joystick
commands only when the trigger is pressed.

The Follow Me Application
---------------------------------

The following steps deploy the Follow Me application, which moves Kaya autonomously towards a
designated AprilTag fiducial. This application combines AprilTag detection, path planning, control,
and the Kaya driver. It requires a working Intel RealSense camera for AprilTag detection and path
planning and a joystick to enable autonomous mode.

1. Deploy :code:`//apps/kaya:follow_me-pkg` to Kaya as shown in step 1 of the Joystick Application.

2. Switch to the directory on Kaya and run the application with the following commands:

   .. code-block:: bash

      cd deploy/<your_username>/follow_me-pkg/
      ./apps/kaya/follow_me

3. Open Isaac Sight on the desktop browser at :samp:`<robot_ip>:3000`. In the **Application
   Configuration** panel on the right, click on "fiducial_as_goal" and change the "target_fiducial_id"
   value to the ID of the AprilTag in use.

If you put the AprilTag within the field of view of the RealSense camera, you should see in the
**Follower Kaya - Camera** window that the AprilTag is detected and a planned path, displayed as
a blue line, connects Kaya to the AprilTag.

If you hold down the R1 button on the controller, Kaya will enter autonomous mode and move
towards the AprilTag. Move the AprilTag around while keeping it within view of the camera, and
Kaya will follow it around.


The Object Detection Application
---------------------------------

This application uses DetectNetv2 on Kaya to perform
object detection. The DetectNetv2 model is trained to recognize a tennis ball.
Here are the steps for running this application.

1. Deploy :code:`//apps/kaya:object_detection-pkg` to Kaya as shown in step 1 of the
   Joystick Application..

2. Switch to the directory on Kaya and run the application with the following commands:

   .. code-block:: bash

      cd deploy/<your_username>/object_detection-pkg/
      ./apps/kaya/object_detection_kaya

   Open Isaac Sight on the desktop browser at :samp:`<robot_ip>:3000`. You should see a
   live camera image.

3. Place a tennis ball in front of Kaya's camera, and you should see a bounding box around
   the object labeled "tennis_ball". This sample was trained on a limited dataset and is not
   guaranteed to work in every situation and lighting condition. To improve model accuracy in a
   custom environment, see the instructions to :ref:`train on custom models in docker<training_in_docker>`
   or the :ref:`object detection training documentation<object_detection_with_detect_net>`


The Mapping Application
---------------------------------

The :ref:`GMapping application <gmapping_application>` demonstrates how to use Kaya to build an
occupancy map of its environment. Kaya perceives the depth of obstacles with its RealSense camera
and updates its own state using the wheel odometry and IMU. It then publishes the obstacle depth and
robot state information to the host, which builds a map.

This application has two parts: :code:`gmapping_distributed_kaya`, which runs on Kaya, and
:code:`gmapping_distributed_host`, which runs on the host machine.

1. Deploy :code:`//apps/kaya:gmapping_distributed_kaya-pkg` to Kaya as shown in step 1 of the
   Joystick Application.

2. Switch to the directory on Kaya and run the application with the following commands:

   .. code-block:: bash

      cd deploy/<your_username>/gmapping_distributed_kaya-pkg/
      ./apps/kaya/gmapping_distributed_kaya

   Open Isaac Sight on the desktop browser at :code:`<nano_ip>:3000`. You should see the
   camera RGB and depth image, as well as a plot of the robot state.

3. Use the joystick to drive Kaya around. You should see the robot state being updated.

4. On the desktop system, open :code:`app/kaya/gmapping_distributed_host.app.json` and change the
   tcp-subscriber host from :code:`"YOUR_NANO_IP_HERE"` to the IP address of the Jetson Nano on
   Kaya.

5. Build and run the host application with the following commands:

   .. code-block:: bash

      bazel build //apps/kaya:gmapping_distributed_host
      bazel run //apps/kaya:gmapping_distributed_host

6. Open Isaac Sight on the desktop at :samp:`localhost:3000`. You should see the map that Kaya
   is building. Use the joystick to drive Kaya around and observe the map updates.

If you see performance issues with the map updates, go to the Kaya page on Isaac Sight and
disable camera RGB and depth visualization by unchecking **Channels - viewer**.

