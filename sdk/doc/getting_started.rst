..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

Getting Started
==================================

This section provides pointers on how to get started with developing and running Isaac applications.


Tutorials and Sample Applications
----------------------------------

There is a :ref:`tutorials_webinar` available that explains how to develop an application by
writing a codelet and creating an application graph.

There are over 30 tutorials and samples provided with Isaac SDK to get you started.
You can learn more about sample applications in the :ref:`tutorials_samples` section.

For example, a sample application can be run a easily as executing the following command:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel build //apps/samples/stereo_dummy


.. _running_an_app:

Running an Application
---------------------------------

This section explains how to run the stereo_dummy sample application on your desktop. All other
applications can be run in a similar matter by using the corresponding bazel target name of the
application.

A bazel target name for example has the following form: :code:`//app/samples/stereo_dummy`. This refers to
the application :code:`stereo_dummy` in the folder :code:`app/samples/stereo_dummy`. If you want to run a
different application you have to change the target name correspondingly.

Note that all bazel build and bazel run commands should be executed at the root folder of your
repository. For example if your root folder is :code:`/home/bob/isaac` you first go to the directory
:code:`/home/bob/isaac` and then run the commands mentioned below.

1. Build the sample application with the following command on the host system:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel build //apps/samples/stereo_dummy

2. Run the sample application with the following command:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel run //apps/samples/stereo_dummy

  The :code:`bazel run` command first builds and then runs the application. If you want to run an
  application :code:`bazel run` is enough. Remember to run :code:`bazel run` at the root folder of
  your repository as explained above.

3. Once you have an application running, open the visualization frontend by opening
:samp:`http://localhost:3000` in a browser.

In case you want to run an application with additional configuration parameters you can pass these
configuration files on the command line as follows:

  .. code-block:: bash

     bob@desktop:~/isaac/sdk$ bazel run //apps/samples/stereo_dummy -- --config more_config.json

Note the double dash :code:`--` to separate arguments to bazel from arguments to the application.

The following bazel commands are useful:

* Build and run an application in GDB for debugging (in this example, pose_tensorrt_inference)
  with the following commands:

  .. code-block:: bash

     bob@desktop:~/isaac/sdk$ bazel build -c dbg //apps/samples/pose_tensorrt_inference
     bob@desktop:~/isaac/sdk$ gdb --args bazel-bin/engine/alice/tools/main --app apps/samples/pose_tensorrt_inference/pose_tensorrt_inference.app.json

* Build everything:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel build ...

* Build only one target:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel build //engine/gems/filters/examples:ekf_sin_exp

* Run all tests:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel test ... --jobs=1

* Run one test:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel test //engine/gems/optimization/tests:pso_ackley

* Run linter checks:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel test --config=lint ...

  The linter requires Python 2.7. If all files fail the linter tests, the Python path is
  probably the issue. Try:

  .. code-block:: bash

    bob@desktop:~/isaac/sdk$ bazel test --python_path=/usr/bin/python2.7 --config=lint ...

.. _deployment_device:


Deploying and Running on Jetson
----------------------------------

This section briefly explains how to deploy an application from your desktop machine to the robot
and how to run it.

The Isaac SDK fully supports cross-compilation for Jetson. Compiling the source code on Jetson
itself is not recommended.

The following two steps need to be run only once:

1. Make sure you have an SSH key on your desktop machine.

2. Copy your SSH identity to the robot using the user name and IP you use to login on the robot with
   a command similar to the following:

  .. code-block:: bash

     bob@desktop:~/isaac/sdk$ ssh-copy-id <username_on_robot>@<robot_ip>

   where <username_on_robot> is your user name on the robot, and <robot_ip> is the IP address of the
   robot.

  You might need to connect the robot to a screen to get its IP address.

3. To run the stereo_dummy sample application on the robot, first deploy the package to the robot
   with the following command:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ ./../engine/engine/build/deploy.sh --remote_user <username_on_robot> -p //apps/samples/stereo_dummy:stereo_dummy-pkg -d jetpack44 -h <robot_ip>

   where <robot_ip> is the IP address of the robot and <username_on_robot> is your user name on the
   robot.

   .. note:: If a username is not specified with the --remote_user option, the default username used is :code:`nvidia`.

   The :code:`-d jetpack44` option specifies that we are building and deploying to a Jetson device
   with Jetpack version 4.4.1.

4. Login to the robot to run the application:

  .. code-block:: bash

     bob@jetson:~/isaac$ ssh ROBOTUSER@ROBOTIP

5. Go to the deployment folder and run the application:

  .. code-block:: bash

      bob@jetson:~/$ cd deploy/bob/stereo_dummy-pkg
      bob@jetson:~/deploy/bob/stereo_dummy-pkg$ ./apps/samples/stereo_dummy/stereo_dummy

   Here "bob" is the user name you use on your host system. You can deploy under a different
   folder by specifying :code:`-u OTHER_USER` to :code:`deploy.sh` in step 3.

  .. note:: To automatically run the application on the robot after deployment, run deploy.sh with
            the -\\-run (or -r) option, as follows (from the :code:`sdk/` subdirectory):

            .. code-block:: bash

               ./../engine/engine/build/deploy.sh --remote_user <username_on_robot> -p //apps/samples/stereo_dummy:stereo_dummy-pkg \
                                                -d jetpack44 -h <robot_ip> --run

            Using the -\\-run option in step 3 causes deploy.sh to effectively perform steps 4 and 5
            for you.

6. Once the application is running, connect to it in your browser and inspect the running
   application with websight. To do so navigate to :samp:`http://ROBOTIP:3000` in your browser.

In case you want to run an application with additional configuration parameters you can pass these
configuration files on the command line with the  the :code:`--config` option as follows:

  .. code-block:: bash

     bob@desktop:~/isaac/sdk$ ./apps/samples/stereo_dummy/stereo_dummy --config more_config.json


Python Application Support
---------------------------------

The Isaac SDK provides basic Python support. However support for Python is in an experimental
state in this release. The API is not stable or feature complete.

The tutorial in :code:`//apps/samples/ping_python` is similar to the ping tutorial for C++ but
implemented in Python. It is available on all platforms and does not require any hardware.

Running a Python application on a desktop system is identical to running a C++ application as
explained in section :ref:`running_an_app`. Deploying a Python app to Jetson is identical to
deploying a C++ application as explained in :ref:`deployment_device`.

However running a Python application on Jetson is slightly different. Using the :code:`run` script
is necessary to set certain environment variables required for Python. In step 5 of `Deploying and
Running on Jetson`_, use the following commands to run an application:

.. code-block:: bash

   bob@jetson:~/$ cd deploy/bob/ping_python-pkg
   bob@jetson:~/deploy/bob/ping_python-pkg$ ./run ./apps/tutorials/ping_python/ping_python.py

Where "bob" is you username on your desktop system.


Using a Distributed Workspace
---------------------------------

To manage your code in separate workspace, see the example
:samp:`https://github.com/nvidia-isaac/velodyne_lidar` and fork it as starting point, if desired,
with the following steps:

1. Download Isaac SDK and extract the TAR archive to a preferred folder.

2. Download the velodyne_lidar repository at the link above or fork it as desired.

3. Open the :code:`WORKSPACE` file in the velodyne_lidar repository,
   change the workspace name as desired and specify the path to the Isaac SDK workspaces for
   :code:`com_nvidia_isaac_engine` and :code:`com_nvidia_isaac_sdk`.

4. Test the setup by building and running the workspace with the following commands:

  .. code-block:: bash

     bob@desktop:~/velodyne_lidar$ bazel build ...
     bob@desktop:~/velodyne_lidar$ bazel run //packages/velodyne_lidar/apps:vlp16_sample

5. Write more code!

.. _using_docker:

Using Docker
------------

Isaac SDK development can be done in Docker container, allowing teams to use a standard environment,
and use that environment inside of non-Linux operating systems, such as Windows and Mac OS. This
section describes how to build and run an Isaac SDK Docker container.

Installing Dependencies
~~~~~~~~~~~~~~~~~~~~~~~

1. Install off-the-shelf docker with the following command:

.. code-block:: bash

   bob@desktop:~/isaac/sdk$ ./engine/engine/build/docker/install_docker.sh

2. Install NVIDIA docker over off-the-shelf Docker by following the steps in the `Installation Guide
   <https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)>`_.


Creating an Isaac SDK Development Image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. After installing dependencies, run the following script to create the ``isaacbuild`` image for
   Isaac SDK development:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ ./engine/engine/build/docker/create_image.sh

2. Create a cache volume for faster builds with the following command:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ docker volume create isaac-sdk-build-cache

3. Run the container with the following command:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ docker run --mount source=isaac-sdk-build-cache,target=/root -v `pwd`:/src/workspace -w /src/workspace --runtime=nvidia -it isaacbuild:latest /bin/bash

4. Run the following command inside the container to build Isaac SDK:

   .. code-block:: bash

      bob@desktop:~/isaac/sdk$ bazel build ...
