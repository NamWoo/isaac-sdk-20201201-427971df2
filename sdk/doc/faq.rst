..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _isaac_faq:

Frequently Asked Questions
==========================

This section provides answers to frequently asked questions about NVIDIA Isaac SDK™. Use it
as the first step in troubleshooting problems.

To ask and answer questions in the Isaac SDK developer community, visit the `Isaac SDK Forum <https://devtalk.nvidia.com/default/board/375/isaac-sdk/>`_.


Hardware Components
-----------------------------------------------------------------

**When I run my application that uses a ZED Camera, I get the following error:**

.. code-block:: bash

   engine/alice/backend/modules.cpp@74: dlopen failed: libhidapi-libusb.so.0: cannot open shared
   object file: No such file or directory

The :code:`libusb-dev` library is not installed. Install :code:`libusb-dev` and all other required
dependencies on the robot with the following command:

.. code-block:: bash

  bob@desktop:~/isaac/engine/$ ./engine/build/scripts/install_dependencies_jetson.sh -h <jetson_ip> -u <jetson_username>

Where :code:`<jetson_ip>` is the IP address of the robot.

**I am running my app on Kaya. Everything runs fine, but the joystick does not move the robot.**

Make sure you press the "deadman switch" L1 on the joystick while using the direction knobs.
Pressing L1 is required to prevent unwanted movements.

**When I run my application that uses a BMI160 IMU, I get the following error:**

.. code-block:: bash

    I2C Error: Device or resource busy (errno 16)

The kernel module :code:`nvs_bmi160` is using the I2C resource needed by the application. Unload
the module with the command :code:`sudo rmmod nvs_bmi160`. You can also run the
:code:`install_dependencies_jetson.sh` script, which prevents :code:`nvs_bmi160` from loading.

.. code-block:: bash

  bob@desktop:~/isaac/engine/$ ./engine/build/scripts/install_dependencies_jetson.sh -h <jetson_ip> -u <jetson_username>

Visualization and WebSight
-----------------------------------------------

**I don't see images in Sight. Or the images are blank. Or the labels displayed are red/yellow.**

There are a number of reasons why a channel may not update correctly:

* **The channel is not enabled in the bottom left menu**. If the channel
  is not enabled, the robot does not send the update to the frontend and the channel appears in
  red in the renderer's legend. Enable this channel from the channel menu
  directly or right click on the renderer and click **enable all channels**.

* **The channel is enabled, but there is no valid transformation between the channel and the
  reference frame used by the renderer**. If this is the case, the channel appears in yellow
  in the renderer's legend. If you hover over the channel name, additional information is provided
  about the missing transformation: '[reference_T_channel] is not defined'. You can check the
  PoseTree widget to figure out if there is a path between both coordinate frames.

  .. note:: ''#####'' is used as a default frame. It means that no frame has been provided
            for the channel.)

* **Your image/channel is rendered, but a later channel is overriding it**. Channels are rendered
  in the order they appear in the legend: the top one first, followed by second one, and so on. If
  you render an image the size of the renderer, it covers all the previous channels. Make sure
  your channels are in the right order. You can change the order by right clicking the
  renderer and clicking **settings**. From there you can use the arrows on the left side of the
  channels to update the order.

* **The timestamp of the channel is in the future**. Each show operation in Sight has its own
  timestamp (if none is provided, the current time is used). In general, it is good practice to use
  the acquisition time to render images or objects; it helps to synchronize the channels. Sight uses
  the application time (seconds since the beginning) to render the different channels. If the timestamp
  of a channel is in the future (either because the acquisition time used is not relative to the start
  of the application or because the wrong unit is used), then the channel is not updated until the
  application time catches up with the channel.

  This case can be detected by refreshing the web page. If the image/channel updates but then
  freezes, it means that Sight received the latest message but is waiting for the time to catch up
  before rendering.

  If you own the codelet rendering the image, make sure that you are using the right time in your
  show call. If you do not own the codelet, then the acquisition time of one of the
  channels is most likely used to render. Make sure the publisher provides the right acquisition
  time.

**I am having lag issues in Sight.**

See below for different types of lag issue:

* **All the Sight widgets are lagging**. You are most likely streaming too much data to Sight and
  your network connection can't handle the load. First, try enabling only the channels
  you are interested in visualizing. To achieve this quickly, first unselect all the channels by
  clicking on the root of the channel list (left panel). You can then re-enable all the channels of
  a given renderer by right clicking on it and selecting **Enable all channels**.

* **I enabled only the channels I need, but Sight is still lagging**. You are probably looking at a
  channel that requires a lot information. It could come from either a channel with a lot of drawing
  primitives (PointCloudViewer for example) or large images. Most of the Isaac default viewers have
  an option to downscale the amount of data sent to Sight (CameraViewer has a `reduce_scale`
  parameter and PointCloudViewer has a`skip` parameter). One way to figure out how much bandwidth a
  given channel is using is to look at the `Channel Statistics`. To open it, click on `Channels` in
  the left panel. Once the widget is open, you can have a look at the bandwidth/frequency and other
  information about how much data are streamed to sight. Disable very demanding channels
  when they are not needed. If you need to visualize a channel with a high bandwidth, you may need
  to skip some messages. One way to achieve this result is to reduce the total allowed bandwidth
  from the websight config (type `WebsightServer` in the search box of the config panel on the
  right, then modify the `bandwidth` parameter).

* **Sight is rendering with a decent framerate, but some channels seem to be lagging behind**. By
  default, each renderer renders with a delay of 0.2s to allow all the channels to arrive. However,
  0.2s might not be enough for channels where the duration from sensor acquisition to Sight is more
  than 0.2s. You can adjust the rendering delay of each renderer individually by right-clicking the
  renderer and hovering over the `Change delay` option: a slider will appear, allowing you to
  adjust the delay from 0 to 2s.

* **The bounding box for ML detections do not match the image**. If you have tried the solutions
  above and they did not help, check that each detection matches an image and each image has its
  own detection. If you reduced the frame rate of the CameraViewer, it is possible a detection is
  being rendered without its matching image. On the other hand, if the detection pipeline can't
  keep up with the camera framerate, it is possible some frames are not being processed. Try
  reducing the frame rate of the camera to make sure no frames are skipped.

**Unable to see the Sight webpage after opening http://localhost:3000 in a browser.**

Make sure an application is running on your desktop. If you are running the application on the
robot, you must use the robot IP address.

If the `"Failed to start Webserver!"` error message appears in the logs, the port may not
have been released by a previous application.

Use the following command to determine the application using the port:

.. code-block:: bash

   lsof -i TCP:3000

You can then kill the application with the following command:

.. code-block:: bash

   killall -9 <app name>

If you need to keep the app running, update the configuration file to
change the port to another open port.

**Sight visualization is very slow.**

When network bandwidth is insufficient, channels displayed in Sight may exhibit latency or very
low framerate. Follow these steps to improve bandwidth:

1. In Sight, under **Channels**, uncheck all unnecessary channels.

   .. tip:: You can disable all channels at once by clicking on the application name in the channel
            menu. Enable all channels you want to visualize by right clicking a renderer and selecting
            **enable all channels**.

2. Verify Wi-Fi antennas and cables are connected to the PCIe Wi-Fi card.
3. Determine whether power-saving mode is enabled on the Wi-Fi adapter:

    .. code-block:: bash

     cat /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
     [connection]
     wifi.powersave = 3

   Set the ``wifi.powersave`` value to 2 to disable power-saving mode:

    .. code-block:: bash

     wifi.powersave = 2

4. Try a wired connection to rule out Wi-Fi bandwidth issues.
5. If Sight has been running for a long time, it might have accumulated a lot of data, which
   slows down the frontend. Try refreshing from time to time to see if it improves performance.

Navigation Stack
-----------------------------------------------

**The global planner is taking forever to start or to run**

If the global planner is too slow, you can try to reduce some parameters:

- ``graph_initialization_steps``: The number of random samplings done in the
  start function.
- ``graph_in_tick_steps``: The number of random samplings done each tick. If the
  graph is already dense enough, this parameter might be set to 0.

If reducing the number of random samplings compromises the quality of the
navigation graph produced, consider pre-computing the graph and loading it from a file. To
produce this graph, follow these steps:

1. Run the application with ``graph_initialization_steps`` set to a high value (you can also set
   ``graph_in_tick_steps`` to a high value to keep increasing the graph each tick).
2. Choose a destination to save the current graph and update the ``graph_file_out`` parameter.
3. Wait for the graph to be large enough. Then kill the app.
4. You can now restart the app and set the ``graph_file_in`` parameter to the file containing the graph.

**The global planner can't find a path or produces a suboptimal path**

If your map is large, you probably need to generate a large graph. You can increase the
``graph_initialization_steps`` parameter or load the graph from a file (see the steps described
in the question above).

Other Questions
---------------

**What is the easiest way to generate a capnp id for a new message?**

Every capnp file requires a unique ID at the beginning of the file. If you create a new capnp
file and attempt to build Isaac without an ID in your new file, an ID is generated and printed in
the error message, similar to the following:

.. code-block:: none

   messages/my_new.capnp:1:1: error: File does not declare an ID. I've generated one for you.
   Add this line to your file: @0xcdeac1e381086f01;

As instructed by the error message above, add an "@" symbol, then the generated ID, then a semicolon to the top of
your capnp file (if you don't add the semicolon, you will recieve a parse error).

**When running an application that requires TensorFlow, I get the following error:
PANIC   engine/core/buffers/algorithm_cuda.cpp@55: Could not copy memory. Error: 35**

To resolve this error, install CUDA 10.0 using the instructions at `NVIDIA CUDA Installation Guide
for Linux <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#abstract>`_.

**When using a USB Camera on a TX2 I see no image or a heavily distorted image.**

This happens when the CPU clock speed is too low on the TX2 and it cannot process the USB frames in
time. Try running the :code:`jetson_clocks` command to increase the CPU clock speed. See `this NVIDIA Developer Forums topic
<https://devtalk.nvidia.com/default/topic/1055732/jetson-tx2/usb-port-disabled-when-no-hdmi-connected-/post/5362568/#5362568>`_ for more details.

.. _pose_3d_syntax:

**What is the syntax for setting a Pose3d in json?**

Earlier versions of Isaac SDK expected an array of 7 numbers to set a Pose3d in JSON. The
old syntax, which is still supported, looks as follows:

.. code::

  [qw, qx, qy, qz, x, y, z]

The first four numbers form the quaternion while the latter three are the translation values.

Some users may find the new format more convenient. For details on the syntax, please see
:code:`engine/gems/serialization/json_formatter.hpp` and for examples please check
:code:`engine/gems/serialization/tests/json_formatter.cpp`. Below are some equivalent poses:

+-----------------------------------------+---------------------------------------------+
| old syntax (supported)                  | new syntax                                  |
| Use square brackets                     | Use curly brackets                          |
+=========================================+=============================================+
|                                         |                                             |
| .. code-block:: json                    | .. code-block:: json                        |
|                                         |                                             |
|   [0.0, 0.0, 0.0, 1.0, 25.0, 20.0, 0.0] |   {                                         |
|                                         |      "translation": [25.0, 20.0, 0.0],      |
|                                         |      "rotation": {                          |
|                                         |         "qw": 0.0,                          |
|                                         |         "qx": 0.0,                          |
|                                         |         "qy": 0.0,                          |
|                                         |         "qz": 1.0                           |
|                                         |      }                                      |
|                                         |   }                                         |
+-----------------------------------------+---------------------------------------------+
|                                         | .. code-block:: json                        |
| .. code-block:: json                    |                                             |
|                                         |   {                                         |
|   [0.0, 0.0, 0.0, 1.0, 25.0, 20.0, 0.0] |      "translation": [25.0, 20.0, 0.0],      |
|                                         |                                             |
|                                         |      "rotation": { "yaw_degrees": 180 }     |
|                                         |   }                                         |
+-----------------------------------------+---------------------------------------------+
|                                         |                                             |
| .. code-block:: json                    | .. code-block:: json                        |
|                                         |                                             |
|   [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]   |   {                                         |
|                                         |      "rotation": { "yaw_radians": 3.14159 } |
|                                         |   }                                         |
+-----------------------------------------+---------------------------------------------+
|                                         |                                             |
| .. code-block:: json                    | .. code-block:: json                        |
|                                         |                                             |
|   [1.0, 0.0, 0.0, 0.0, 25.0, 20.0, 0.0] |   {                                         |
|                                         |      "translation": [25.0, 20.0, 0.0]       |
|                                         |   }                                         |
+-----------------------------------------+---------------------------------------------+

**How can I convert my log files that contain deprecated types like ColorCameraProto?**

//packages/cask/apps:cask_converter can be used to convert log files with deprecated types.
Please see help from this application.

Troubleshooting
---------------

**What should I check first if my build is failing?**

Make sure you are running Ubuntu 18.04 on your host system, and that you have run the
`install_dependencies.sh` script.

**Why am I seeing "crosstool" errors when running Bazel build?**

If you see the following errors during Bazel build, your Bazel version is out of date:

.. code-block:: bash

   ERROR: @bazel_tools//tools/build_defs/cc:action_names.bzl' does not contain symbol 'ACTION_NAMES
   ERROR: error loading package '@toolchain//crosstool': Extension file 'crosstool/cc_toolchain_config.bzl' has errors

Use the :code:`engine/build/scripts/install_dependencies.sh` script inside :code:`engine` directory
to upgrade Bazel to the current version required by Isaac SDK. To perform a manual update, see the
GitHub page for the Bazel installer.

**When using a USB Camera on a TX2 I see no image or a heavily distorted image.**

This happens when the CPU clock speed is too low on the TX2 and it cannot process the USB frames in
time. Try running the :code:`jetson_clocks` command to increase the CPU clock speed. See `this NVIDIA Developer Forums topic
<https://devtalk.nvidia.com/default/topic/1055732/jetson-tx2/usb-port-disabled-when-no-hdmi-connected-/post/5362568/#5362568>`_
for more details.

**I am seeing an error similar to "libnppc.so.10.0: cannot open shared object file: No such file or directory"**

Some Isaac SDK code requires CUDA 10.0 to execute properly. Ensure CUDA 10.0 is installed
locally and the :code:`/usr/local/cuda-10.0/lib64` value is part of the :code:`LD_LIBRARY_PATH`
environment variable.

**I am seeing "Could not deserialize configuration parameter" errors when I run an application.**

Configuration values must match the data type specified in the component API. See the :ref:`component_api_documentation`
or the component *.hpp* file for the expected data type.

Note that an integer value is accepted as a type of double value.

How to Add a New External Dependency
------------------------------------

Your packages or components may require external dependencies not yet available in Isaac. The right
strategy for integrating a new external dependency can vary case by case. There are several examples
in the `third_party` folder. This tutorial explains how to add the zlib library.

1. Find a reliable source for your dependency. In the case of zlib, get it from the official zlib webpage at
   `zlib.net <https://zlib.net>`_. To avoid unexpected problems in your codebase when the external dependency is updated,
   fix a specific version of the library. This tutorial chooses the version `1.2.11`.
2. Add the dependency to your WORKSPACE file using the Bazel rule ``isaac_http_archive``:

   .. code-block:: python

    isaac_http_archive(
        name = "zlib",
        build_file = clean_dep("//zlib.BUILD"),
        sha256 = "c3e5e9...cb1a1",
        strip_prefix = "zlib-1.2.11",
        url = "https://zlib.net/zlib-1.2.11.tar.gz",
    )

   Importing external dependencies as an archive is generally preferred over using the rules
   ``new_git_repository`` or ``git_repository``, which import git repositories, because archives are
   smaller and have less overhead than git repositories.
3. For the purposes of this tutorial, assume that you have to write a BUILD file for the new
   external dependency (this may not always be required). In the case of zlib, this is quite
   straightforward:

   .. code-block:: python

    cc_library(
      name = "zlib",
      srcs = glob(["*.c", "*.h"], exclude="zlib.h"),
      hdrs = ["zlib.h"],
      copts = [
        "-Wno-shift-negative-value",
        "-Wno-implicit-function-declaration",
      ],
      includes = ["."],
    )

   This BUILD file defines a single C++ library named ``zlib`` using the Bazel rule ``cc_library``. The
   library includes all C source and header files from zlib and compiles them into a single library.
   The only external header file is ``zlib.h``, which is excluded from the sources so that the same file
   does not appear as a source and a header.

   In general, ``srcs`` is used for files internal to the library and can be used for both source and
   header files, while ``hdrs`` is used for publicly facing header files that are required in applications
   that use the library.

   A couple of compiler flags are also required. Add them using the ``copts`` option to avoid compiler
   warnings that are treated as errors due to use of the `-Wall` option by the Isaac compilation tool.

The new library is ready for use as a dependency in one of your libraries or binaries, similar to
the following example:

.. code-block:: python

  cc_library(
    name = "foo",
    srcs = ["foo.cpp"]
    hdrs = ["foo.hpp"],
    deps = ["@zlib"]
  )

Every external dependency creates its namespace with the same name as the one used for the
``isaac_http_archive`` or corresponding rule. In this case, the name of both the dependency and the
library is `zlib` and thus the shortcut ``@zlib`` can be used to refer to the library.
If there were a second library ``foo`` inside the same archive, the explicit form ``@zlib//:foo`` would
be needed to reference it.

How To Analyze a Crash using Minidump
-------------------------------------

In case of a crash, Isaac SDK applications attempt to collect information with `breakpad
<https://github.com/google/breakpad>`_. This information is written into a minidump file in binary
format. This section explains how to extract human-readable information from minidump files.


Prepare the Minidump Toolkit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepare the minidump toolkit in :code:`/tmp/minidump/` with the following command:

.. code-block:: bash

    bob@desktop:~/isaac/engine/$ ./engine/build/scripts/prepare_minidump_tools.sh

Prepare Symbols
^^^^^^^^^^^^^^^

Prepare symbols before testing applications by passing the argument ``-s`` when deploying applications
as shown in the following command:

.. code-block:: bash

    bob@desktop:~/isaac/sdk/$ ./../engine/engine/build/deploy.sh -s -h <robot_ip> -d <device> -p <target>
    --remote_user <username_on_robot>

where :code:`<robot_ip>` is the IP address of the robot and :code:`<username_on_robot>` is your username on the
robot.


Analyzing Minidumps
^^^^^^^^^^^^^^^^^^^

When a crash occurs, Isaac application minidump file paths are reported on the console similar to
the following:

.. code-block:: bash

    Minidump written to: /tmp/28db00a1-e756-4c47-62f6a7b6-fc26c1a0.dmp

To make the minidump data human-readable, run the following command on a desktop that has symbols prepared:

.. code-block:: bash

    bob@desktop:~/isaac/engine/$ ./engine/build/scripts/process_jetson_minidump.sh -h <robot_ip> -d
    /tmp/28db00a1-e756-4c47-62f6a7b6-fc26c1a0.dmp --remote_user <username_on_robot>

where :code:`<robot_ip>` is the IP address of the robot and :code:`<username_on_robot>` is your user name on the
robot. If the minidump file is locally available, run the following command:

.. code-block:: bash

    bob@desktop:~/isaac/engine/$ ./engine/build/scripts/process_minidump.sh <minidump>

Crash information is presented on the console, and is similar to the following:

.. code-block:: bash

    Crash reason:  SIGSEGV /SEGV_MAPERR
    Crash address: 0x0
    Process uptime: not available

    Thread 11 (crashed)
     0  realsense_camera!isaac::RealsenseCamera::tick() [RealsenseCamera.cpp : 533 + 0x4]
         x0 = 0x0000000000000000    x1 = 0x0000007f962b2180
         x2 = 0x0000007fa0000080    x3 = 0x0000007f800008d0
         x4 = 0x0000000000000007    x5 = 0x00000000007e8ac8
         x6 = 0x00000000007e89f0    x7 = 0x0000000000063762
         x8 = 0x00000000000000d7    x9 = 0x001dcd6500000000
        x10 = 0x000000005c787d6b   x11 = 0x000000000f171c90
        x12 = 0x0000000000000017   x13 = 0x000000005c787d6b
        x14 = 0x00076f61bec52c49   x15 = 0x00001c2c8948c9df
        x16 = 0x0000007fa7708688   x17 = 0x0000007fa7471838
        x18 = 0x0000007fa005bdcc   x19 = 0x0000000037cc4610
        x20 = 0x0000007fa004bbb0   x21 = 0x0000007fa0248c01
        x22 = 0x0000000037cc47e0   x23 = 0x00000006556cdce8
        x24 = 0x000000000091f000   x25 = 0x0000007fa53dd330
        x26 = 0x0000000000000000   x27 = 0x0000007f962b31b0
        x28 = 0x0000000000000001    fp = 0x0000007f962b20a0
         lr = 0x0000000000459f6c    sp = 0x0000007f962b20a0
         pc = 0x0000000000459f74
