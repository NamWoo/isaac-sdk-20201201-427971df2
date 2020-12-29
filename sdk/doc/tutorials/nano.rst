..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _get-started-nano:

Getting Started With Jetson Nano
================================

This section describes how to run Isaac SDK sample applications on the Jetson Nano device. For
directions on how to get started with Nano in general, see `Getting Started with the Jetson Nano
Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit>`_.

.. _run-samples-nano:

Getting the IP Address
----------------------

Obtain the IP address of Jetson Nano:

1. Connect a keyboard, mouse, and display, and boot the device as shown in the `Setup
and First Boot <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#setup>`_
section of `Getting Started with the Jetson Nano Developer Kit`.

2. At a terminal prompt, enter the following command:

   .. code-block:: bash

      bob@jetson:~/$ ip addr show


Running Sample Applications on Jetson Nano
------------------------------------------

This section describes the steps to run sample applications on Jetson Nano. The first sample does
not require any peripherals. The second sample is a more useful application that requires a
connected camera. The third sample demonstrates how to deploy a TensorFlow model and run inference
on the device.

Other applications can be deployed and run using the methods described here.

Ping
^^^^

1. Deploy :code:`//apps/tutorials/ping:ping-pkg to the robot as explained` in :ref:`deployment_device`.

2. Change to the directory on your Jetson Nano and run the application with the following commands:

  .. code-block:: bash

    bob@jetson:~/$ cd deploy/<bob>/ping-pkg/
    bob@jetson:~/deploy/<bob>/ping-pkg$ ./apps/tutorials/ping/ping

Where <bob> is your username on the host system. You should see "Hello World!"
print every 1.5 seconds.

OpenCV Edge Detection
^^^^^^^^^^^^^^^^^^^^^

1. For this sample, connect a camera to one of the USB ports on the Jetson Nano.

2. Deploy :code:`//apps/tutorials/opencv_edge_detection:opencv_edge_detection-pkg` to the robot as
   explained in :ref:`deployment_device`.

3. Change to the directory on your Jetson Nano and run the application with the following commands:

  .. code-block:: bash

    bob@jetson:~/$ cd deploy/<bob>/opencv_edge_detection-pkg/
    bob@jetson:~/deploy/<bob>/opencv_edge_detection-pkg/$ ./apps/tutorials/opencv_edge_detection/opencv_edge_detection

4. To view the results, load :samp:`http://<nano_ip>:3000/` in your browser. Make sure that the
   application is running when you are loading the webpage.

TensorFlow Model Inference with TensorRT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Deploy :code:`//apps/samples/ball_segmentation:inference_tensorrt-pkg` to the robot as explained
   in :ref:`deployment_device`.

2. Change to the directory on your Jetson Nano and run the application with the following commands:

  .. code-block:: bash

    bob@jetson:~/$ cd deploy/<bob>/inference_tensorrt-pkg/
    bob@jetson:~/deploy/<bob>/inference_tensorrt-pkg/$ ./apps/samples/ball_segmentation/inference_tensorrt

3. To view the results, load :samp:`http://<nano_ip>:3000/` in your browser. Make sure that the
   application is running when you are loading the webpage.


Frequently Asked Questions
--------------------------

How to Flash Jetson Nano
^^^^^^^^^^^^^^^^^^^^^^^^

Nano can be flashed in two ways:

- Via an SD Card: See the procedures in `Getting Started with the Jetson Nano Developer Kit
  <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit>`_.
- Via SDK Manager: See the procedures in `SDK Manager User Guide
  <https://docs.nvidia.com/sdk-manager>`_

Use the documents above to determine the best method for your use case.

How to Connect a Wi-Fi/Bluetooth Card
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Intel 8265 card is used for Wi-Fi and Bluetooth connectivity. The following steps describe how
to install a Wi-Fi/Bluetooth card for Jetson Nano.

1. To access the M.2 slot on the carrier board, remove the two screws on the side and open the
   SODIMM latches using both your hands.

   .. image:: images/nano-wifi-1.jpg

2. When the Jetson Nano module pops up, slide it out gently.

   .. image:: images/nano-wifi-2.jpg

3. Take out the Intel wireless card and attach the antenna to its U.FL sockets before inserting the
   card into the M.2 socket. Attaching the antenna on the sockets requires patience and getting
   used to. Use your nail to gently apply force. You do not need much force to clamp it
   in once you are in the right position.

   .. image:: images/nano-wifi-3.jpg

4. Slide the Intel 8265 card into the socket.

   .. image:: images/nano-wifi-3.jpg

5. Fix the Intel 8265 card in place with a screw, and replace the Jetson Nano module. Make
   sure to use the correct screws in each case.

   .. image:: images/nano-wifi-4.jpg