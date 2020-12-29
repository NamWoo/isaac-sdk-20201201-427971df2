..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _python-api:

Python API
=============================

Though most parts of Isaac SDK are coded in C++, you have the option to build your applications with
Python. This document introduces the Python API for Isaac SDK. The Python API allows you to do
the following:

* Create, manage, and run Isaac application in Python.

* Access recorded Isaac Log data in Python.

* Implement complicated robotics behavior with Behavior Tree in Python.


Creating Applications in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Isaac SDK includes several sample applications coded with the Python API, for example,
:code:'packages/flatsim/apps:flatsim'. The example below begins with a skeleton Python app.

To begin with, a Bazel target is needed along with the Python code itself:

   .. code-block:: python

      load("//engine/build:isaac.bzl", "isaac_py_app")

      isaac_py_app(
          name = "foo_app",
          srcs = [ "foo_app.py" ],
          data = [ "foo_subgraph" ],
          modules=[ 'viewers' ],
          deps = [ "//packages/pyalice" ],
      )

Along with common definitions like :code:`srcs` and :code:`data`, :code:`modules` pulls in
specified modules that come with Isaac SDK.

To run the app on a PC, use following command:

   .. code-block:: bash

      bazel run //apps/foo_app:foo_app

To run it on Jetson devices, first deploy it with the :code:`deploy.sh` script (from the
:code:`sdk/` subdirectory):

   .. code-block:: bash

      ./../engine/engine/build/deploy.sh -h <jetson_ip> -p //apps/foo_app:foo_app-pkg -d jetpack44

where :code:`jetson_ip` is the IP of the Jetson device.

Then, on the Jetson device, run this app:

   .. code-block:: bash

      ./run ./apps/foo_app/foo_app.py


To create an instance of Isaac Application with the Python API, import and create the instance
using 'engine.pyalice.Application'. As with C++, the :code:`init` function may take the path of the
application JSON file as an argument:

   .. code-block:: python

      from engine.pyalice import Application

      app = Application(name="foo_app")

With the application instance, you can load Isaac subgraphs as can be done with
:code:`*.app.json` files:

   .. code-block:: python

      app.load("apps/foo_app/foo.subgraph.json", prefix="foo")

Besides loading pre-authored computing graphs, Python API can make things more flexible.
For example, many codelets in Isaac SDK are provided by modules located in the :code:`packages`
folder, and these modules have to be loaded via :code:`*.app.json` and :code:`*.subgraph.json` files
just like in C++ applications:

   .. code-block:: json

      {
        "name": "foo_app",
        "modules": [
          "message_generators",
          "viewers"
        ]
      }

Here the :code:`message_generators` module provides dummy codelets that publish pre-configured
messages for testing purposes. The :code:`viewers` module provides codelets that visualizes messages
in Sight.

With the Python API, besides specifying modules in JSON files, you can also load modules when creating application
and/or when they are deemed necessary:

   .. code-block:: python

      app = Application(name="foo_app", modules=["message_generators"])
      app.load_module('viewers')

.. Note:: Ensure that the modules are loaded before creating instances from the
          codelets provided by the modules or loading any subgraph that uses the codelets.

Like its C++ counterpart, Application manages the computing graph with nodes consisting of
components. Now, let's create a node and attach a component from the :code:`ImageViewer`
codelet provided by the :code:`viewers` module we just loaded above:

   .. code-block:: python

      node = app.add(name='viewer')
      component = node.add(name='ImageViewer',
                           ctype=app.registry.isaac.viewers.ImageViewer)

Here, the :code:`app.add()` function returns a node instance while the :code:`node.add()` function
returns a component instance. These instances can also be retrieved as follows:

   .. code-block:: python

      node = app.nodes['viewer']
      component = node['ImageViewer']

Now set the config parameter of :code:`target_fps` to 15fps. Refer to the
:ref:`isaac.viewers.ImageViewer` API entry for details about its config parameters.

   .. code-block:: python

      component.config.target_fps = 15.0

Similarly, you can create a component from the :code:`CameraGenerator` codelet that publishes
messages and configure it as follows. Note that :code:`CameraGenerator` is provided by the
module of :code:`message_generators`, which needs to be loaded beforehand.

   .. code-block:: python

      image_node = app.add(name='camera')
      camera_generator = node.add(name='CameraGenerator',
                                  ctype=app.registry.isaac.message_generators.CameraGenerator)
      camera_generator.config.rows = 480
      camera_generator.config.cols = 640
      camera_generator.config.tick_period = '15Hz'

Now we have a generator component that publishes messages and a viewer component that visualizes
messages. We can connect these components so that the generated messages are sent to the
viewer component:

   .. code-block:: python

      app.connect(camera_generator, "color_left", component, "color")
      app.connect('camera/CameraGenerator', 'color_left', 'viewer/ImageViewer', 'image')

Here, the components can be specified either with the instance mentioned above or their names.

With the code above, we now have a complete application graph. You can run it with the :code:`run()`
function. Calling :code:`run()` without an argument allows it to run indefinitely. You can also
specify that it run for a certain duration (in seconds) or stop when a specific node is not running anymore:

   .. code-block:: python

      app.run()

      app.run(10.0)

      app.run('foo_node')

In all cases, pressing Ctrl-C will stop the application.


Accessing Cask Logs
^^^^^^^^^^^^^^^^^^^

Cask is the recording format used in Isaac SDK. Refer to ::ref:`rec-replay` for recording and
replaying logs. A sample application for recording logs can be found at
:code:`apps/samples/camera/record_dummy.py`.

Assuming that you have a recorded log in the :code:`/path/to/log/` folder, you can load the log in
Python as follows:

   .. code-block:: python

      from engine.pyalice import Cask, Message
      cask = Cask('/path/to/log/')

      # List all channels recorded
      series = cask.channels['foo_channel']:    # Looks for channel named 'foo_channel'
      for msg in series:                        # Goes through every messages one by one in recorded order
         print(msg.proto)
         print(msg.uuid)
         print(msg.acqtime)
         print(msg.pubtime)


Behavior Tree
^^^^^^^^^^^^^

Isaac SDK features a special module called Behavior Tree, which provides different codelets that can
be used to manage other codelets for complicated application behavior.
TimerBehavior, for example, can start a specific codelet and keep it running for a specified
duration before shutting it down. SwitchBehavior, on the other hand, could be used to switch
behavior between pre-configured modes.

Before creating and manipulating Behavior codelets, ensure the module is loaded:

   .. code-block:: python

      app.load_module('behavior_tree')

A Behavior codelet can also be managed by other Behaivor codelets--you can create quite complicated
functionality by stacking Behaviors.

You can achieve more flexibility by creating and configuring these Behavior codelets with the
Python API.


Please refer to ::ref:`pycodelet` for sample of developing codelets with Python.
