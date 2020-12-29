..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _pycodelet:

Developing Codelets in Python
=============================

While in terms of performance, the best language for writing codelets is C++, not all codelets of an
application need to be in the same language. The Isaac SDK also supports Python codelets, or
pyCodelets, for those who are more familiar with Python. This section shows you how to do the following:

- Run Python codelets, using `ping_python` included in the Isaac SDK as an example
- Create Python codelets

This section also describes the run script deployed with Python codelets to the target system, and
the differences between JSON and Bazel BUILD files for C++ codelets and JSON and Bazel BUILD files
for Python codelets.

Running a Python Codelet
^^^^^^^^^^^^^^^^^^^^^^^^

A Python version of the Ping codelet described in the :ref:`ping_cpp` section can be found in
the `apps/tutorials/ping_python/` directory.

This application can be run on your system by executing the following command:

.. code-block:: bash

   bob@desktop:~/isaac/sdk$ bazel run //apps/tutorials/ping_python

If you want to run the application on a Jetson device you can have to follow these instructions

1. Deploy //apps/tutorials/ping_python:ping_python-pkg to the robot as explained in
   :ref:`deployment_device`.

2. Change to the directory of the deployed package on Jetson with the following command:

   .. code-block:: bash

      bob@desktop:~/$ cd ~/deploy/bob/ping_python-pkg

      Where "bob" is your username on the host system.

3. Run the application by executing the following command:

   .. code-block:: bash

     bob@desktop:~/deploy/bob/ping_python-pkg/$ ./run apps/tutorials/ping_python/ping_python.py

When you run the codelet, by either method, a "Hello World!" message is printed every 1.5 seconds.
Modify the script at `apps/tutorials/ping_python/ping_python.py` and run it again to see the effects
of your changes.

.. note:: If a "ImportError: No module named capnp" error is displayed, make sure pycapnp is
          installed with the install_dependencies_jetson script mentioned in
          :ref:`install_dependencies_jetson`.

A more complete example, the Python version of the Proportional Control codelet described in the
:ref:`p_control_cpp` section is shown below. The following Python script is functionally equivalent
to a combination of `main.cpp`, `ProportionalControlCpp.hpp`, and `ProportionalControlCpp.cpp`:

   .. literalinclude:: ../../apps/tutorials/proportional_control_python/proportional_control_python.py
      :language: python

Import statements in Python are analogous to preprocessor `#include` statements in C++. Like in
`ProportionalControlCpp.cpp`, the codelet is defined in `start` and `tick` functions. The Isaac
parameters `desired_position_meters` and `gain` are used, with values either configured in JSON
files or set through Sight at runtime.

At every tick, if an odometry message is received, the appropriate command is computed and published
for the robot. Some important data is displayed in Sight.

The main function simply loads the graph and configuration files before running the application, the
way that `main.cpp` does in the C++ codelet. In addition, the Python main function also creates the
edges for the PyCodelet.

Creating Python Codelets
^^^^^^^^^^^^^^^^^^^^^^^^

Follow a procedure similar to the following when creating your Python codelets.

Create a Workspace
------------------

1. Copy `apps/tutorials/proportional_control_python/` or another existing Python-based codelet to
   `apps/<your_app_name>` as a template or starting point.

   If you use the Proportional Control codelet unmodified for this tutorial, a Carter robot or
   equivalent is required. See :ref:`carter_hardware` for more information.

2. Rename the files to reflect the name of your codelet instead of the codelet you copied.

Create a Bazel BUILD File
-------------------------

1. In `apps/<your_app_name>/BUILD` copied in with the other files used as a starting point, replace
   all `proportional_control_python` strings with `<your_app_name>`.

2. Modify the `data` property in the `py_binary` rule depending on the C++ codelets you use.

   For example, if you were to omit or remove `//packages/segway` in
   `apps/tutorials/proportional_control_python/BUILD`, and run the codelet, the error `Component with
   typename 'isaac::SegwayRmpDriver' not registered` would be displayed, because the Proportional
   Control codelet (`proportional_control_python.graph.json`) uses the C++
   based segway codelet.

3. Since our application is located in apps and not apps/tutorials, remove the specification of
   `//apps/tutorials:py_init`, leaving `//apps:py_init` in place.

   If instead of moving the application up to apps, you move it to `apps/tutorials/tutorials_sub`,
   the BUILD file in `apps/tutorials/tutorials_sub` must specify `py_init` in all three directories,
   `//apps:py_init`, `//apps/tutorials:py_init`, and `//apps/tutorials/tutorials_sub:py_init`. Each
   directory would also need a copy of `__init__.py`.

Create a Python Codelet
-----------------------

1. In your `<your_app_name>.py`, replace `import apps.tutorials.proportional_control_python` with
   `import apps.<your_app_name>`.

2. Rename and modify the `ProportionalControlPython` class as needed. You can define multiple Python
   codelets in this file.

3. In the main function, replace all `proportional_control_python` strings with <your_app_name>. You
   must register all pyCodelets using their class names, such as ProportionalControlPython in the
   files we used as a starting point. Modify node names, `py_controller` in this case, to match the
   name you chose in your `graph.json` file.

   Your main function will be similar to the following:

  .. code-block:: python

    def main():
      app = Application(app_filename = "my_new_app.app.json")
      app.register({"my_py_node1": PyCodeletType1, "my_py_node2a": PyCodeletType2, "my_py_node2b": PyCodeletType2})
      app.run()

4. Add or remove nodes, components, or edges in the graph section of
   `apps/<your_app_name>/<your_app_name>.app.json` depending on your codelet.

5. Configure nodes and components in the config section of
   `apps/<your_app_name>/<your_app_name>.app.json` as needed. Make sure to replace all instances
   of the codelet name with the name of your new codelet.

Run the codelet locally or deploy and run it on a Jetson system as described in `Running a
Python Codelet`_.

The run Script
^^^^^^^^^^^^^^

The run script, provided along with deployment (using deploy.sh) of an Isaac application that
includes a Python codelet or codelets, performs the following functions:

- Checks that the filename of the Python script ends in ".py"

- Verifies that every directory has an `__init__.py` file

- Runs the application using the following command.

  .. code-block:: bash

     PYTHONPATH=$PWD:$PWD/packages/pyalice:$PWD/packages/ml python

These functions are performed by the run script when we use the following command:

.. code-block:: bash

   ./run apps/tutorials/proportional_control_python/proportional_control_python.py

JSON and BUILD Files for Python Codelets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

JSON files for Python codelets are very similar to those for C++ codelets, except that the component
type of Python codelets is always `isaac::alice::PyCodelet`.

Bazel BUILD files are somewhat different, as shown in the following example:

.. code-block:: none

    load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_pkg")

    py_binary(
        name = "proportional_control_python",
        srcs = [
            "__init__.py",
            "proportional_control_python.py",
        ],
        data = [
            "proportional_control_python.config.json",
            "proportional_control_python.graph.json",
            "//apps:py_init",
            "//apps/tutorials:py_init",
            "//packages/navigation:libnavigation_module.so",
            "//packages/segway:libsegway_module.so",
            "//packages/sensors:libjoystick_module.so",
        ],
        deps = ["//packages/pyalice"],
    )

    isaac_pkg(
        name = "proportional_control_python-pkg",
        srcs = ["proportional_control_python"],
    )

Use of C++ codelets is enabled by specifying the corresponding modules in `data` in the `py_binary`
rule. For example, `//packages/segway:libsegway_module.so` is required to use C++ Codelet of type
`isaac::SegwayRmpDriver`. Omitting or forgetting this dependency causes the error `Component with
typename 'isaac::SegwayRmpDriver' not registered` to be displayed when the application is run.

The `isaac_pkg` rule is responsible for packing all the files up and creating an archive which
can be transferred to the target device with the deploy script.
