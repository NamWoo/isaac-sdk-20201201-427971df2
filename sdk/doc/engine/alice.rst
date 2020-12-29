..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

Introduction to Isaac Robotics Engine
=====================================

.. _ping_cpp:

Basics
------

This section introduces Isaac Robotics Engine usage. It presents the relevant terminology and
explains the structure of Isaac applications.

An Isaac application is defined by a JavaScript Object Notation (JSON) file. The following example,
located in the Isaac SDK at :code:`apps/tutorials/ping/ping.app.json`, shows the basic format.

.. code-block:: javascript

  {
    "name": "ping",
    "modules": [
      "//apps/tutorials/ping:ping_components",
      "sight"
    ],
    "graph": {
      "nodes": [
        {
          "name": "ping",
          "components": [
            {
              "name": "ping",
              "type": "isaac::Ping"
            }
          ]
        }
      ],
      "edges": []
    },
    "config": {
      "ping" : {
        "ping" : {
          "tick_period" : "1Hz"
        }
      }
    }
  }

Isaac application is defined by four sections:

* :code:`name` is a string with the name of the application.

* :code:`modules` are a list of libraries in use. In the above example, we include
   :code:`//apps/tutorials/ping:ping_components` so that "apps/tutorials/ping/Ping.cpp". Is
   available. The Isaac SDK comes with high-quality and well-tested packages that we can import as
   modules. A longer ``modules`` example is shown later in this tutorial.

* :code:`graph` has two subsections to define the functionality of the application:

  * :code:`nodes` are the fundamental blocks of our application. In this simple example, we have just
    one node named "ping" that has a single component. Note that the type of this component,
    :code:`isaac::Ping`, matches the last line of :code:`apps/tutorials/ping/Ping.hpp`. A typical
    Isaac application has multiple nodes and each node typically has multiple components.

  * :code:`edges` connect components together and enable the communication between them. This example
    does not require any components to be connected.

* :code:`config` lets you tune the parameters of each node depending on our use case. In this example,
  it is specified that the ping component should tick at one hertz.

Having defined the application, next you need to create a makefile. The following is the
:code:`BUILD` file associated with the ping application.

.. code-block:: bash

    load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_app", "isaac_cc_module")

    isaac_cc_module(
        name = "ping_components",
        srcs = ["Ping.cpp"],
        hdrs = ["Ping.hpp"],
        visibility = ["//visibility:public"],
        deps = ["//engine/alice"],
    )

    isaac_app(
        name = "ping",
        data = ["fast_ping.json"],
        modules = [
            "//apps/tutorials/ping:ping_components",
        ],
    )

Codelets
--------
Codelets are the basic building blocks of a robotics application built with Isaac. The Isaac SDK
includes various codelets which can use in your application. Derive your codelets as illustrated in
the example below.

For a better understanding of codelets, see the :ref:`understanding_codelets` section.

The following listings of Ping.hpp and Ping.cpp show the source for the example codelet at
:code:`apps/tutorials/ping`:

.. code-block:: cpp

  // This is the header file located at apps/tutorials/ping/Ping.hpp

  #pragma once

  #include <string>

  #include "engine/alice/alice_codelet.hpp"

  namespace isaac {

  // A simple C++ codelet that prints periodically
  class Ping : public alice::Codelet {
   public:
    // Has whatever needs to be run in the beginning of the program
    void start() override;
    // Has whatever needs to be run repeatedly
    void tick() override;

    // Message to be printed at every tick
    ISAAC_PARAM(std::string, message, "Hello World!");
  };

  }  // namespace isaac

  ISAAC_ALICE_REGISTER_CODELET(isaac::Ping);

.. code-block:: cpp

  // This is the C++ file located at apps/tutorials/ping/Ping.cpp

  #include "Ping.hpp"

  namespace isaac {

  void Ping::start() {
    // This part will be run once in the beginning of the program

    // We can tick periodically, on every message, or blocking. The tick period is set in the
    // json ping.app.json file. You can for example change the value there to change the tick
    // frequency of the node.
    // Alternatively you can also overwrite configuration with an existing configuration file
    // like in the example file fast_ping.json. Run the application like this to use an
    // additional config file:
    //   bazel run //apps/tutorials/ping -- --config apps/tutorials/ping/fast_ping.json
    tickPeriodically();
  }

  void Ping::tick() {
    // This part will be run at every tick. We are ticking periodically in this example.

    // Print the desired message to the console
    LOG_INFO(get_message().c_str());
  }

  }  // namespace isaac

.. _p_control_cpp:

A Complete Application
^^^^^^^^^^^^^^^^^^^^^^

The application shown below is more powerful, featuring a graph with multiple nodes,
nodes with multiple components, edges between nodes, and codelets that receive and transmit
messages.

Look in the JSON file first to see how edges are defined. Note that this file is longer than the
ping example above, but it follows the very same syntax.

.. code-block:: javascript

  {
    "name": "proportional_control_cpp",
    "modules": [
      "//apps/tutorials/proportional_control_cpp:proportional_control_cpp_codelet",
      "navigation",
      "segway",
      "sight"
    ],
    "config": {
      "cpp_controller": {
        "isaac.ProportionalControlCpp": {
          "tick_period": "10ms"
        }
      },
      "segway_rmp": {
        "isaac.SegwayRmpDriver": {
          "ip": "192.168.0.40",
          "tick_period": "20ms",
          "speed_limit_angular": 1.0,
          "speed_limit_linear": 1.0,
          "flip_orientation": true
        },
        "isaac.alice.Failsafe": {
          "name": "segway"
        }
      },
      "diffbase_joystick": {
        "isaac.alice.FailsafeHeartbeat": {
          "interval": 0.25,
          "failsafe_name": "segway",
          "heartbeat_name": "deadman_switch"
        },
        "isaac.navigation.RobotRemoteControl": {
          "tick_period": "10ms"
        }
      },
      "odometry": {
        "isaac.navigation.DifferentialBaseOdometry": {
          "tick_period": "100Hz"
        }
      },
      "websight": {
        "WebsightServer": {
          "webroot": "packages/sight/webroot",
          "assetroot": "/home/nvidia/isaac-lfs/sight/assets",
          "port": 3000,
          "ui_config": {
            "windows": {
              "Proportional Control C++": {
                "renderer": "plot",
                "channels": [
                  {
                    "name": "proportional_control_cpp/cpp_controller/isaac.ProportionalControlCpp/reference (m)"
                  },
                  {
                    "name": "proportional_control_cpp/cpp_controller/isaac.ProportionalControlCpp/position (m)"
                  }
                ]
              }
            }
          }
        }
      }
    },
    "graph": {
      "nodes": [
        {
          "name": "cpp_controller",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "isaac.ProportionalControlCpp",
              "type": "isaac::ProportionalControlCpp"
            }
          ]
        },
        {
          "name": "segway_rmp",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "isaac.SegwayRmpDriver",
              "type": "isaac::SegwayRmpDriver"
            },
            {
              "name": "isaac.alice.Failsafe",
              "type": "isaac::alice::Failsafe"
            }
          ]
        },
        {
          "name": "odometry",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "isaac.navigation.DifferentialBaseOdometry",
              "type": "isaac::navigation::DifferentialBaseOdometry"
            }
          ]
        },
        {
          "name": "joystick",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "isaac.Joystick",
              "type": "isaac::Joystick"
            }
          ]
        },
        {
          "name": "diffbase_joystick",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "isaac.navigation.RobotRemoteControl",
              "type": "isaac::navigation::RobotRemoteControl"
            },
            {
              "name": "isaac.alice.FailsafeHeartbeat",
              "type": "isaac::alice::FailsafeHeartbeat"
            }
          ]
        }
      ],
      "edges": [
        {
          "source": "segway_rmp/isaac.SegwayRmpDriver/segway_state",
          "target": "odometry/isaac.navigation.DifferentialBaseOdometry/state"
        },
        {
          "source": "odometry/isaac.navigation.DifferentialBaseOdometry/odometry",
          "target": "cpp_controller/isaac.ProportionalControlCpp/odometry"
        },
        {
          "source": "cpp_controller/isaac.ProportionalControlCpp/cmd",
          "target": "diffbase_joystick/isaac.navigation.RobotRemoteControl/ctrl"
        },
        {
          "source": "joystick/isaac.Joystick/js_state",
          "target": "diffbase_joystick/isaac.navigation.RobotRemoteControl/js_state"
        },
        {
          "source": "diffbase_joystick/isaac.navigation.RobotRemoteControl/segway_cmd",
          "target": "segway_rmp/isaac.SegwayRmpDriver/segway_cmd"
        }
      ]
    }
  }

The file :code:`isaac_app` is quite similar to the ping example. However, modules like "segway" are
added that are required by this application.

.. code-block:: bash

  load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_app", "isaac_cc_module")

  isaac_cc_module(
      name = "proportional_control_cpp_codelet",
      srcs = ["ProportionalControlCpp.cpp"],
      hdrs = ["ProportionalControlCpp.hpp"],
      visibility = ["//visibility:public"],
      deps = [
          "//engine/alice",
          "//engine/gems/state:io",
          "//messages",
          "//messages/state:differential_base",
      ],
  )

  isaac_app(
      name = "proportional_control_cpp",
      app_json_file = "proportional_control_cpp.app.json",
      modules = [
          "//apps/tutorials/proportional_control_cpp:proportional_control_cpp_codelet",
          "navigation",
          "segway",
          "sensors:joystick",
          "viewers",
      ],
  )

The ProportionalControlCpp codelet below enables communication with other components via the
``ISAAC_PROTO_RX`` and ``ISAAC_PROTO_TX`` macros and associated ``edges`` in the JSON file.

.. code-block:: cpp

  // This is the header file located at
  // apps/tutorials/proportional_control_cpp/ProportionalControlCpp.hpp

  #pragma once

  #include "engine/alice/alice_codelet.hpp"
  #include "messages/differential_base.capnp.h"
  #include "messages/state.capnp.h"

  namespace isaac {

  // A C++ codelet for proportional control
  //
  // We receive odometry information, from which we extract the x position. Then, using refence
  // and gain parameters that are provided by the user, we compute and publish a linear speed
  // command using `control = gain * (reference - position)`
  class ProportionalControlCpp : public alice::Codelet {
   public:
    // Has whatever needs to be run in the beginning of the program
    void start() override;
    // Has whatever needs to be run repeatedly
    void tick() override;

    // List of messages this codelet receives
    ISAAC_PROTO_RX(Odometry2Proto, odometry);
    // List of messages this codelet transmits
    ISAAC_PROTO_TX(StateProto, cmd);

    // Gain for the proportional controller
    ISAAC_PARAM(double, gain, 1.0);
    // Reference for the controller
    ISAAC_PARAM(double, desired_position_meters, 1.0);
  };

  }  // namespace isaac

  ISAAC_ALICE_REGISTER_CODELET(isaac::ProportionalControlCpp);

.. code-block:: cpp

  // This is the C++ file located at
  // apps/tutorials/proportional_control_cpp/ProportionalControlCpp.cpp

  #include "ProportionalControlCpp.hpp"

  #include "engine/gems/state/io.hpp"
  #include "messages/math.hpp"
  #include "messages/state/differential_base.hpp"

  namespace isaac {

  void ProportionalControlCpp::start() {
    // This part will be run once in the beginning of the program

    // Print some information
    LOG_INFO("Please head to the Sight website at <IP>:<PORT> to see how I am doing.");
    LOG_INFO("<IP> is the Internet Protocol address where the app is running,");
    LOG_INFO("and <PORT> is set in the config file, typically to '3000'.");
    LOG_INFO("By default, local link is 'localhost:3000'.");

    // We can tick periodically, on every message, or blocking. See documentation for details.
    tickPeriodically();
  }

  void ProportionalControlCpp::tick() {
    // This part will be run at every tick. We are ticking periodically in this example.

    // Nothing to do if we haven't received odometry data yet
    if (!rx_odometry().available()) {
      return;
    }

    // Read parameters that can be set through Sight webpage
    const double reference = get_desired_position_meters();
    const double gain = get_gain();

    // Read odometry message received
    const auto& odom_reader = rx_odometry().getProto();
    const Pose2d odometry_T_robot = FromProto(odom_reader.getOdomTRobot());
    const double position = odometry_T_robot.translation.x();

    // Compute the control action
    const double control = gain * (reference - position);

    // Show some data in Sight
    show("reference (m)", reference);
    show("position (m)", position);
    show("control", control);
    show("gain", gain);

    // Publish control command
    navigation::DifferentialBaseControl command;
    command.linear_speed() = control;
    command.angular_speed() = 0.0;  // This simple example sets zero angular speed
    ToProto(command, tx_cmd().initProto());
    tx_cmd().publish();
  }

  }  // namespace isaac
