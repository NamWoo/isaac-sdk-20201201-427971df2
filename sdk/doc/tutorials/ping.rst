..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _cplusplus_ping:

Developing Codelets in C++
==========================

The goal of this tutorial is to develop two codelets in C++: The first is effectively a machine that
goes "ping", while the second listens for and ingests the "ping" message. For this tutorial,
no external dependencies or special hardware is required.

Creating a New Application
--------------------------

Each Issac Robotics application requires an application JSON file and a Bazel build file. We will
create these files first before writing the codelets.

Create a new directory for the application
******************************************

Create a folder in the :code:`isaac/sdk/packages` directory with a command similar to the following:

.. code-block:: bash

   bob@desktop:~/isaac/sdk/packages/$ mkdir ping

For the rest of the tutorial, whenever you are asked to create a new file, place it directly into
this folder. More complicated packages will have subfolders, but for this tutorial things are kept
simple.

Create an application JSON file
*******************************

Every Isaac application is based on a JSON file, which describes the dependencies of the
application, the node graph, and the message flow; it also contains custom configuration data.
Create a new JSON file called ``ping.app.json`` and specify its name as seen in the following
snippet:

.. code-block:: python

  {
    "name": "ping"
  }

Create a Bazel build file
*************************

Next, create a Bazel build file for compiling and running the application. Bazel provides very good
dependency management and excellent build speed for large projects, and Bazel build files are very
easy to write. Create a file named ``BUILD`` with a new app target named ``ping``, as shown below:

.. code-block:: bash

    load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

    isaac_app(
         name = "ping"
    )

Build the application with Bazel
********************************

Now you can build the application by running the following command in the ``ping`` directory:

.. code-block:: bash

    bob@desktop:~/isaac/sdk/packages/ping$ bazel build ping

From now on, when you are asked to execute a command, execute it in the *ping* directory.

The command may take some time as all external dependencies for the Isaac Robot Engine are
downloaded and compiled. After a while, the first build should succeed with output similar
to the following:

.. code-block:: bash

   bob@desktop:~/isaac/sdk/packages/ping$ bazel build ping
   Starting local Bazel server and connecting to it...
   INFO: Analysed target //packages/ping:ping (54 packages loaded, 2821 targets configured).
   INFO: Found 1 target...
   Target //packages/ping:ping up-to-date:
     bazel-genfiles/packages/ping/run_ping
     bazel-bin/ping/packages/ping
   INFO: Elapsed time: 112.170s, Critical Path: 30.14s, Remote (0.00% of the time):
   [queue: 0.00%, setup: 0.00%, process: 0.00%]
   INFO: 691 processes: 662 linux-sandbox, 29 local.
   INFO: Build completed successfully, 1187 total actions

Next you can run your new application by executing the following command:

.. code-block:: bash

   bob@desktop:~/isaac/sdk/packages/ping$ bazel run ping

This will start the ping application and keep it running. You can stop a running application by
pressing **Ctrl+C** in the console. This will gracefully shut down the application.

You will notice that not much is happening because we don't have an application graph yet.
Next we will create some nodes for the application.

Creating a Node
----------------------

An Isaac application consists of many nodes running in parallel. They can send each other messages
or interact with each other using various other mechanisms provided by the Isaac Robot Engine. Nodes
are lightweight and do not require their own processes, or even their own threads.

To customize the behavior of the ping node, we have to equip it with components. We will create our
own component called "Ping".

Create a codelet .hpp file
****************************

Create a new file called ``Ping.hpp`` in the ``ping`` directory, with the following contents:

.. code-block:: cpp

    #pragma once
    #include "engine/alice/alice_codelet.hpp"
    class Ping : public isaac::alice::Codelet {
     public:
      void start() override;
      void tick() override;
      void stop() override;
    };
    ISAAC_ALICE_REGISTER_CODELET(Ping);

Codelets provide three main functions, which can be overloaded: ``start``, ``tick`` and ``stop``.
When a node is started, the start functions of all attached codelets are called first.
For example, ``start`` is a good place to allocate resources. You can configure a codelet to
``tick`` periodically or each time a new message is received. Most of the functionality is then
performed by the ``tick`` function.

At the end, when a node stops, the ``stop`` function is called. You should free all previously
allocated resources in the ``stop`` function. Do not use constructors or destructors: You do not
have access to any Isaac Robot Engine functionality (such as configuration) in the constructor.

Each custom codelet you create needs to be registered with the Isaac Robot Engine. This is done at
the end of the file using the ``ISAAC_ALICE_REGISTER_CODELET`` macro. If your codelet is inside a
namespace, you have to provide the fully qualified type name, for example
``ISAAC_ALICE_REGISTER_CODELET(foo::bar::MyCodelet);``.

Create a codelet .cpp File
****************************

To add some functionality to the codelet, create a source file called ``Ping.cpp``, which contains
this functionality:

.. code-block:: cpp

    #include "Ping.hpp"
    void Ping::start() {}
    void Ping::tick() {}
    void Ping::stop() {}

Define the tick() behavior
**************************

Codelets can tick in different ways, but for now we will use periodic ticking, which can be achieved
by calling the ``tickPeriodically`` function in the ``Ping::start`` function. Add the following
code to the ``start`` function in ``Ping.cpp``:

.. code-block:: cpp

    void Ping::start() {
      tickPeriodically();
    }

Add a log message
*****************

To verify that something is in fact happening, we will print a message when the codelet ticks. The
Isaac SDK includes utility functions for logging data; ``LOG_INFO`` can be used to print a message
on the console. It follows the `printf-style syntax <https://en.cppreference.com/w/cpp/io/c/fprintf>`_.
Add the ``tick`` function to *Ping.cpp* as shown below:

.. code-block:: cpp

    void Ping::tick() {
      LOG_INFO("ping");
    }

Add the component to the BUILD file
***********************************

Add the component to the BUILD file as a module, as shown below:

.. code-block:: python

    isaac_app(
      ...
    )

    isaac_cc_module(
      name = "ping_components",
      srcs = ["Ping.cpp"],
      hdrs = ["Ping.hpp"],
    )

An Isaac module defines a shared library that encapsulates a set of codelets and can be used by
different applications.

Add a new node to the JSON File
*********************************

To use the Ping codelet in the application, we first need to create a new node in the
application JSON file:

.. code-block:: python

    {
      "name": "ping",
      "graph": {
        "nodes": [
          {
            "name": "ping",
            "components": []
          }
        ],
        "edges": []
      }
    }

Add the Ping codelet to the node
********************************

Each node can contain multiple components, which define its functionality. Add the Ping codelet to
the node by adding a new section in the components array:

.. code-block:: python

    {
      "name": "ping",
      "graph": {
        "nodes": [
          {
            "name": "ping",
            "components": [
              {
                "name": "ping",
                "type": "Ping"
              }
             ]
          }
        ],
        "edges": []
      }
    }

An application graph normally has edges connecting different nodes, which determine the
message-passing sequence between nodes. Because this application does not have any other
nodes, we will leave the edges blank.

Add the component to the modules lists
**************************************

If you try to run this application, it will panic and show the error message
`Could not load component 'Ping'`. This happens because all components used in an application
must be added to the modules list. You need to do this both in the BUILD file and in the application
JSON file:

.. code-block:: python

    load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

    isaac_app(
        name = "ping",
        modules = ["//packages/ping:ping_components"]
    )

.. code-block:: python

    {
      "name": "ping",
      "modules": [
        "ping:ping_components"
      ],
      "graph": {
        ...
      }
    }

Note that the expression ``ping:ping_components`` refers to the module
``//package/ping:ping_components``, which we created previously.

If you run the application now, you will get a different panic message:
"Parameter 'ping/ping/tick_period' not found or wrong type". This message appears because we need
to set the tick period of the Ping codelet in the configuration section. We will do this in the next
section.


Configuration
-------------

Most code requires various parameters for customizing behavior. For example, you might want to give
the user of our ping machine the option to change the tick period. In the Isaac framework, this can
be achieved with configuration.

Let's specify the tick period in the "config" section of the application JSON file so that we
can finally run the application.

.. code-block:: python

    {
      "name": "ping",
      "modules": [
        "ping:ping_components"
      ],
      "graph": {
        ...
      },
      "config": {
        "ping" : {
          "ping" : {
            "tick_period" : "1Hz"
          }
        }
      }
    }

Every configuration parameter is referenced with three elements: node name, component name, and
parameter name. In this case we are setting the parameter ``tick_period`` of the component ``ping``
in the node ``ping``.

.. Note:: Configuration values must match the data type specified in the component API. See the
          :ref:`component_api_documentation` or the component *.hpp* file for the expected data
          type. Also note that an integer value is accepted as a type of double value.

Now the application will run successfully and print :code:`ping` once a second. You should see
output similar to the snippet below. You can gracefully stop the application by pressing **Ctrl+C**.

.. code-block:: bash

    bob@desktop:~/isaac/sdk/packages/ping$ bazel run ping
    2019-03-24 17:09:39.726 DEBUG   engine/alice/backend/codelet_backend.cpp@61: Starting codelet 'ping/ping' ...
    2019-03-24 17:09:39.726 DEBUG   engine/alice/backend/codelet_backend.cpp@73: Starting codelet 'ping/ping' DONE
    2019-03-24 17:09:39.726 DEBUG   engine/alice/backend/codelet_backend.cpp@291: Starting job for codelet 'ping/ping'
    2019-03-24 17:09:39.726 INFO    packages/ping/Ping.cpp@8: ping
    2019-03-24 17:09:40.727 INFO    packages/ping/Ping.cpp@8: ping
    2019-03-24 17:09:41.726 INFO    packages/ping/Ping.cpp@8: ping

Add a new parameter to the codelet .hpp file
********************************************

The ``tick_period`` parameter is automatically created for us, but we can also create our own
parameters to customize the behavior of codelets. Add a parameter to your codelet as shown below:

.. code-block:: cpp

   class Ping : public isaac::alice::Codelet {
    public:
     void start() override;
     void tick() override;
     void stop() override;
     ISAAC_PARAM(std::string, message, "Hello World!");
   };

``ISAAC_PARAM`` takes three arguments:

1. The type of the parameter, usually ``double``, ``int``, ``bool``, or ``std::string``.
2. The name of the parameter, which is used to access or specify the parameter.
3. The default value of the parameter. If no default value is given, and the parameter is not
   specified via a configuration file, the program asserts when the parameter is accessed.

The ``ISAAC_PARAM`` macro creates an accessor called ``get_message`` and a bit more code to properly
connect the parameter with the rest of the system.

Use the new parameter from the codelet .hpp file
************************************************

We can now use the parameter in the ``tick()`` function instead of the hard-coded value. Call
``get_message()`` to retrieve the value of the ``message`` parameter:

.. code-block:: cpp

    void tick() {
      LOG_INFO(get_message().c_str());
    }

Configure the parameter in the JSON file
****************************************

The next step is to add the configuration for the node. Use the node name (:code:`ping`),
component name (:code:`ping`), and the parameter name (:code:`message`) to specify the desired
value.

.. code-block:: python

    {
      "name": "ping",
      "modules": [
        "ping:ping_components"
      ],
      "graph": {
        ...
      },
      "config": {
        "ping" : {
          "ping" : {
            "message": "My own hello world!",
            "tick_period" : "1Hz"
          }
        }
      }
    }

That's it! You now have an application that can periodically print a custom message. Run the
application with the following command:

.. code-block:: bash

    bob@desktop:~/isaac/sdk/packages/ping$ bazel run ping

As expected, the codelet prints the message periodically on the command line.


Sending Messages
----------------

The custom codelet Ping is now happily ticking. For other nodes to react to the ping, the
Ping codelet must send a message that other codelets can receive.

Add the ISAAC_PROTO_TX macro to the codelet .hpp file
*******************************************************

Publishing a message is easy. Use the ``ISAAC_PROTO_TX`` macro to specify that a codelet is
publishing a message. Add it to ``Ping.hpp`` as shown below:

.. code-block:: cpp

    #pragma once

    #include "engine/alice/alice.hpp"
    #include "messages/ping.capnp.h"

    class Ping : public isaac::alice::Codelet {
     public:
      ...

      ISAAC_PARAM(std::string, message, "Hello World!");
      ISAAC_PROTO_TX(PingProto, ping);
    };

   ISAAC_ALICE_REGISTER_CODELET(Ping);

The ``ISAAC_PROTO_TX`` macro takes two arguments. The first argument specifies the message to
publish. Here, use the PingProto message, which comes with the :ref:`Isaac message API <message_api_documentation>`.
Access PingProto by including the corresponding header file. The second argument specifies the name
of the channel under which we want to publish the message.


Modify the tick() function in the codelet .cpp file
*****************************************************

Next, change the ``tick()`` function to publish a message instead of printing to the console. The
Isaac SDK currently supports `cap’n’proto <https://capnproto.org/>`__ messages. Protos are a
platform- and language-independent way of representing and serializing data. Creating a message is
initiated by calling the ``initProto`` function on the accessor that the ``ISAAC_PROTO_TX`` macro
created. This function returns a cap’n’proto builder object, which can be used to write data
directly to the proto.

The ``ProtoPing`` message has a field called ``message`` of type string, so in this instance we can
use the ``.setMessage()`` function to write some text to the proto. After the proto is populated, we
can send the message by calling the publish function. This immediately sends the message to any
connected receivers.

Change the ``.tick()`` function in ``Ping.cpp`` to the following:

.. code-block:: cpp

    ...
    void Ping::tick() {
      // create and publish a ping message
      auto proto = tx_ping().initProto();
      proto.setMessage(get_message());
      tx_ping().publish();
    }
    ...

Add the MessageLedger component to the node
*******************************************

Lastly, upgrade the node (in the JSON file) to support message passing. Nodes in Isaac SDK are
by default light-weight objects requiring minimal setup of mandatory components, and some nodes in
your application may not need to send or receive messages.

To enable message passing on a node, we need to add a component called ``MessageLedger``. This
component handles incoming and outgoing messages and relays them to ``MessageLedger`` components
in other nodes.

.. code-block:: python

    {
      "name": "ping",
      "graph": {
        "nodes": [
          {
            "name": "ping",
            "components": [
              {
                "name": "message_ledger",
                "type": "isaac::alice::MessageLedger"
              },
              {
                "name": "ping",
                "type": "Ping"
              }
             ]
          }
        ],
        "edges": []
    },
    "config": {
      ...
    }

Build and run the application. It appears that nothing happens because right now nothing is
connected to your channel. While you are publishing a message, no one is there to receive it and
react to it. We will fix that in the next section.

Receiving Messages
------------------

You need a node that can receive the ping message and react to it in some way. For this purpose,
let's create a ``Pong`` codelet, which is triggered by the message sent by ``Ping``.

Create a codelet .hpp file for Pong
*************************************

Create a new file named ``Pong.hpp`` with the following contents:

.. code-block:: cpp

    #pragma once
    #include "engine/alice/alice.hpp"
    #include "messages/ping.capnp.h"

    class Pong : public isaac::alice::Codelet {
     public:
      void start() override;
      void tick() override;

      // An incoming message channel on which we receive pings.
      ISAAC_PROTO_RX(PingProto, trigger);

      // Specifies how many times we print 'PONG' when we are triggered
      ISAAC_PARAM(int, count, 3);
    };

    ISAAC_ALICE_REGISTER_CODELET(Pong);

Add the Pong component to the BUILD file
****************************************

The Pong codelets need to be added to the ``ping_components`` module in order to be compiled. Add
them to the BUILD file as shown below (we will create the ``Pong.cpp`` file later in this section):

.. code-block:: python

    isaac_cc_module(
      name = "ping_components",
      srcs = [
        "Ping.cpp",
        "Pong.cpp"
      ],
      hdrs = [
        "Ping.hpp",
        "Pong.hpp"
      ],
    )

Create a Pong node in the JSON file
***********************************

In the application JSON file, create a second node and attach the new Pong codelet to it.

.. code-block:: python

      "nodes": [
        {
          "name": "ping",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "ping",
              "type": "Ping"
            }
          ]
        },
        {
          "name": "pong",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "pong",
              "type": "Pong"
            }
          ]
        }
      ],

Add an edge to the JSON file
****************************

Edges connect receiving RX channels to transmitting TX channels. A transmitting channel can
transmit data to multiple receivers. A receiving channel can also receive data from multiple
transmitters; however, this comes with caveats and is discouraged.

Similar to parameters, channels
are referenced with three elements: node name, component name, and channel name. An edge can be
created by adding it to the :code:`edges` section in the application JSON file. Here
:code:`source` is the full name of the transmitting channel and :code:`target` is the full name
of the receiving channel.

Connect the Ping and the Pong nodes using an edge:

.. code-block:: python

  {
    "name": "ping",
    "modules": [
      "ping:ping_components"
    ],
    "graph": {
      "nodes": [
        {
          "name": "ping",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "ping",
              "type": "Ping"
            }
          ]
        },
        {
          "name": "pong",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "pong",
              "type": "Pong"
            }
          ]
        }
      ],
      "edges": [
        {
          "source": "ping/ping/ping",
          "target": "pong/pong/trigger"
        }
      ]
    },
    "config": {
      "ping" : {
        "ping" : {
          "message": "My own hello world!",
          "tick_period" : "1Hz"
        }
      }
    }
  }

Create a codelet .cpp for Pong
********************************

The last remaining task is to set up the Pong codelet to do something when it receives the ping.
Create a new file named ``Pong.cpp``. Call the ``tickOnMessage()`` function in ``start()`` to
instruct the codelet to tick each time it receives a new message on that channel. In ``tick()``, we
add the functionality to print out "PONG!" as many times as defined by the ``count`` parameter in
the Pong header file:

.. code-block:: cpp

  #include "Pong.hpp"

  #include <cstdio>

  void Pong::start() {
    tickOnMessage(rx_trigger());
  }

  void Pong::tick() {
    // Parse the message we received
    auto proto = rx_trigger().getProto();
    const std::string message = proto.getMessage();

    // Print the desired number of 'PONG!' to the console
    const int num_beeps = get_count();
    std::printf("%s:", message.c_str());
    for (int i = 0; i < num_beeps; i++) {
      std::printf(" PONG!");
    }
    if (num_beeps > 0) {
      std::printf("\n");
    }
  }

By using ``tickOnMessage()`` instead of ``tickPeriodically()``, we instruct the codelet to only
tick when a new message is received on the incoming data channel, in this case ``trigger``. The tick
function now only executes when you receive a new message. This is guaranteed by the Isaac Robot
Engine.

Run the application. You should see that a "pong" is generated every time the Pong codelet receives
a ping message from the Ping codelet. By changing the parameters in the configuration file, you can
change the interval at which a ping is created, alter the message that is sent together with each
ping, and print pong more or less often whenever a ping is received.

Sending Messages Over a Network
-------------------------------

If the ``Ping`` and ``Pong`` components run on different devices, we need network connections.
The ``TcpPublisher`` and  ``TcpSubscriber`` nodes facilitate network connections as shown below:

.. code-block:: python

  {
    "name": "ping",
    "modules": ["engine_tcp_udp"],
    "graph": {
      "nodes": [
        ...
        {
          "name": "pub",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "tcp_publisher",
              "type": "isaac::alice::TcpPublisher"
            }
          ]
        }
      ],
      "edges": [
        {
          "source": "ping/ping/ping",
          "target": "pub/tcp_publisher/tunnel"
        }
      ]
    },
    "config": {
      ...
      "pub": {
        "tcp_publisher": {
          "port": 5005
        }
      }
    }
  }

The ``port`` parameter specifies the network port for accepting a connection. Make sure it is
available on the device. On the other end, ``TcpSubscriber`` can deliver messages when set up in
the JSON file as shown below:

.. code-block:: python

  {
    "name": "pong",
    ...
    "graph": {
      "nodes": [
        ...
        {
          "name": "sub",
          "components": [
            {
              "name": "message_ledger",
              "type": "isaac::alice::MessageLedger"
            },
            {
              "name": "tcp_receiver",
              "type": "isaac::alice::TcpSubscriber"
            }
          ]
        }
      ],
      "edges": [
        {
          "source": "sub/tcp_receiver/tunnel",
          "target": "pong/pong/trigger"
        }
      ]
    },
    "config": {
      ...
      "sub": {
        "tcp_receiver": {
          "port": 5005,
          "reconnect_interval": 0.5,
          "host": "127.0.0.1"
        }
      }
    }
  }

The ``host`` parameter specifies the IP address to listen to. Make sure ``host`` and ``port``
specify the open port and IP address of the device where ``Ping`` is running. Run these applications
on separate devices to see messages communicated through the network.

This is just quick start with a very simple application. A real-world application consists of
dozens of nodes, each with multiple components with one or more codelets. Codelets receive
multiple types of messages, call specialized libraries to solve hard computational problems, and
publish their results again to be consumed by other nodes.
