..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _breaking-changes:

Breaking Changes
================

**October 2020**

* All the navigation apps inside Isaac SDK have been ported over to use evidential grid maps (EGMs).
  EGMs provide a cleaner local map and natively account for uncertainty while providing interesting
  fusion rules like the Proportional Conflict Redistribution fusion rule (PCR6).

  The subgraph `packages/navigation/apps/local_map.subgraph.json` has been replaced by
  `packages/egm_fusion/evidence_grid_map.subgraph.json`.

  There is a 1:1 correspondence between the older Bayesian map and newer evidential map codelets.
  However some parameters and channels have been modified.

  To migrate, replace the codelets as :

  .. code-block:: cpp

     navigation::LocalMap -> egm_fusion::EvidenceMapFusion
     navigation::OccupancyMapCleanup -> egm_fusion::EvidenceMapInpaint
     navigation::OccupancyToBinaryMap -> egm_fusion::EvidenceToBinaryMap

* When specifying the flatscan frame names, the GridSearchLocalizer now
  accepts a list of strings rather than a single string. Each represents
  the frame name of the flatscan source's origin.

* The flatscan message channels to use now need to be specified
  explicitly via the GridSearchLocalizer parameter
  `flatscan_channel_names`. They must be in the same order as the frame
  names from `flatscan_frames`. Only unique entries are allowed.

* Both above lists need to have the same amount of entries.

Migration from previous setups:

* For single flatscan source setups, set the `flatscan_frames` parameter
  to a list with the only entry being the desired flatscan frame. Also,
  set `flatscan_channel_names` to a list with the only entry being the
  message channel used for receiving flatscan messages.

* For multi flatscan source setups, extent above lists to the number of
  flatscan sources used, and keep both lists in order (the indices of
  the frames are associated to the indices of then channel names).


**September**

* Migration to CUDA 10.2 and cuDNN 8.0.3. This changes the system requirements for Isaac SDK from
  JetPack 4.3 to JetPack 4.4.

* Split into multiple workspaces: The folders :code:`engine` and :code:`sdk` now contain separate
  parts of Isaac SDK. :code:`sdk` depends on :code:`engine`, and references to elements that reside
  in :code:`engine` must be preceded by :code:`@com_nvidia_isaac_engine`.


**August 2020**

* Added :code:`read_message_proto_bytes()` and :code:`set_message_proto_bytes()` to the C API
  package.


**July 2020**

* Moved the implementation of DifferentialBaseModel as DifferentialWheelModel and rewrote it.

  Before:

  .. code-block:: cpp

     planner::DifferentialBaseModel model;
     planner::WheelDynamics wheel_dynamics;
     model.computeBodyDynamicsFromWheelDynamics(wheel_dynamics);

  Now:

  .. code-block:: cpp

     planner::DifferentialBaseModel model;
     planner::DifferentialWheelModel current_copy = model.current_model();
     messages::DifferentialWheelVAState wheel_dynamics;
     current_copy.computeBodyDynamicsFromWheelDynamics(wheel_dynamics);


* Engine now uses new StagingQueue for message queues.

* The functions :code:`readAllLatest()`, :code:`readAllNew()`, :code:`peekAllNew()`,
  :code:`readLatestNew()`, :code:`peekLatestNew()`, and :code:`checkChannelMessages()` were removed
  from MessageLedger. Use the new queue and its functions like :code:`peek()` and :code:`pop()`
  instead.

* Moved Tcp/Udp components to separate packages.

* In order to use the :code:`TcpPublisher`, :code:`TcpSubscriber`, :code:`UdpPublisher`, or
  :code:`UdpSubscriber` components, the new package :code:`engine_tcp_udp` needs to be loaded.


**June 2020**

* Moved the Python API and related tests to the :code:`pyalice` package.

* Added dependencies of :code:`pyalice` for all :code:`isaac_py_app` applications.

* The :code:`import isaac.*` command is now supported in the Python API.

* Updated all imports accordingly.

* Added :code:`pyalice` to :code:`PYTHONPATH` when running :code:`pytest` on Jetson CI.

* The following codelets were moved to a new module :code:`libcask_module.so` and
  thus also to the new namespace :code:`isaac::cask`:

  .. code-block:: cpp

     isaac::alice::Recorder
     isaac::alice::Replay
     isaac::alice::RecorderBridge
     isaac::alice::ReplayBridge

* The following subgraphs were moved to :code:`packages/casks/apps`:

  .. code-block:: cpp

     packages/record_replay/apps:record.subgraph.json
     packages/record_replay/apps:replay.subgraph.json


* The codelet :code:`StereoVisualOdometry` was moved to package :code:`visual_slam`.


**January 2020**

* Bazel 2.2.0 was introduced as the required build environment.
