..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _scheduler:

The Isaac Scheduler
================================

The Isaac SDK uses a scheduler to manage and dispatch work, following the Earliest Deadline First
(EDF) model. The scheduler attempts to ensure that jobs are dispatched according to their desired
start times. The scheduler also manages multithreading concerns and core management. See the
`Scheduling Interface and Policy`_ section for details.

Jobs are specified by a configuration structure and referenced via opaque handles after creation.
Jobs are dispatched either to a pool of worker threads or executed on a dedicated thread. The
scheduler also tracks statistics for jobs when possible. See the `Job Description`_ and
`Job Statistics`_ sections for more information.

Thread and core management are configurable, and scheduler timing can be optimized with clock
scaling and a system called time machine. Time machine removes periods of idle time in the schedule
to accelerate training times. See the `Scheduler Configuration`_ and `Time Machine and Clock
Scaling`_ sections for more information.

All codelets use the scheduler implicitly, reducing the need to actively manage jobs. In most cases
it is unnecessary to create jobs directly. However, understanding the scheduler makes it easier to
optimize application graphs and codelet runtime performance.


Job Description
--------------------------------

The scheduler operates in terms of JobDescriptors and JobHandles. A JobDescriptor specifies how a
job is to be executed and a JobHandle is used to track a created job.

The JobDescriptor has the following fields:

* JobAction action: This is a function pointer which takes no arguments and has no return value.

* std::string execution_group: This optional field specifies which execution unit is responsible
  for executing the job. If no name is specified, a default group is assigned based on execution
  characteristics. Execution groups define relationships between jobs and hardware cores. See the
  `Scheduler Configuration`_ section for more information.

* std::string name: This optional field can hold a human-readable name to help with diagnostics.

* int64_t period: This field specifies the period of the job in nanoseconds. This is only applicable
  to jobs which are periodic. This field is mutually exclusive with target_start_time.

* int64_t target_start_time: This field specifies the target start time of the job in nanoseconds.
  It only applies to events and oneshot jobs.

* int64_t slack: This field specifies how much delay is allowable when scheduling a job. This is
  mainly used as a tiebreaker when two jobs are scheduled for the same time window.

* std::optional<int64_t> deadline: This value in nanoseconds is used to detect if a job is running
  longer than desired. By default, periodic jobs set this value to their period.

* int priority: The value in this field is used as a tie breaker if two jobs start in the same
  time window. Higher values have priority.

* ExecutionMode execution_mode: This field specifies how a job is executed. Options include the
  following:

    * kBlocking: The job is executed repeatedly on a dedicated thread. This is useful for dedicated
      hardware polling or other activities which must run repeatedly and have dedicated resources.

    * kBlockingOneShot: The job is executed once on a dedicated thread.

    * kOneShotTask: The job is executed once within the worker thread pool.

    * kPeriodicTask: The job is executed periodically within the worker thread pool.

    * kEventTask: The job is executed on event notification within the worker thread pool.

* EventTriggerLimit event_trigger_limit: This field specifies how receiving multiple events is
  handled. The job will not triggered if it is already queued more than often than the given limit.
  For example a limit of 1 means that the job can be queued at most once. If the job is currently
  executed it does not count against the limit. If set to -1 no limit will be used.

* bool has_statistics: This field controls if runtime statistics are collected for the job. Default
  behavior is to collect statistics.

See gems/scheduler/job_descriptor.hpp for more details.


Job Statistics
--------------------------------

Jobs which execute within the worker thread pools can collect behavior statistics. Jobs which
execute on their own threads do not collect statistics as there is no clearly defined way to measure
their performance. A summary report of all jobs executed is written to the log file at the end of
application execution.

The scheduler reports the median, as well as the upper bound of 90% of all
of the execution times. The job medians and 90th percentiles are more representative of the
execution behaviors than the average times because jobs can have long start up times, sporadically
long execution times, or optimized paths that can bias the average time estimates. In fact,
thread execution times may present non-Gaussian distributions. The metrics are estimates from
execution time subsamples. As a rule of thumb, a 90th percentile greater than three times the
median (which represents a heavy-tailed distribution) may warrant further analysis of the
job performance.

Below is a sample jobs report. A report is generated for each execution group. See
gems/scheduler/job_statistics.hpp for more details about what is tracked.

.. code-block:: none

   |=========================================================================================================================================================|
   |                                                             Job Statistics Report (regular)                                                             |
   |=========================================================================================================================================================|
   | Name                                               |   Job Mode |   Count | Time (Median - 90% - Max) [ms] | Rl Load | Overrun |   Overrun | Exec Delay |
   |---------------------------------------------------------------------------------------------------------------------------------------------------------|
   | ..tion/isaac.navigation.ParticleFilterLocalization |      Event |     357 |    24.11 |    34.13 |    42.55 |  35.7 % |   0.0 % |   0.00 ms |   74.4 mus |
   | ..p.local_map/isaac.navigation.BinaryToDistanceMap |      Event |     457 |     8.57 |    17.31 |    26.10 |  25.9 % |   0.0 % |   0.00 ms |  265.8 mus |
   | ..on.local_map.local_map/isaac.navigation.LocalMap |      Event |     458 |     4.87 |     9.70 |    20.15 |  16.4 % |   0.0 % |   0.00 ms |   90.7 mus |
   | ..global_plan/isaac.navigation.BinaryToDistanceMap |      Event |     358 |     3.54 |     6.69 |    11.20 |   6.6 % |   0.0 % |   0.00 ms |  249.1 mus |
   |   navigation.planner.global_plan_smoother/smoother |      Event |      18 |    49.55 |    56.54 |    63.87 |   4.0 % |   0.0 % |   0.00 ms |  278.3 mus |
   | ..rol.lqr/isaac.lqr.DifferentialBaseLqrPlanner     |   Periodic |     230 |     3.38 |     7.16 |    22.01 |   3.6 % |   0.0 % |   0.00 ms |  305.7 mus |
   | ..lation.sim_range_scan/isaac.flatsim.SimRangeScan |   Periodic |     460 |     0.71 |     0.96 |     1.30 |   1.3 % |   0.0 % |   0.00 ms |  -75.3 mus |
   | ..ometry.odometry/DifferentialBaseWheelImuOdometry |   Periodic |    2293 |     0.08 |     0.13 |     0.31 |   1.0 % |   0.0 % |   0.00 ms |  102.1 mus |
   | ..mulation/isaac.flatsim.DifferentialBaseSimulator |   Periodic |    2293 |     0.08 |     0.12 |     0.19 |   0.8 % |   0.0 % |   0.00 ms |   20.6 mus |
   | ...er.global_plan/isaac.path_planner.GlobalPlanner |   Periodic |      18 |     8.20 |    12.39 |    15.64 |   0.7 % |   0.0 % |   0.00 ms |  -25.7 mus |
   | ..simulation/isaac.flatsim.DifferentialBasePhysics |   Periodic |    2293 |     0.06 |     0.09 |     0.48 |   0.6 % |   0.0 % |   0.00 ms |   11.6 mus |
   | ...local_map/isaac.navigation.OccupancyToBinaryMap |      Event |     457 |     0.14 |     0.38 |    11.41 |   0.6 % |   0.0 % |   0.00 ms |  268.6 mus |
   | ..ol.control/isaac.planner.DifferentialBaseControl |   Periodic |    2294 |     0.07 |     0.10 |     0.17 |   0.6 % |   0.0 % |   0.00 ms |  174.0 mus |
   | ..lobal_plan/isaac.navigation.OccupancyToBinaryMap |      Event |     358 |     0.19 |     0.46 |    11.43 |   0.6 % |   0.0 % |   0.00 ms |  235.9 mus |
   |             navigation.local_map.local_map/cleanup |      Event |     458 |     0.11 |     0.19 |    11.80 |   0.5 % |   0.0 % |   0.00 ms |   71.6 mus |
   | ..zation.global_localization/grid_search_localizer |   Periodic |      22 |     0.27 |     0.41 |    86.98 |   0.4 % |   0.0 % |   0.00 ms |  -30.9 mus |
   |               _pose_tree_bridge/PoseTreeJsonBridge |   Periodic |     462 |     0.09 |     0.17 |     0.21 |   0.2 % |   0.0 % |   0.00 ms |  326.6 mus |
   | ..n.localization.scan_localization/flatscan_viewer |      Event |     357 |     0.09 |     0.20 |     0.30 |   0.2 % |   0.0 % |   0.00 ms |   52.9 mus |
   |                         _statistics/NodeStatistics |   Periodic |      93 |     0.32 |     0.57 |     7.94 |   0.2 % |   0.0 % |   0.00 ms |  -15.5 mus |
   |     navigation.planner.go_to/isaac.navigation.GoTo |   Periodic |     230 |     0.06 |     0.11 |     0.14 |   0.1 % |   0.0 % |   0.00 ms |   21.5 mus |
   | ..tion.localization.scan_localization/robot_viewer |   Periodic |     896 |     0.02 |     0.03 |     0.11 |   0.1 % |   0.0 % |   0.00 ms |  174.8 mus |
   | ..aluation/isaac.navigation.LocalizationEvaluation |   Periodic |     460 |     0.02 |     0.05 |     0.14 |   0.0 % |   0.0 % |   0.00 ms |  -84.6 mus |
   | ..ation.localize/isaac.navigation.LocalizeBehavior |   Periodic |     460 |     0.02 |     0.03 |     0.05 |   0.0 % |   0.0 % |   0.00 ms |  -68.7 mus |
   | ..eractive_markers_bridge/InteractiveMarkersBridge |   Periodic |     462 |     0.01 |     0.01 |     0.03 |   0.0 % |   0.0 % |   0.00 ms |  275.5 mus |
   | ...goal_behavior/isaac.navigation.SelectorBehavior |   Periodic |     460 |     0.01 |     0.01 |     0.03 |   0.0 % |   0.0 % |   0.00 ms |   -2.7 mus |
   | ..tion_mode/isaac.navigation.GroupSelectorBehavior |   Periodic |     460 |     0.00 |     0.01 |     0.01 |   0.0 % |   0.0 % |   0.00 ms |  -51.7 mus |
   |        simulation.sim_range_scan/lidar_initializer |   Periodic |     115 |     0.02 |     0.03 |     0.22 |   0.0 % |   0.0 % |   0.00 ms |  -87.4 mus |
   |                                  FailsafeHeartBeat |   Periodic |    2311 |     0.00 |     0.00 |     0.02 |   0.0 % |   0.0 % |   0.00 ms |  231.6 mus |
   |      goals.random_walk/isaac.navigation.RandomWalk |   Periodic |      92 |     0.01 |     0.02 |     0.10 |   0.0 % |   0.0 % |   0.00 ms |   -2.1 mus |
   | ..localization.robot_pose_initializer/initial_pose |   Periodic |     116 |     0.01 |     0.01 |     0.04 |   0.0 % |   0.0 % |   0.00 ms |  125.2 mus |
   |  navigation.planner.go_to/isaac.viewers.GoalViewer |      Event |       1 |     0.04 |     0.04 |     0.04 |   0.0 % |   0.0 % |   0.00 ms |   41.0 mus |
   |            _config_bridge/isaac.alice.ConfigBridge |      Event |       0 |     0.00 |     0.00 |     0.00 |   0.0 % |   0.0 % |   0.00 ms |    0.0 mus |
   | navigation.imu_odometry.imu_corrector/ImuCorrector |      Event |       0 |     0.00 |     0.00 |     0.00 |   0.0 % |   0.0 % |   0.00 ms |    0.0 mus |
   |=========================================================================================================================================================|

Scheduling Interface and Policy
--------------------------------

The scheduler follows the principles of Earliest Deadline First, but it does not require a known
execution time. The desired start time is used as a proxy. Jobs are executed in order of their
desired start times with priority and available slack as tie breakers, if needed.

In order to expose the ability to pin tasks to specific hardware cores, the scheduler uses the
concept of execution groups. Each group represents a set of cores on which associated jobs may be
executed. The scheduler operates in a greedy manner and allocates default groups to all cores not
explicitly allocated by a configuration file.

The Isaac SDK requires at least one core to operate certain internal behaviors. If all cores are
allocated, the scheduler still reserves one core for operations.

Execution groups are defined in gems/scheduler/execution_group_descriptor.hpp. They have three
primary values:

* std::string name: Specifies the name of the group for indexing.
* std::vector<int> cores: Specifies a list of cores on which the group operates.
* bool has_workers: A flag which controls whether the system spawns worker threads. If set to true
  then one worker thread per core in the group is spawned and pinned to those cores.

The primary entry point to the scheduler is the interface in gems/scheduler/scheduler.hpp.
The following 4 functions are the primary methods used to interact with the scheduler.

.. code-block:: c

   std::optional<JobHandle> createJob(const JobDescriptor& descriptor)
   void destroyJob(const JobHandle& handle);
   void startJob(const JobHandle& handle) const;
   void waitForJobDestruction(const JobHandle& handle) const;

Those functions create, destroy and start jobs. You must call waitForJobDestruction after
destroying a job to ensure that is safe to free resources as the job may be executing when
destroyJob is invoked.

The following convenience functions combine some of the common actions used when interacting with
the scheduler.

.. code-block:: c

   std::optional<JobHandle> createJobAndStart(const JobDescriptor& descriptor);
   void destroyJobAndWait(const JobHandle& handle);

The following functions handle event based tasks:

.. code-block:: c

   void registerEvents(const JobHandle& handle, const std::unordered_set<std::string>& events) const;
   void unregisterEvents(const JobHandle& handle, const std::unordered_set<std::string>& events) const;
   void notify(const std::string& event, int64_t target_time) const;

The following function returns statistics for a given job. See the `Job Statistics`_ for more
information.

.. code-block:: c

   JobStatistics getJobStatistics(const JobHandle& handle) const;

These functions control the time machine functionality of the scheduler. See `Time Machine and Clock
Scaling`_ for more information.

.. code-block:: c

   void enableTimeMachine();
   void disableTimeMachine();

Time Machine and Clock Scaling
--------------------------------

To accelerate simulation and training of algorithms, the scheduler is tied to the Isaac system
clock which supports clock scaling and to a system called the time machine.

Clock scaling allows you to speed up or slow down the system clock for the Isaac SDK, and the time
machine detects gaps in the schedule and removes them at run time. This can greatly reduce training
times during simulation with no discernable impact on training results.

The time machine does not work with blocking jobs because it is unable to detect when those jobs are
idle.

Scheduler Configuration
--------------------------------

The scheduler exposes a few basic parameters in the application configuration file. The following
sample JSON blocks demonstrates how to set these configurations.  If no default configurations are specified
the system will attempt to generate one manually.

.. code-block:: javascript

 "scheduler": {
    "use_time_machine": true,
    "clock_scale": 1.0,
    "execution_groups": [
      {
        "name": "MyTestWorkerGroup",
        "cores": [0,1,2,3],
        "workers": true
      },
      {
        "name": "MyTestBlockerGroup",
        "cores": [4,5,6,7],
        "workers": false
      }
    ]
  }

.. code-block:: javascript

 "scheduler": {
    "use_time_machine": true,
    "clock_scale": 1.0,
    "default_execution_group_config": [
      {
        "worker_cores": [0,1],
        "blocker_cores": [4,5]
      }
    ]
  }
