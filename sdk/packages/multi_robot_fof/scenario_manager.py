'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from threading import Lock
from typing import List, Dict

from isaac import Application, Codelet
from packages.missions import Mission


class Scenario(object):
    '''This class tracks available actors and missions. It allows actor managers and mission
    generator(s) Codelets to exchange actor availability/missions'''

    status_lock = Lock()
    mission_lock = Lock()

    def __init__(self, types: List[str]):
        '''Initialize availability and missions for each class of actors

        Inputs:
          types: names to identify each class (group) of actors (robots or assets)
        '''

        self.available = {}
        self.mission = {}
        for type in types:
            self.available[type] = []
            self.mission[type] = {}

    def provide(self, type: str, name: str):
        '''Update an actor to available

        Inputs:
          type: name of the actor's class
          name: name of the actor
        '''

        with self.status_lock:
            if name not in self.available[type]:
                self.available[type].append(name)

    def acquire(self, types: List[str]):
        '''Attempts to acquire one available actor from each of the classes in types

        Inputs:
          types: a list of actor classes
        Outputs:
          Dictionary of {type:name} with one available actor for each actor class in types
        '''

        with self.status_lock:
            for type in types:
                if not self.available[type]:
                    return
            resource = {}
            for type in types:
                name = self.available[type][0]
                self.available[type].remove(name)
                resource[type] = name
            return resource

    def pop_missions(self, type: str, name: str = None) -> List[Mission]:
        '''Pop all the missions for an actor. If actor name is None, pops all the missions of the
        actor class. This clears the mission queue for the particular actor (class)

        Inputs:
          type: name of the actor's class
          name: name of the actor
        Outputs:
          List[Mission]
        '''

        missions = []
        with self.mission_lock:
            if name is None:
                for name in self.mission[type]:
                    missions += self.mission[type][name]
                    self.mission[type][name] = []
            else:
                if name in self.mission[type]:
                    missions = self.mission[type][name]
                    self.mission[type][name] = []
        return missions

    def add_missions(self, type: str, name: str, missions: List[Mission]):
        '''Appends missions for an actor to the mission queue

        Inputs:
          type: name of the actor's class
          name: name of the actor
          missions: a list of missions to append to the mission queue
        '''

        with self.mission_lock:
            if name in self.mission[type]:
                self.mission[type][name] += missions
            else:
                self.mission[type][name] = missions


class VanillaManager(Codelet):
    '''Handles missions by marking any ready-to-run missions as done

    Required config:
      num_assets (int): number of instances of assets in this group. the name of individual
          assets should be identifiable by index
      type (str): type name of the asset group, used to identify mission/availability in Scenario
    Optional config:
      name (str): name of the assets with placeholder for instance index, for example "cube_{0}". If
          not provided, default to type as prefix
    '''

    def start(self):
        self.missions = []
        if self.config.name is None:
            self.config.name = self.config.type + "_{0}"
        for i in range(self.config.num_assets):
            self.scenario.provide(self.config.type, self.config.name.format(i))
        self.tick_periodically(1)

    def tick(self):
        # Claim missions from exchange
        new_missions = self.scenario.pop_missions(self.config.type)
        for mission in new_missions:
            mission.status = Mission.Status.QUEUED
        self.missions += new_missions

        for mission in self.missions:
            if mission.outstanding_upstream <= 0:
                self.missions.remove(mission)
                mission.status = Mission.Status.SUCCESS
                self.scenario.provide(self.config.type, mission.config['name'])


class TeleportManager(Codelet):
    '''Handles missions to reset asset poses by publishing Rigidbody3Group message to teleport
    the assets in simulation.

    Required config:
      num_assets (int): number of instances of assets in this group. the name of individual
          assets should be identifiable by index
      type (str): type name of the asset group, used to identify mission/availability in Scenario
    Optional config:
      name (str): name of the assets with placeholder for instance index, for example "cube_{0}". If
          not provided, default to type as prefix
    '''

    def start(self):
        self.ongoing_missions = {}
        # Initialize channels for sending teleport message
        self.tx_teleport = self.isaac_proto_tx("RigidBody3GroupProto", "teleport")

        # Mark all assets as available
        if self.config.name is None:
            self.config.name = self.config.type + "_{0}"
        for i in range(self.config.num_assets):
            name = self.config.name.format(i)
            self.scenario.provide(self.config.type, name)
            self.ongoing_missions[name] = []
        self.tick_periodically(1)

    def tick(self):
        # Get new missions
        new_missions = self.scenario.pop_missions(self.config.type)
        for mission in new_missions:
            mission.status = Mission.Status.QUEUED
            self.ongoing_missions[mission.config['name']].append(mission)

        # Get missions that are ready to run (has no outstanding upstream missions)
        ready_missions = {}
        keys = list(self.ongoing_missions.keys())
        for name in keys:
            for mission in self.ongoing_missions[name]:
                # Will teleport to reset even if upstream fails (outstanding_upstream = -1)
                if mission.outstanding_upstream <= 0:
                    ready_missions[name] = mission
                    self.ongoing_missions[name].remove(mission)
                    mission.status = Mission.Status.SUCCESS

        # Teleport all the assets in the ready-to-run missions in one message
        count = len(ready_missions)
        if count > 0:
            msg = self.tx_teleport.init()
            bodies = msg.proto.init('bodies', count)
            names = msg.proto.init('names', count)
            keys = list(ready_missions.keys())
            for i in range(count):
                mission = ready_missions[keys[i]]
                names[i] = mission.config['name']
                pose = mission.config['pose']
                bodies[i].refTBody.translation.x = pose[4]
                bodies[i].refTBody.translation.y = pose[5]
                bodies[i].refTBody.translation.z = pose[6]
                bodies[i].refTBody.rotation.q.w = pose[0]
                bodies[i].refTBody.rotation.q.x = pose[1]
                bodies[i].refTBody.rotation.q.y = pose[2]
                bodies[i].refTBody.rotation.q.z = pose[3]
                self.log_info("Execute ({0}/{1}) {2} for {3}".format(i, count, mission.uuid,
                                                                     names[i]))
            self.tx_teleport.publish()

        # Mark actors with no ongoing mission as free
        for name in ready_missions:
            if not self.ongoing_missions[name]:
                self.scenario.provide(self.config.type, name)


class RobotManager(Codelet):
    '''Handles missions to robots by submitting to mission server

    Required config:
      robot_name (str): base name of the robots in this group, individual robot name published by
        the robots should be robot_name_[robot_index]
      idle_threshold (float): number of ticks a robot must be idle before making it available for
        next mission. This simulates charging/downtime between missions
    '''

    def start(self):
        self.ongoing_missions = {}
        self.idle_time = {}
        self.tick_periodically(1)
        self.success_count = {}
        self.fail_count = {}

    def tick(self):
        # Add new robots
        for robot in self.mission_server.get_robots():
            if robot.startswith(self.config.robot_name) and robot not in self.ongoing_missions:
                self.log_info("Add robot {0}".format(robot))
                self.ongoing_missions[robot] = []
                self.idle_time[robot] = self.config.idle_threshold + 1
                self.success_count[robot] = 0
                self.fail_count[robot] = 0

        # Remove finished missions and increment idle time
        for robot in self.ongoing_missions:
            if not self.ongoing_missions[robot]:
                self.idle_time[robot] += 1
            else:
                for mission in self.ongoing_missions[robot]:
                    # Submit all created missions whose upstreams have been submitted
                    if mission.status == Mission.Status.CREATED and mission.all_upstream_submitted:
                        self.mission_server.submit(mission)
                    # Remove successful mission
                    elif mission.status == Mission.Status.SUCCESS:
                        self.log_info("{0} succeeds with mission {1}".format(robot, mission.uuid))
                        self.ongoing_missions[robot].remove(mission)
                        self.success_count[robot] += 1
                    # Remove failed mission
                    elif mission.status in (Mission.Status.FAILED,
                                            Mission.Status.FAILED_DISCONNECTED,
                                            Mission.Status.FAILED_TIMEDOUT,
                                            Mission.Status.FAILED_UPSTREAM):
                        self.log_error("{0} fails with mission {1}".format(robot, mission.uuid))
                        self.ongoing_missions[robot].remove(mission)
                        self.fail_count[robot] += 1

        # Update availability to scenario manager, and get new missions from mission generator
        for robot in self.ongoing_missions:
            for mission in self.scenario.pop_missions(self.config.robot_name, robot):
                self.ongoing_missions[robot].append(mission)
                self.idle_time[robot] = 0
            if self.idle_time[robot] > self.config.idle_threshold:
                self.scenario.provide(self.config.robot_name, robot)
            self.show("success.{}".format(robot), self.success_count[robot])
            self.show("fail.{}".format(robot), self.fail_count[robot])


def transporter_mission_config(waypoint: str, tolerance: List[float]) -> Dict:
    '''Helper function to generate mission config for transporter app given waypoint and arrival
    tolerance

    Input:
      waypoint: name of the goal waypoint on the map waypoint layer
      tolerance: arrival tolerance as [position in meter, rotation angle in radian]
    '''

    config = {
        "goals.waypoint_as_goal": {
            "isaac.navigation.MapWaypointAsGoal": {
                "waypoint": waypoint
            }
        },
        "navigation.go_to.goal_monitor": {
            "GoalMonitor": {
                "arrived_position_thresholds": tolerance
            }
        }
    }
    return config


def station_mission_config(name: str, pose: List[float]) -> Dict:
    '''Helper function to generate mission config for station pick and place task

    Input:
      name: name of the object to pick
      place: 3D pose as [q.w, q.x, q.y, q.z, t.x, t.y, t.z]
    '''

    config = {"mission_feeder": {"PyCodelet": {"tasks": [{"pick": name, "place": pose}]}}}
    return config
