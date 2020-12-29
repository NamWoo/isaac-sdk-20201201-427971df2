'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from datetime import datetime
import enum
import json
import logging
from threading import Lock
from typing import List, Dict
import uuid

from isaac import Message as im
from .JsonTcpServer import JsonTcpServer, JsonTcpServerConnection

MASK_64BIT = ((1 << 64) - 1)


class Mission:
    status_lock = Lock()

    class Status(enum.Enum):
        # The mission has not yet been submitted to the mission server
        CREATED = 0

        # The mission has been submitted but has not started because either:
        #  - The robot it should run on is not connected
        #  - There is another mission before it in the robot's mission queue
        #  - At least one of this mission's "upstream" missions hasn't completed yet
        QUEUED = 1

        # The mission has been sent to the robot but the robot has not yet acknowledged it
        STARTED = 2

        # The mission has been sent to and acknowledged by the robot
        RUNNING = 3

        # The robot indicated that the mission has succeeded
        SUCCESS = 4

        # The robot indicated that the mission failed
        FAILED = 5

        # The robot lost connection in the middle of the mission
        FAILED_DISCONNECTED = 6

        # One of the following things happened:
        # - The mission was sent to the robot but the robot did not acknowledge it before the
        #   start_timeout
        # - The robot acknowledged the mission but did not indicate success or failure before
        #   timeout
        FAILED_TIMEDOUT = 7

        # The mission will not be scheduled because one of its upstream missions failed
        FAILED_UPSTREAM = 8

    def __init__(self, robot: str, channel: str, config: Dict, status_channel: str = None,
                 **kwargs):
        '''
        Constructs a Mission object

        Args:
            robot (str): The name of the robot that should run this mission
            channel (str): The JsonTcpServer channel to transmit the MissionProto on
            config (dict): The ISAAC application configuration to embed in the MissionProto
            status_channel (str)(optional): The JsonTcpServer channel to listen for the
                                       MissionStatusProto on.
            start_timeout (int)(optional): The number of seconds to wait between sending the
                                           MissionProto and receiving an acknowledgment
                                           MissionStatusProto before giving up
            timeout (int)(optional): The number of seconds to allow a mission to run before giving
                                     up
        '''
        self._robot = robot
        self._channel = channel
        self._config = config
        self._status_channel = status_channel if status_channel is not None else channel + "_status"
        self._status = Mission.Status.CREATED
        self._uuid = uuid.uuid4()
        self._start_timeout = kwargs.get("start_timeout", 5)
        self._timeout = kwargs.get("timeout", float("inf"))

        # "Downstream missions" that can't run until this one has been run
        self._downstream = []
        # The number of upstream missions that need to finish before this mission can start
        self._outstanding_upstream = 0

        # Count all of the upstream missions that are blocking this one from starting.
        # Add this mission to the downstream list of each "outstanding" upstream mission
        self._upstream = set(kwargs.get("upstream", []))
        with self.status_lock:
            for upstream_mission in self._upstream:
                # If the upstream mission is CREATED/RUNNING/QUEUED, it is "outstanding"
                # and therefore blocking this mission from starting
                if upstream_mission.status in (Mission.Status.CREATED, Mission.Status.RUNNING,
                                               Mission.Status.QUEUED):
                    upstream_mission._downstream.append(self)
                    self._outstanding_upstream += 1

                # If the upstream mission has already completed, it is not outstanding
                elif upstream_mission.status == Mission.Status.SUCCESS:
                    pass

                # If the upstream mission has failed, then this mission cannot run
                else:
                    self._outstanding_upstream = -1
                    break

    @property
    def uuid(self):
        return self._uuid

    @property
    def uuid_dict(self):
        return {
            "lower": (self.uuid.int >> 0) & MASK_64BIT,
            "upper": (self.uuid.int >> 64) & MASK_64BIT,
        }

    @property
    def status(self) -> "Mission.Status":
        return self._status

    @status.setter
    def status(self, new_status: "Mission.Status"):
        with self.status_lock:
            # Skip if reassigning the same status so start/end times dont get changed
            if new_status == self._status:
                return
            # If this mission started, mark the start time
            if new_status == Mission.Status.STARTED:
                self._sent_time = datetime.now()
            # If this mission is running, mark the time it started running
            elif new_status == Mission.Status.RUNNING:
                self._start_time = datetime.now()
            # If this mission succeeded, mark the end time and decrement the "outstanding_upstream"
            # counter for all downstream missions
            elif new_status == Mission.Status.SUCCESS:
                self._end_time = datetime.now()
                for downstream in self._downstream:
                    downstream._outstanding_upstream -= 1
            # If this mission failed, mark the end time and set the "outstanding_upstream" counter
            # to -1 for all downstream missions indicating an upstream mission failed
            elif new_status in (Mission.Status.FAILED, Mission.Status.FAILED_DISCONNECTED,
                                Mission.Status.FAILED_TIMEDOUT, Mission.Status.FAILED_UPSTREAM):
                self._end_time = datetime.now()
                for downstream in self._downstream:
                    downstream._outstanding_upstream = -1
            self._status = new_status

    def check_timeout(self) -> bool:
        '''
        Checks the state of the mission and updates the status to FAILED_TIMEOUT if necessary

        Returns:
            True if the mission status has been updated to FAILED_TIMEDOUT, otherwise false
        '''
        timed_out = False
        with self.status_lock:
            # If the mission was sent, make sure an acknowledgement was sent in a timely manner
            if self._status == Mission.Status.STARTED and (
                    datetime.now() - self._sent_time).seconds > self._start_timeout:
                timed_out = True
            # If the mission was sent and acknowledge, make sure it completes within the timeout
            elif self._status == Mission.Status.RUNNING and (
                    datetime.now() - self._start_time).seconds > self._timeout:
                timed_out = True

        if timed_out:
            self.status = Mission.Status.FAILED_TIMEDOUT

        return timed_out

    @property
    def config(self) -> Dict:
        return self._config

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def outstanding_upstream(self):
        return self._outstanding_upstream

    @property
    def all_upstream_submitted(self):
        ''' Checks if all upstream missions have already been submitted '''
        with self.status_lock:
            not_submitted = [
                mission for mission in self._upstream if mission._status == Mission.Status.CREATED
            ]
            return len(not_submitted) == 0


class _RobotConnection:
    ''' A class to help MissionServer keep track of all of the active robot TCP connections '''

    def __init__(self, connection: JsonTcpServerConnection, name_channel: str = "name"):
        '''
        Constructs a RobotConnection object

        Args:
            connection (JsonTcpServerConnection): The connection used to communicate with the robot
            name_channel (str): The JsonTcpServer channel to listen for TextProtos to get the robot
                                name
        '''
        self._connection = connection
        self._name = None
        self._current_mission = None
        self._connection.set_message_callback(self._message_callback)
        self._name_channel = name_channel
        self._logger = logging.getLogger('MissionServer')

    @property
    def name(self) -> str:
        return self._name

    def _message_callback(self, message, channel):
        # Update the robot name if applicable
        proto = message.proto
        if channel == self._name_channel and self._name is None:
            self._name = proto.text
            self._logger.info("Robot at address {} identified as \"{}\"".format(
                self._connection.address, self.name))
            return

        # If there is a mission running, see if there is a status update
        if self._current_mission is not None and channel == self._current_mission._status_channel:
            status_uuid = ((proto.uuid.upper & MASK_64BIT) << 64) + (proto.uuid.lower & MASK_64BIT)
            # Does this status message have to do with the current mission?
            if status_uuid == self._current_mission.uuid.int:
                if proto.missionStatus == "running":
                    self._current_mission.status = Mission.Status.RUNNING
                elif proto.missionStatus == "success":
                    self._current_mission.status = Mission.Status.SUCCESS
                    self._current_mission = None
                elif proto.missionStatus == "failure":
                    self._current_mission.status = Mission.Status.FAILED
                    self._current_mission = None

    def start_mission(self, mission: Mission):
        # Generate the message
        builder = im.MessageBuilder()
        builder.proto = im.CAPNP_DICT["MissionProto"].from_dict({
            "uuid": mission.uuid_dict,
            "config": {
                "serialized": json.dumps(mission.config)
            }
        })

        # Send the mission
        self._connection.send_message(builder, mission.channel)

        # Set the mission state to STARTED
        mission.status = Mission.Status.STARTED

        self._current_mission = mission

    @property
    def connected(self) -> bool:
        return self._connection.connected

    def close(self):
        self._logger.info("Robot \"{}\" lost connection".format(self.name))
        if self._current_mission is not None:
            self._current_mission.status = Mission.Status.FAILED_DISCONNECTED

    def check_timeout(self):
        if self._current_mission is None:
            return

        if self._current_mission.check_timeout():
            self._current_mission = None


class MissionServer:
    '''
    A class to manage the submission and running of missions to robots. It instantiates a
    JsonTcpServer and uses it to listen for connections to robots. Mission submitted to this class
    are queued, scheduled, and sent to connected robots.
    '''

    def __init__(self, port: int = 9998, name_channel: str = "name"):
        '''
        Constructs a MissionServer object

        Args:
            port (int): The TCP port to listen on for robot connections
            name_channel (str): The JsonTcpServer channel to listen for TextProtos to get the robot
                                name
        '''

        self._name_channel = name_channel

        # Create dictionary for robots by name and a list of unidentified robots
        self._robots = {}
        self._unidentified_robots = []

        # Create a dictionary of mission queues by robot name
        self._mission_queues = {}
        self._mission_queue_lock = Lock()

        # Initialize logger
        FORMAT = '%(asctime)-15s %(levelname)s %(message)s'
        logging.basicConfig(format=FORMAT)
        self._logger = logging.getLogger('MissionServer')
        self._logger.setLevel(logging.DEBUG)

        # Create a TCP server for robots to connect to and recieve missions
        self._server = JsonTcpServer(port, self._new_connection_handler, self._update_callback)

    def get_robots(self) -> List[str]:
        return list(self._robots.keys())

    def submit(self, mission: Mission):
        ''' Queue a mission to be run '''

        # Verify the mission has not been submitted before
        if mission._status != Mission.Status.CREATED:
            raise ValueError("Expected mission in state {} but is in state {}".format(
                Mission.Status.CREATED, mission._status))

        # Verify that all upstream missions have been submitted
        if not mission.all_upstream_submitted:
            raise ValueError("At least one upstream mission hasn't been submitted yet.")
        # Clear the upstream list so we dont have cyclical references
        mission._upstream = []

        # Add the mission to the appropriate QUEUE
        with self._mission_queue_lock:
            robot_name = mission._robot
            if robot_name not in self._mission_queues.keys():
                self._mission_queues[robot_name] = []
            self._mission_queues[robot_name].append(mission)

        # Mark the mission as queued
        mission.status = Mission.Status.QUEUED

    def send_message(self, robot_name: str, message, channel: str):
        '''
        Sends a message to the given channel on the specified robot

        Args:
            robot_name (str): The name of the robot to send the message to
            message (MessageBuilder): A capnp message to send to the robot
            channel (str): The channel on which to send the message

        Raises:
            IOError: The specified robot never connected or has lost connection
        '''
        robot = self._robots.get(robot_name, None)
        if robot is None:
            raise IOError("Robot \"{}\" is not connected".format(robot_name))
        robot._connection.send_message(message, channel)

    def get_next_message(self, robot_name: str, channel: str):
        '''
        Gets the next message from the given channel on the given robot and pops it off of the
        channel's queue.

        Args:
            robot_name (str): The name of the robot to fetch the message from
            channel (str): The channel to read the message from

        Returns:
            A MessageBuilder containing the oldest unread message or 'None' if all messages have
            been read

        Raises:
            IOError: The specified robot never connected or has lost connection
        '''
        robot = self._robots.get(robot_name, None)
        if robot is None:
            raise IOError("Robot \"{}\" is not connected".format(robot_name))
        return robot._connection.get_next_message(channel)

    def _update_callback(self):
        # Move all robots with names in the unidentified_robots list to the robots dict
        self._robots.update(
            {robot.name: robot
             for robot in self._unidentified_robots if robot.name is not None})
        self._unidentified_robots = [
            robot for robot in self._unidentified_robots if robot.name is None
        ]

        # Check if any robots have disconnected
        for key in list(self._robots.keys()):
            if not self._robots[key].connected:
                self._robots[key].close()
                del self._robots[key]

        # Check if any robots are ready for new mission
        with self._mission_queue_lock:
            for name, robot in self._robots.items():
                mission_queue = self._mission_queues.get(name, [])
                while robot._current_mission is None and len(mission_queue) > 0:
                    # Get the first mission off the queue
                    mission = mission_queue[0]

                    # This mission still has oustanding upstream missions, nothing can be scheduled
                    if mission.outstanding_upstream > 0:
                        break

                    # If this mission doesn't have any outstanding dependencies, it can be scheduled
                    if mission.outstanding_upstream == 0:
                        robot.start_mission(mission)
                        mission_queue.pop(0)
                        break

                    # If this mission has a failed upstream mission, mark it as failed and throw it
                    # out
                    mission.status = Mission.Status.FAILED_UPSTREAM
                    mission_queue.pop(0)

        # Check for timed out missions
        for name, robot in self._robots.items():
            robot.check_timeout()

    def _new_connection_handler(self, connection: JsonTcpServerConnection):
        self._unidentified_robots.append(_RobotConnection(connection, self._name_channel))
