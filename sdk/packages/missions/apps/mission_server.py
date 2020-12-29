'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import argparse
import json
import jsonschema
import time

from packages.missions import Mission, MissionServer

# The range of valid TCP port values (uint16)
MIN_PORT_NUM = 0
MAX_PORT_NUM = (2**16) - 1

# Parameters copied directly from mission json into the Mission class's constructor
DIRECT_MISSION_PARAMS = ("robot", "channel", "status_channel", "config", "timeout", "start_timeout")
# Status codes that indicate a mission hasn't yet completed
INCOMPLETE_STATUSES = (Mission.Status.CREATED, Mission.Status.QUEUED, Mission.Status.STARTED,
                       Mission.Status.RUNNING)


# Verify the "port" parameter provided on command line is a valid TCP port
def check_port(port_string):
    port = int(port_string)
    if port < MIN_PORT_NUM or port > MAX_PORT_NUM:
        raise argparse.ArgumentTypeError("Port must be in range [{}, {}]".format(
            MIN_PORT_NUM, MAX_PORT_NUM))
    return port


if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser(description="Navsim navigation app")
    parser.add_argument(
        "--port",
        help="The TCP port to listen for robot connections on",
        type=check_port,
        default=9998)
    parser.add_argument(
        "--name_channel", help="The channel on which robots broadcast their name", default="name")
    parser.add_argument(
        "mission_file",
        type=argparse.FileType(),
        help="A json file that contains missions to submit")
    args = parser.parse_args()

    # Parse json file and create mission objects
    json_data = json.load(args.mission_file)
    missions = []
    for num, mission in enumerate(json_data["missions"]):
        mission_params = {key: mission[key] for key in mission if key in DIRECT_MISSION_PARAMS}
        upstream_missions = []
        if "upstream" in mission:
            for upstream_num in mission["upstream"]:
                if num <= upstream_num:
                    raise ValueError(
                        "Mission number {} is upstream from mission {}. All upstream missions must \
                         be listed first!".format(upstream_num, num))
                upstream_missions.append(missions[upstream_num])
        mission_params["upstream"] = upstream_missions
        missions.append(Mission(**mission_params))

    # Create mission server and submit missions
    server = MissionServer(args.port, args.name_channel)
    for mission in missions:
        server.submit(mission)

    # Wait for execution of missions
    incomplete_missions = set(missions)
    while len(incomplete_missions) > 0:
        time.sleep(2)
        print("\n\n================================")
        print("Connected Robots: ")
        robots = list(server.get_robots())
        if len(robots) == 0:
            print(" (None)")
        else:
            for robot in server.get_robots():
                print(" - \"{}\"".format(robot))
        print("")
        print("Mission | Robot        | Status ")
        print("--------+--------------+--------")
        for num, mission in enumerate(missions):
            print(" {:>6} | {:<12} | {}".format(num, mission._robot, mission.status.name))

        # Check for missions that have been completed
        completed_missions = []
        for mission in incomplete_missions:
            if mission.status not in INCOMPLETE_STATUSES:
                completed_missions.append(mission)
        for mission in completed_missions:
            incomplete_missions.remove(mission)
