'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from isaac import *
from packages.missions import Mission, MissionServer

import time
import unittest

# The statuses that indicate that a mission has not yet completed
PENDING_MISSION_STATUSES = (Mission.Status.CREATED, Mission.Status.QUEUED, Mission.Status.STARTED,
                            Mission.Status.RUNNING)


class SimpleBehavior(Codelet):
    '''
    A simple mock behavior tree that will immediately return success for failure depending on the
    value of the boolean "success" config parameter
    '''

    def start(self):
        self.tick_periodically(0.1)

    def tick(self):
        if self.config["success"]:
            self.report_success("success")
            return
        else:
            self.report_failure("failure")
            return


class TestMissions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Creates an application with the mission subgraph and a simple pass/fail behavior tree
        cls.app = Application()
        cls.app.load("packages/behavior_tree/apps/missions.graph.json")
        cls.app.add("behavior", [SimpleBehavior])
        cls.app.nodes["mission_control"].components["NodeGroup"].config["node_names"] = ["behavior"]
        cls.app.nodes["robot_name"].components["JsonMockup"].config["json_mock"] = \
            {"text":"mock-carter"}

        # Start the application and wait for it to be ready
        cls.app.start()
        cls.app.wait_for_node("behavior")

        # Mission configs for a failing and passing mission
        cls.success_config = {"behavior": {"PyCodelet": {"success": True}}}
        cls.failure_config = {"behavior": {"PyCodelet": {"success": False}}}

        # Create a mission server
        cls.server = MissionServer()

    @classmethod
    def tearDownClass(cls):
        cls.app.stop()

    def _wait_with_timeout(self, mission: Mission, expected: Mission.Status,
                           timeout: float = 5) -> None:
        '''
        Waits for a mission to complete with a given timeout.
         - Asserts that the mission completes within the timeout
         - Asserts that the mission completes with the expected status
        '''
        start_time = time.time()
        while mission.status in PENDING_MISSION_STATUSES:
            self.assertTrue(
                time.time() < start_time + timeout,
                "Mission did not complete within timeout! Mission had status {} when timeout was \
                reached".format(mission.status))
            time.sleep(0.1)

        self.assertEqual(mission.status, expected,
                         "Mission did not complete with the expected status!")

    def test_successful_mission(self):
        ''' Tests one mission that should succeed '''
        mission = Mission("mock-carter", "mission", self.success_config)
        self.server.submit(mission)
        self._wait_with_timeout(mission, Mission.Status.SUCCESS)

    def test_failed_mission(self):
        ''' Tests one mission that should fail '''
        mission = Mission("mock-carter", "mission", self.failure_config)
        self.server.submit(mission)
        self._wait_with_timeout(mission, Mission.Status.FAILED)

    def test_dependent_successful_missions(self):
        '''
        Tests two mission (1 and 2). 2 depends on 1 and should run afterwards. Both should succeed.
        '''
        mission1 = Mission("mock-carter", "mission", self.success_config)
        mission2 = Mission("mock-carter", "mission", self.success_config, upstream=[mission1])
        self.server.submit(mission1)
        self.server.submit(mission2)
        self._wait_with_timeout(mission1, Mission.Status.SUCCESS)
        self._wait_with_timeout(mission2, Mission.Status.SUCCESS)

    def test_dependent_failed_missions(self):
        '''
        Tests two mission (1 and 2). 2 depends on 1 and should run afterwards. Mission 1 should
        fail, and mission 2 should automatically fail because it depends on mission 1.
        '''
        mission1 = Mission("mock-carter", "mission", self.failure_config)
        mission2 = Mission("mock-carter", "mission", self.success_config, upstream=[mission1])
        self.server.submit(mission1)
        self.server.submit(mission2)
        self._wait_with_timeout(mission1, Mission.Status.FAILED)
        self._wait_with_timeout(mission2, Mission.Status.FAILED_UPSTREAM)


if __name__ == "__main__":
    unittest.main()
