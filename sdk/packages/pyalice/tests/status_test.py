'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import unittest
import tempfile
import time
import uuid
import numpy as np
from isaac import Application, Codelet, Node, Status


class AlwaysSucceedCodelet(Codelet):
    '''
    Test codelet that reports success after 2 ticks
    '''
    def start(self):
        self.tick_periodically(0.1)
        self._cnt = 0

    def stop(self):
        assert self._cnt == 3

    def tick(self):
        self._cnt += 1
        if self._cnt > 2:
            self.report_success('Success after {} ticks'.format(self._cnt))


class AlwaysFailureCodelet(Codelet):
    '''
    Test codelet that reports failure after 2 ticks
    '''
    def start(self):
        self.tick_periodically(0.1)
        self._cnt = 0

    def stop(self):
        assert self._cnt == 3

    def tick(self):
        self._cnt += 1
        if self._cnt > 2:
            self.report_failure('Failure after {} ticks'.format(self._cnt))


class TestPyCodeletStatus(unittest.TestCase):
    '''
    Test loading subgraph via the application API
    '''
    @classmethod
    def setUpClass(cls):
        # method will be ran once before any test is ran
        pass

    @classmethod
    def tearDownClass(cls):
        # method will be ran once after all tests have run
        pass

    def setUp(self):
        # ran before each test
        return super().setUp()

    def tearDown(self):
        # ran after each test
        return super().tearDown()

    def test_report_status(self):
        self._app = Application()
        self._app.add('success')
        self._app.add('failure')

        node_success = self._app.nodes['success']
        self.assertIsNotNone(node_success)
        component = node_success.add(self._app.registry.isaac.alice.PyCodelet)
        self.assertIsNotNone(component)

        node_failure = self._app.nodes['failure']
        self.assertIsNotNone(node_failure)
        component = node_failure.add(self._app.registry.isaac.alice.PyCodelet)
        self.assertIsNotNone(component)

        self._app.nodes['success'].add(AlwaysSucceedCodelet)
        self._app.nodes['failure'].add(AlwaysFailureCodelet)

        self._app.start_wait_stop(1.5)

        self.assertEqual(node_success.status, Status.Success)
        self.assertEqual(node_failure.status, Status.Failure)


if __name__ == '__main__':
    unittest.main()
