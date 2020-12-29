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
from isaac import Application, Cask, Message, Node, Component


class TestComponent(unittest.TestCase):
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

    def test_component_config(self):
        self._app = Application()
        self._app.add('node')

        node = self._app.nodes['node']
        self.assertIsNotNone(node)

        result = self._app.load_module('viewers')
        self.assertTrue(result)

        component = node.add(self._app.registry.isaac.viewers.ImageViewer)
        self.assertTrue(isinstance(component, Component.Component))
        self.assertIsNotNone(component)

        self.assertIsNotNone(component.config)
        component.config.reduce_scale = 2.0
        self.assertEqual(component.config.reduce_scale, 2.0)

        self.assertEqual(component.config['reduce_scale'], 2.0)
        component.config['reduce_scale'] = 3.0
        self.assertEqual(component.config['reduce_scale'], 3.0)
        self.assertEqual(component.config.reduce_scale, 3.0)

        self._app.start()
        self._app.stop()

    def test_component_access(self):
        self._app = Application()
        self._app.add('node')

        node = self._app.nodes['node']
        self.assertIsNotNone(node)

        result = self._app.load_module('viewers')
        self.assertTrue(result)
        node.add(self._app.registry.isaac.viewers.ImageViewer)

        component = node.components['ImageViewer']
        self.assertTrue(isinstance(component, Component.Component))
        self.assertIsNotNone(component)

        self.assertEqual(node.components['ImageViewer'].config['target_fps'], 30.0)

        component = node.components['ImageViewer']
        self.assertTrue(isinstance(component, Component.Component))
        self.assertIsNotNone(component)

        self.assertEqual(node.components['ImageViewer'].config['target_fps'], 30.0)

        node.components['ImageViewer'].config['target_fps'] = 45.0
        self.assertEqual(node.components['ImageViewer'].config['target_fps'], 45.0)


if __name__ == '__main__':
    unittest.main()
