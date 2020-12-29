'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import os
import unittest
import tempfile
import time
import types
import uuid

import numpy as np
import quaternion

from isaac import Application, Cask, Message, Node, Status, Codelet


class SucceedLaterCodelet(Codelet):
    '''
    Test codelet that reports success after 2 ticks
    '''
    def start(self):
        self.tick_periodically(0.1)
        self._cnt = 0

    def stop(self):
        assert self._cnt == 5

    def tick(self):
        self._cnt += 1
        if self._cnt > 4:
            self.report_success('Success after {} ticks'.format(self._cnt))


class FailureLaterCodelet(Codelet):
    '''
    Test codelet that reports failure after 2 ticks
    '''
    def start(self):
        self.tick_periodically(0.1)
        self._cnt = 0

    def stop(self):
        assert self._cnt == 5

    def tick(self):
        self._cnt += 1
        if self._cnt > 4:
            self.report_failure('Failure after {} ticks'.format(self._cnt))


class TestApplication(unittest.TestCase):
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

    def test_get_component(self):
        self._app = Application()
        self._app.add('node')

        node = self._app.nodes['node']
        self.assertIsNotNone(node)

        # Get None if no component has such name
        component = node['ledger']
        self.assertIsNone(component)

        component = node['MessageLedger']
        self.assertIsNotNone(component)

        self._app.start()
        self._app.stop()

    def test_load_subgraph(self):
        self._app = Application()
        self._app.add('node')

        # loads subgraph and checks if the node/component are created, config is good
        self._app.load('packages/pyalice/tests/application_test.subgraph.json', 'foo')

        node = self._app.nodes['foo.camera_viewer']
        component = node['ImageViewer']
        self.assertIsNotNone(component)

        fps_readback = self._app.nodes['foo.camera_viewer'].components['ImageViewer'].config[
            'target_fps']
        self.assertEqual(fps_readback, 11)

        camera_name_readback = self._app.nodes['foo.camera_viewer'].components[
            'ImageViewer'].config['camera_name']
        self.assertEqual(camera_name_readback, 'bar')

        # loads the subgraph again with different prefix name
        self._app.load('packages/pyalice/tests/application_test.subgraph.json', 'foo1')
        node = self._app.nodes['foo1.camera_viewer']
        component = node['ImageViewer']
        self.assertIsNotNone(component)

        fps_readback = self._app.nodes['foo1.camera_viewer'].components['ImageViewer'].config[
            'target_fps']
        self.assertEqual(fps_readback, 11)

        camera_name_readback = self._app.nodes['foo1.camera_viewer'].components[
            'ImageViewer'].config['camera_name']

        self._app.start()
        self._app.stop()

    def test_load_bogus_subgraph(self):
        bogus_json_filename = ''
        with tempfile.NamedTemporaryFile('w') as f:
            f.write('this is bogus json')
            bogus_json_filename = f.name

        self._app = Application()
        with self.assertRaises(ValueError):
            self._app.load('/no/such/file')

        with self.assertRaises(ValueError):
            self._app.load(bogus_json_filename)

    def test_load_subgraph_in_asset_path(self):
        self._app = Application()
        self._app.add('node')

        # override the asset path to point to our test location
        self._app._asset_path = 'packages/pyalice/tests/mock_asset_path'

        # loads subgraph and checks if the node/component are created, config is good
        self._app.load('mock_installed_application_test.subgraph.json', 'foo')

        pynode = self._app.nodes['foo.camera_viewer']
        self.assertIsNotNone(pynode)
        component = pynode['ImageViewer']
        self.assertIsNotNone(component)

        fps_readback = self._app.nodes['foo.camera_viewer'].components['ImageViewer'].config[
            'target_fps']
        self.assertEqual(fps_readback, 11)

        camera_name_readback = self._app.nodes['foo.camera_viewer'].components[
            'ImageViewer'].config['camera_name']
        self.assertEqual(camera_name_readback, 'bar')

        self._app.start()
        self._app.stop()

    def test_module_explorer(self):
        self._app = Application()
        self._app.load_module('json')
        self._app.registry.isaac.json.JsonToProto
        with self.assertRaises(RuntimeError):
            self._app.registry.isaac.foo

    def test_node_accesor(self):
        self._app = Application()
        self._app.load_module('json')

        node = self._app.add('foo1')
        self.assertIsNotNone(node)
        component = node.add(self._app.registry.isaac.json.JsonToProto, 'bar1')
        self.assertIsNotNone(component)

        node = self._app.add('foo2')
        self.assertIsNotNone(node)
        component = node.add(self._app.registry.isaac.json.ProtoToJson, 'bar2')
        self.assertIsNotNone(component)

        self.assertIsNotNone(self._app.nodes['foo1'])
        self.assertIsNotNone(self._app.nodes['foo2'])
        self.assertIsNone(self._app.nodes['foo3'])

        self._app.start()
        self._app.stop()

    def test_load_module(self):
        self._app = Application()

        result = self._app.load_module('message_generators')
        self.assertTrue(result)

        component = self._app.registry.isaac.message_generators.ImageLoader
        self.assertIsNotNone(component)
        self.assertEqual(str(component), "['isaac::message_generators::ImageLoader']")

    def test_clock(self):
        self._app = Application()

        self._app.start()
        clock = self._app.clock
        self.assertIsNotNone(clock)
        self.assertIsNotNone(clock.time)
        cur_time = clock.time
        self.assertGreater(cur_time, 0.0)
        self.assertGreater(clock.time, cur_time)
        self._app.stop()

    def test_pose(self):
        self._app = Application()

        self._app.start()

        self.assertTrue(
            self._app.atlas.set_pose('foo', 'bar', 1.0,
                                     [np.quaternion(1.0, 0.0, 0.0, 0.0),
                                      np.array([0.0, 0.0, 0.0])]))
        self.assertTrue(
            self._app.atlas.set_pose('foo', 'bar', 2.0,
                                     [np.quaternion(1.0, 0.0, 0.0, 0.0),
                                      np.array([1.0, 2.0, 3.0])]))

        read_pose = self._app.atlas.pose('foo', 'bar', 1.5)
        self.assertIsNotNone(read_pose)
        self.assertEqual(len(read_pose), 2)

        q = read_pose[0]
        t = read_pose[1]
        self.assertLess(q.w - 1.0, 1e-6)
        self.assertLess(q.x - 0.0, 1e-6)
        self.assertLess(q.y - 0.0, 1e-6)
        self.assertLess(q.z - 0.0, 1e-6)
        self.assertLess(t[0] - 0.5, 1e-6)
        self.assertLess(t[1] - 1.0, 1e-6)
        self.assertLess(t[2] - 1.5, 1e-6)

        self._app.stop()

    def test_node_start_stop(self):
        self._app = Application()

        result = self._app.load_module('message_generators')
        self.assertTrue(result)

        node = self._app.add('src')
        self.assertIsNotNone(node)
        component = node.add(self._app.registry.isaac.message_generators.PanTiltStateGenerator,
                             'pantilt')
        component.config['tick_period'] = '20 Hz'
        self.assertIsNotNone(component)

        node_sink = self._app.add('sink')
        self.assertIsNotNone(node_sink)

        self._app.connect(component, 'target', node_sink.components['MessageLedger'], 'rcv')

        self._app.start()

        time.sleep(0.1)
        msg = self._app.receive('sink', 'MessageLedger', 'rcv')
        self.assertIsNotNone(msg)

        node.stop()
        msg = self._app.receive('sink', 'MessageLedger', 'rcv')
        time.sleep(0.1)
        msg = self._app.receive('sink', 'MessageLedger', 'rcv')
        self.assertIsNone(msg)
        time.sleep(0.1)
        msg = self._app.receive('sink', 'MessageLedger', 'rcv')
        self.assertIsNone(msg)

        node.start()

        time.sleep(0.1)
        msg = self._app.receive('sink', 'MessageLedger', 'rcv')
        self.assertIsNotNone(msg)

        self._app.stop()

        with self.assertRaises(RuntimeError):
            self._app.stop()

    def test_wait_for_node(self):
        self._app = Application()
        self._app.load_module('behavior_tree')
        self._app.add('node').add(self._app.registry.isaac.behavior_tree.TimerBehavior)
        self._app.start()
        status = self._app.wait_for_node('node')
        self._app.stop()
        self.assertEqual(str(status), 'Status.Success', 'Should reach Status.Success in 1 second.')

    def test_wait_for_node_timeout(self):
        self._app = Application()
        self._app.load_module('behavior_tree')
        self._app.add('node').add(self._app.registry.isaac.behavior_tree.TimerBehavior)
        self._app.start()
        status = self._app.wait_for_node('node', duration=0.1)
        self._app.stop()
        self.assertEqual(str(status), 'Status.Running', 'Should still be in Status.Running')

    def test_run_until_succeed(self):
        now_secs = time.time()

        self._app = Application()
        self._app.add('success')

        node_success = self._app.nodes['success']
        self.assertIsNotNone(node_success)

        self._app.nodes['success'].add(SucceedLaterCodelet)

        self._app.run('success')

        delta_secs = time.time() - now_secs
        self.assertGreater(delta_secs, 0.4)

    def test_run_until_failure(self):
        now_secs = time.time()

        self._app = Application()
        self._app.add('failure')

        node_failure = self._app.nodes['failure']
        self.assertIsNotNone(node_failure)

        self._app.nodes['failure'].add(FailureLaterCodelet)

        self._app.run('failure')

        delta_secs = time.time() - now_secs
        self.assertGreater(delta_secs, 0.4)

    def test_perf_report(self):
        PERF_REPORT_PATH = '/tmp/perf_report'
        if os.path.exists(PERF_REPORT_PATH):
            os.remove(PERF_REPORT_PATH)

        self._app = Application(argv=['--performance_report_out', PERF_REPORT_PATH])
        self._app.load_module('behavior_tree')
        self._app.add('node').add(self._app.registry.isaac.behavior_tree.TimerBehavior)
        self._app.run(0.2)
        self.assertTrue(os.path.exists(PERF_REPORT_PATH))

    def test_max_duration(self):
        time_now = time.time()
        self._app = Application(argv=['--max_duration', '0.5s'])
        self._app.load_module('behavior_tree')
        self._app.add('node').add(self._app.registry.isaac.behavior_tree.TimerBehavior)
        self._app.run(2.0)
        time_dt = time.time() - time_now

        self.assertLess(time_dt, 1.0)

        time_now = time.time()
        self._app = Application(argv=['--max_duration', '0.5s'])
        self._app.load_module('behavior_tree')
        self._app.add('node').add(self._app.registry.isaac.behavior_tree.TimerBehavior)
        self._app.run(2)
        time_dt = time.time() - time_now

        self.assertLess(time_dt, 1.0)

    def test_print_node(self):
        self._app = Application()
        self._app.add('foo')

        node_names = self._app.nodes._names
        self.assertTrue(isinstance(node_names, list) and 'foo' in node_names)

        self._app.add('bar')
        node_names = self._app.nodes._names
        self.assertTrue(
            isinstance(node_names, list) and 'foo' in node_names and 'bar' in node_names)

        foo_node = None
        bar_node = None
        for n in self._app.nodes:
            if n.name == 'foo':
                foo_node = n
            if n.name == 'bar':
                bar_node = n

        self.assertIsNotNone(foo_node)
        self.assertIsNotNone(bar_node)

    def test_expand_asset(self):
        self._app = Application()
        ws, path = self._app._app.expand_asset_path('@workspace_//foo/bar')
        self.assertEqual(path, 'external/workspace_/foo/bar')
        self.assertEqual(ws, 'workspace_')

        ws1 = self._app.home_workspace_name
        self.assertEqual(len(ws1), 0)
        self._app.home_workspace_name = 'workspace_'
        ws2 = self._app.home_workspace_name
        self.assertEqual(ws2, 'workspace_')

        ws3, path3 = self._app._app.expand_asset_path('@workspace_//foo/bar')
        self.assertEqual(ws3, 'workspace_')
        self.assertEqual(path3, 'foo/bar')


if __name__ == '__main__':
    unittest.main()
