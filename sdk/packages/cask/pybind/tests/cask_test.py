'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import numpy as np
import unittest
import tempfile
import time
import uuid
from isaac import Application, Cask, Message, Node


class TestCask(unittest.TestCase):
    '''
    Test receiving messages via the application message API
    '''
    @classmethod
    def setUpClass(cls):
        # method will be ran once before any test is ran
        cls.app = Application()
        cls.app.add('node')
        cls.app.start()

    @classmethod
    def tearDownClass(cls):
        # method will be ran once after all tests have run
        cls.app.stop()

    def setUp(self):
        # ran before each test
        self._cask_folder = tempfile.mkdtemp()
        return super().setUp()

    def tearDown(self):
        # ran after each test
        return super().tearDown()

    def test_write_channel_msg(self):
        uuid_str = 'eea8e2cc-a352-4bb5-b34d-5d453dde8c7e'
        cask = Cask(str(self._cask_folder), writable=True)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        buffer = np.empty(3, dtype=np.dtype('B'))
        buffer[0] = 1
        buffer[1] = 11
        buffer[2] = 111
        send_msg.buffers = [buffer]
        send_msg.uuid = uuid_str
        self.app.publish('node', 'MessageLedger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'MessageLedger', 'in')
        cask.write_message(recv_msg)

        # Writes channel index message which is required upon reading back
        send_msg = Message.create_message_builder('MessageChannelIndexProto')
        send_msg.uuid = str(uuid.UUID('6d73675f-6368-6e6c-5f69-647800000000'))
        self.app.publish('node', 'MessageLedger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'MessageLedger', 'in')
        cask.write_message(recv_msg)

        cask = Cask(self._cask_folder)
        # Verifies read-back proto message
        msg = cask.read_message(uuid_str)
        self.assertIsNotNone(msg)
        self.assertEqual(str(msg.uuid), uuid_str, 'Mismatching uuid')
        self.assertIsNotNone(msg.proto)
        self.assertEqual(msg.proto.message, 'payload')
        # Verifies read-back buffer
        self.assertEqual(len(msg.buffers), 1, 'Mismatching number of tensor')
        self.assertEqual(msg.buffers[0][0], 1)
        self.assertEqual(msg.buffers[0][1], 11)
        self.assertEqual(msg.buffers[0][2], 111)

    def test_write_proto_msg(self):
        uuid_str = 'eea8e2cc-a352-4bb5-b34d-5d453dde8c7e'
        cask = Cask(str(self._cask_folder), writable=True)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        buffer = np.empty(3, dtype=np.dtype('B'))
        buffer[0] = 1
        buffer[1] = 11
        buffer[2] = 111
        send_msg.buffers = [buffer]
        send_msg.uuid = uuid_str

        send_msg.acqtime = 1234
        send_msg.pubtime = 5678

        cask.write_message(send_msg)

        # Writes channel index message which is required upon reading back
        send_msg = Message.create_message_builder('MessageChannelIndexProto')
        send_msg.uuid = str(uuid.UUID('6d73675f-6368-6e6c-5f69-647800000000'))
        cask.write_message(send_msg)

        cask = Cask(self._cask_folder)
        # Verifies read-back proto message
        msg = cask.read_message(uuid_str)
        self.assertIsNotNone(msg)
        self.assertEqual(str(msg.uuid), uuid_str, 'Mismatching uuid')
        self.assertIsNotNone(msg.proto)
        self.assertEqual(msg.proto.message, 'payload')
        # Verifies read-back buffer
        self.assertEqual(len(msg.buffers), 1, 'Mismatching number of tensor')
        self.assertEqual(msg.buffers[0][0], 1)
        self.assertEqual(msg.buffers[0][1], 11)
        self.assertEqual(msg.buffers[0][2], 111)
        # Verifies time
        self.assertEqual(msg.acqtime, 1234)
        self.assertEqual(msg.pubtime, 5678)

    def test_write_builder_msg(self):
        uuid_str = 'eea8e2cc-a352-4bb5-b34d-5d453dde8c7e'
        cask = Cask(str(self._cask_folder), writable=True)

        node = self.app.nodes['node']
        self.assertIsNotNone(node)

        component = node['MessageLedger']
        self.assertIsNotNone(component)

        write_channel = cask.open_channel(component, 'foo_tag')
        self.assertIsNotNone(write_channel)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        buffer = np.empty(3, dtype=np.dtype('B'))
        buffer[0] = 1
        buffer[1] = 11
        buffer[2] = 111
        send_msg.buffers = [buffer]
        send_msg.uuid = uuid_str

        send_msg.acqtime = 1234
        send_msg.pubtime = 5678

        write_channel.write_message(send_msg)

        # Verifies
        cask = Cask(self._cask_folder)
        channels = cask.channels
        self.assertIsNotNone(channels)
        self.assertEqual(len(channels), 1)

        read_channel = channels[0]
        self.assertIsNotNone(read_channel)
        msg = read_channel[0]

        self.assertIsNotNone(msg)
        self.assertEqual(str(msg.uuid), uuid_str, 'Mismatching uuid')
        self.assertIsNotNone(msg.proto)
        self.assertEqual(msg.proto.message, 'payload')
        # Verifies read-back buffer
        self.assertEqual(len(msg.buffers), 1, 'Mismatching number of tensor')
        self.assertEqual(msg.buffers[0][0], 1)
        self.assertEqual(msg.buffers[0][1], 11)
        self.assertEqual(msg.buffers[0][2], 111)
        # Verifies time
        self.assertEqual(msg.acqtime, 1234)
        self.assertEqual(msg.pubtime, 5678)

    def test_write_reader_msg(self):
        uuid_str = 'eea8e2cc-a352-4bb5-b34d-5d453dde8c7e'
        cask = Cask(str(self._cask_folder), writable=True)

        node = self.app.nodes['node']
        self.assertIsNotNone(node)

        component = node['MessageLedger']
        self.assertIsNotNone(component)

        write_channel = cask.open_channel(component, 'foo_tag')
        self.assertIsNotNone(write_channel)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        buffer = np.empty(3, dtype=np.dtype('B'))
        buffer[0] = 1
        buffer[1] = 11
        buffer[2] = 111
        send_msg.buffers = [buffer]
        send_msg.uuid = uuid_str
        send_msg.acqtime = 1234
        self.app.publish('node', 'MessageLedger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'MessageLedger', 'in')

        write_channel.write_message(recv_msg)

        # Verifies
        cask = Cask(self._cask_folder)
        channels = cask.channels
        self.assertIsNotNone(channels)
        self.assertEqual(len(channels), 1)

        read_channel = channels[0]
        self.assertIsNotNone(read_channel)
        msg = read_channel[0]

        self.assertIsNotNone(msg)
        self.assertEqual(str(msg.uuid), uuid_str, 'Mismatching uuid')
        self.assertIsNotNone(msg.proto)
        self.assertEqual(msg.proto.message, 'payload')
        # Verifies read-back buffer
        self.assertEqual(len(msg.buffers), 1, 'Mismatching number of tensor')
        self.assertEqual(msg.buffers[0][0], 1)
        self.assertEqual(msg.buffers[0][1], 11)
        self.assertEqual(msg.buffers[0][2], 111)
        # Verifies time
        self.assertEqual(msg.acqtime, 1234)
        self.assertEqual(msg.pubtime, recv_msg.pubtime)

    def test_read_series(self):
        uuid_str = 'eea8e2cc-a352-4bb5-b34d-5d453dde8c7e'
        cask = Cask(str(self._cask_folder), writable=True)

        node = self.app.nodes['node']
        self.assertIsNotNone(node)

        component = node['MessageLedger']
        self.assertIsNotNone(component)

        write_channel = cask.open_channel(component, 'foo_tag')
        self.assertIsNotNone(write_channel)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload1'
        send_msg.uuid = uuid_str

        send_msg.acqtime = 1234
        send_msg.pubtime = 5678

        write_channel.write_message(send_msg)

        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload2'
        send_msg.uuid = str(uuid.uuid4())

        send_msg.acqtime = 2345
        send_msg.pubtime = 6789

        write_channel.write_message(send_msg)

        # Verifies
        cask = Cask(self._cask_folder)
        channels = cask.channels
        self.assertIsNotNone(channels)
        self.assertEqual(len(channels), 1)

        series = channels[0].metadata
        self.assertIsNotNone(series)
        self.assertEqual(len(series), 2)

        self.assertEqual(series[0][1], uuid_str)
        self.assertTrue(isinstance(series[0][0], int))
        self.assertEqual(series[0][0], 5678)
        self.assertEqual(series[1][0], 6789)


if __name__ == '__main__':
    unittest.main()
