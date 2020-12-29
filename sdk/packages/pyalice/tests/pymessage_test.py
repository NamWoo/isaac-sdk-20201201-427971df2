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
import time
from isaac import Application, Message


class TestPythonMessage(unittest.TestCase):
    '''
    Test receiving messages via the application message API
    '''
    @classmethod
    def setUpClass(cls):
        # method will be ran once before any test is ran
        cls.app = Application('packages/pyalice/tests/pymessage_test.app.json')
        cls.app.start()

    @classmethod
    def tearDownClass(cls):
        # method will be ran once after all tests have run
        cls.app.stop()

    def setUp(self):
        # make sure the app ticks at least one time between tests
        time.sleep(0.5)

    def test_receive_message_as_json(self):
        msg = self.app.receive("mock", "Detections2Generator", "mock_detections")

        self.assertNotEqual(msg.type_id, 0)
        self.assertGreater(msg.acqtime, 0)
        self.assertEqual(msg.json["predictions"][0]["label"], "A")

    def test_receive_message_as_proto(self):
        msg = self.app.receive("mock", "Detections2Generator", "mock_detections")

        self.assertNotEqual(msg.type_id, 0)
        self.assertGreater(msg.acqtime, 0)
        self.assertEqual(msg.proto.predictions[0].label, "A")

    def test_receive_message_tensor(self):
        msg = self.app.receive("mock", "CameraGenerator", "color_left")

        self.assertGreater(msg.acqtime, 0)
        self.assertIsNotNone(msg.tensor)

    def test_no_message_available(self):
        msg = self.app.receive("node", "component", "channel")
        self.assertIsNone(msg)

    def test_send_message_with_acqtime(self):
        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        send_msg.acqtime = 10
        self.app.publish('node', 'ledger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'ledger', 'in')
        self.assertEqual(recv_msg.proto.message, 'payload')
        self.assertEqual(recv_msg.acqtime, 10)

    def test_send_message_has_metadata(self):
        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        self.app.publish('node', 'ledger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'ledger', 'in')
        self.assertGreater(recv_msg.pubtime, 0)
        self.assertNotEqual(recv_msg.uuid, "")

    def test_send_message_with_buffer(self):
        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        buffer = np.empty(3, dtype=np.dtype('B'))
        buffer[0] = 1
        buffer[1] = 11
        buffer[2] = 111
        send_msg.buffers = [buffer]
        self.app.publish('node', 'ledger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'ledger', 'in')
        self.assertEqual(recv_msg.buffers[0][0], 1)
        self.assertEqual(recv_msg.buffers[0][1], 11)
        self.assertEqual(recv_msg.buffers[0][2], 111)

    def test_send_message_proto(self):
        send_msg = Message.create_message_builder('PingProto')
        send_msg.proto.message = 'payload'
        self.app.publish('node', 'ledger', 'in', send_msg)

        recv_msg = self.app.receive('node', 'ledger', 'in')
        self.assertNotEqual(recv_msg.type_id, 0)
        self.assertEqual(recv_msg.proto.message, 'payload')


if __name__ == '__main__':
    unittest.main()
