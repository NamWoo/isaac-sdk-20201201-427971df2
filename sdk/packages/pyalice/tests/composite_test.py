'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import unittest
import numpy as np

from isaac import *


class TestComposite(unittest.TestCase):
    '''
    Test loading subgraph via the application API
    '''
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        # ran before each test
        return super().setUp()

    def tearDown(self):
        # ran after each test
        return super().tearDown()

    def test_roundtrip(self):
        # method will be ran once before any test is ran
        app = Application()
        pub_node = app.add("pub")
        sub_node = app.add("sub")
        app.connect(pub_node["MessageLedger"], "composite", sub_node["MessageLedger"], "composite")
        app.start()

        quantities = [[x, "position", 1] for x in ["base", "shoulder", "elbow", "wrist"]]
        values = np.random.rand(4)
        input_msg = Composite.create_composite_message(quantities, values)

        app.publish("pub", "MessageLedger", "composite", input_msg)
        output_msg = app.receive("sub", "MessageLedger", "composite")
        assert output_msg is not None

        # identical quantities should return identical messages
        out_val = Composite.parse_composite_message(output_msg, quantities)
        assert out_val is not None
        assert (out_val == values).all()

        # subset of quantities
        index = [0, 2]
        out_quantities = [quantities[x] for x in index]
        out_val = Composite.parse_composite_message(output_msg, out_quantities)
        assert out_val is not None
        assert (out_val == values[index]).all()

        # invalid quantities should return None
        out_quantities = [[x, "speed", 1] for x in ["base", "shoulder", "elbow", "wrist"]]
        with self.assertRaises(ValueError):
            Composite.parse_composite_message(output_msg, out_quantities)

        out_quantities = [[x, "position", 1] for x in ["base_joint", "shoulder_joint"]]
        with self.assertRaises(ValueError):
            Composite.parse_composite_message(output_msg, out_quantities)

        out_quantities = [[x, "position", 2] for x in ["base", "shoulder", "elbow", "wrist"]]
        with self.assertRaises(ValueError):
            Composite.parse_composite_message(output_msg, out_quantities)

        app.stop()


if __name__ == '__main__':
    unittest.main()