'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import unittest
import time

from isaac import Application


class TestPythonAppConfig(unittest.TestCase):
    '''
    Test setting and getting Isaac configuration parameters.
    '''
    @classmethod
    def setUpClass(cls):
        # method will be ran once before any test is ran
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.app = Application('packages/pyalice/tests/pyapp_config_test.app.json')
        self.app.start()

    def tearDown(self):
        self.app.stop()

    def test_get_config(self):
        dummy_bool = self.app.nodes["node"].components["ledger"].config["dummy_bool"]
        self.assertEqual(dummy_bool, True)

        dummy_float = self.app.nodes["node"].components["ledger"].config["dummy_float"]
        self.assertEqual(dummy_float, 1.111)

        dummy_list = self.app.nodes["node"].components["ledger"].config["dummy_list"]
        self.assertEqual(dummy_list, [1, 2, 3])

        dummy_dict = self.app.nodes["node"].components["ledger"].config["dummy_dict"]
        self.assertEqual(dummy_dict['value_int'], 100)
        self.assertEqual(dummy_dict['value_string'], 'hi')

    def test_set_config(self):
        self.app.nodes["node"].components["ledger"].config["dummy_bool"] = False
        dummy_bool = self.app.nodes["node"].components["ledger"].config["dummy_bool"]
        self.assertEqual(dummy_bool, False)

        self.app.nodes["node"].components["ledger"].config["dummy_float"] = 2.222
        dummy_float = self.app.nodes["node"].components["ledger"].config["dummy_float"]
        self.assertEqual(dummy_float, 2.222)

        self.app.nodes["node"].components["ledger"].config["dummy_list"] = [4, 5, 6]
        dummy_list = self.app.nodes["node"].components["ledger"].config["dummy_list"]
        self.assertEqual(dummy_list, [4, 5, 6])

        self.app.nodes["node"].components["ledger"].config["dummy_dict"] = {
            'value_int': 200,
            'value_string': 'hello'
        }
        dummy_dict = self.app.nodes["node"].components["ledger"].config["dummy_dict"]
        self.assertEqual(dummy_dict['value_int'], 200)
        self.assertEqual(dummy_dict['value_string'], 'hello')

    def test_erase_config(self):
        component_config = self.app.nodes["node"].components["ledger"].config
        dummy_float = component_config["dummy_float"]
        self.assertEqual(dummy_float, 1.111)

        del component_config.dummy_float
        dummy_float = self.app.nodes["node"].components["ledger"].config["dummy_float"]
        self.assertIsNone(dummy_float)

        component_config.dummy_float = 2.0
        dummy_float = component_config["dummy_float"]
        self.assertEqual(dummy_float, 2.0)

        del component_config['dummy_float']
        dummy_float = self.app.nodes["node"].components["ledger"].config["dummy_float"]
        self.assertIsNone(dummy_float)


if __name__ == '__main__':
    unittest.main()
