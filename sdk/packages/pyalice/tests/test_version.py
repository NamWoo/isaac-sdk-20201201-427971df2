'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import unittest
import isaac


class TestVersion(unittest.TestCase):
    '''
    Test loading subgraph via the application API
    '''
    def test_version(self):
        ver = isaac.bindings.isaac_sdk_version()
        self.assertIsNotNone(ver)


if __name__ == '__main__':
    unittest.main()
