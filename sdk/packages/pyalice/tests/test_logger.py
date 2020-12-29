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


class TestLogger(unittest.TestCase):
    '''
    Test setting severity via Python API is not crashing
    '''
    def test_set_severity(self):
        isaac.set_severity(isaac.severity.DEBUG)


if __name__ == '__main__':
    unittest.main()
