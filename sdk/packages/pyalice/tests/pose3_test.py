'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import numpy as np
import quaternion
import unittest

from isaac import Pose3


class TestPose3(unittest.TestCase):
    '''
    Test the Pose3 class
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

    def testInit(self):
        pose = Pose3([quaternion.from_euler_angles(0.0, 0.0, 0.0), np.array([1.0, 0.0, 0.0])])

        self.assertEqual(pose.rotation, quaternion.from_float_array([1, 0, 0, 0]))
        self.assertEqual(np.linalg.norm(pose.translation), 1.0)

    def testInverse(self):
        pose = Pose3([quaternion.from_euler_angles(0.0, 0.0, 0.0), np.array([1.0, 0.0, 0.0])])
        self.assertEqual(pose.inverse().rotation, quaternion.from_float_array([1, 0, 0, 0]))
        self.assertTrue(np.array_equal(pose.inverse().translation, np.array([-1.0, 0.0, 0.0])))

    def testMult(self):
        pose1 = Pose3([quaternion.from_euler_angles(0.0, 0.0, 0.0), np.array([1.0, 0.0, 0.0])])
        pose2 = Pose3([quaternion.from_euler_angles(0.0, 0.0, 180), np.array([0.0, 0.0, 0.0])])
        pose3 = pose1 * pose2

        # Multiplying by pose with no translation component should not change translation
        self.assertTrue(np.array_equal(pose3.translation, np.array([1.0, 0.0, 0.0])))


if __name__ == '__main__':
    unittest.main()
