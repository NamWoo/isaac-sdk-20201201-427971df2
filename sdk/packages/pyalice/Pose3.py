'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import quaternion
import numpy as np


class Pose3:
    def __init__(self, pose):
        if not isinstance(pose[0], np.quaternion) or not isinstance(pose[1], np.ndarray):
            raise ValueError('Invalid parameters for setting pose')

        self.rotation = pose[0]
        self.translation = pose[1]

    def __mul__(self, other):
        return Pose3([self.rotation * other.rotation, \
                      quaternion.rotate_vectors(self.rotation, other.translation) + \
                      self.translation])

    def __str__(self):
        out = np.concatenate((quaternion.as_float_array(self.rotation), self.translation)).__str__()
        return ', '.join(filter(None, out.split(' ')))

    def inverse(self):
        qinv = self.rotation.inverse()
        return Pose3([qinv, -quaternion.rotate_vectors(qinv, self.translation)])

    @staticmethod
    def Identity():
        return Pose3([quaternion.from_euler_angles(0.0, 0.0, 0.0), np.array([0.0, 0.0, 0.0])])
