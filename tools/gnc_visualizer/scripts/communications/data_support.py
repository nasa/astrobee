#!/usr/bin/env python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import numpy as np


def todict(obj):
    if hasattr(obj, "__iter__"):
        return [todict(v) for v in obj]
    elif hasattr(obj, "__dict__"):
        return dict(
            [
                (key, todict(value))
                for key, value in list(obj.__dict__.items())
                if not callable(value) and not key.startswith("_")
            ]
        )
    else:
        return obj


class Common:
    def asDict(self):
        return todict(self)


class Header(Common):
    def __init__(self, seq=None, stamp=None, frame_id=None):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class Point(Common):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Quaternion(Common):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Pose(Common):
    def __init__(self, position=Point(), orientation=Quaternion()):
        self.position = position
        self.orientation = orientation


class Vector3(Common):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Wrench(Common):
    def __init__(self, force=Vector3(), torque=Vector3()):
        self.force = force
        self.torque = torque


class Twist(Common):
    def __init__(self, linear=Vector3(), angular=Vector3()):
        self.linear = linear
        self.angular = angular


class PoseStamped(Common):
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class EkfState(Common):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = None
        self.pose = Pose()
        self.velocity = Vector3()
        self.omega = Vector3()
        self.gyro_bias = Vector3()
        self.accel = Vector3()
        self.accel_bias = Vector3()
        # self.cov_diag = [0.0] * 15
        self.cov_diag = np.zeros(15)
        self.confidence = 0
        self.aug_state_enum = 0
        self.status = 0
        self.of_count = 0
        self.ml_count = 0
        self.hr_global_pose = Pose()
        # self.ml_mahal_dists = [0.0] * 50
        self.ml_mahal_dists = np.zeros(50)


class PmcCommand(Common):
    def __init__(self, header=Header()):
        self.header = header
        self.goals = []

    class PmcGoal(Common):
        def __init__(self, motor_speed=None):
            self.motor_speed = motor_speed
            self.nozzle_positions = []

    def to_dict(self):
        dic = dict()
        dic["hdr"] = self.header
        dic["goals"] = []

        for i in range(0, len(self.goals)):
            goal = {"motorSpeed": self.goals[i].motor_speed, "nozzlePositions": []}
            for j in range(0, len(self.goals[i].nozzle_positions)):
                goal["nozzlePositions"].append(self.goals[i].nozzle_positions[j])
            dic["goals"].append(goal)

        return dic


class FamCommand(Common):
    def __init__(self):
        self.header = Header()
        self.wrench = Wrench()
        self.accel = Vector3()
        self.alpha = Vector3()
        self.status = 0
        self.position_error = Vector3()
        self.position_error_integrated = Vector3()
        self.attitude_error = Vector3()
        self.attitude_error_integrated = Vector3()
        self.attitude_error_mag = 0.0
        self.control_mode = 0


class ControlState(Common):
    def __init__(self):
        self.when = None
        self.pose = Pose()
        self.twist = Twist()
        self.accel = Twist()


class Log(Common):
    def __init__(self):
        self.header = Header()
        self.level = None
        self.name = None
        self.msg = None
