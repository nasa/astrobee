#!/usr/bin/python
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
import timestamped_pose_with_covariance
import timestamped_velocity 
import scipy.spatial.transform

# Subtract the bag start time from the timestamp
# to make start time relative to the bag start time 
def relative_timestamp(timestamp, bag_start_time):
    return timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time

# Create a timestamped pose with covariance from a pose msg using relative bag time. 
def pose_from_msg(pose_msg, bag_start_time=0):
    orientation = scipy.spatial.transform.Rotation.from_quat(
        [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ]
    )
    self.add_orientation_and_position(
        orientation, pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
    )
    timestamp = relative_timestamp(msg.header.stamp, bag_start_time)
    return TimestampedPoseWithCovariance(orientation, [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z], msg.covariance, pose_timestamp)

# Create a timestamped velocity from a velocity msg using relative bag time. 
def velocity_from_msg(velocity_msg, bag_start_time=0):
    timestamp = relative_timestamp(velocity_msg.header.stamp, bag_start_time)
    return TimestampedVelocity(velocity_msg.x, velocity_msg.y, velocity_msg.z, timestamp)
