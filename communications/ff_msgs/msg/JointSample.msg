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
#
# JointSample message, based off of rapid::JointSample

# Flag values for joint status. 
# Joint is enabled
uint8 JOINT_ENABLED    = 0      # Joint enabled
uint8 JOINT_DISABLED   = 1      # Joint disabled


# Angle position (in radians) of the joint
float32 angle_pos

# Angle velocity (in radians/sec) of the joint
float32 angle_vel

# Angle acceleration (in radians/sec^2) of the joint (not being used)
float32 angle_acc

# Current draw of joint motor
float32 current

# Torque sensed at the joint (not being used)
float32 torque

# Temperature of the joint (in Celsius)
float32 temperature

# Bit field representing the state of the joint
uint16 status

# Human-readable name
string name
