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
# A compressed file that represents a CompressedFile from
# DDS. Used to send a compressed files to the executive.
# DDS is constrained to a 128k chunk of data, we are only
# limited by our imaginations (and the size of an unsigned
# 32-bit integer)

# Header 
std_msgs/Header header

# Unique file identification
int32 id

uint8 TYPE_NONE = 0
uint8 TYPE_DEFLATE = 1
uint8 TYPE_BZ2 = 2
uint8 TYPE_GZ = 3
uint8 TYPE_ZIP = 4

# Type of compression
uint8 type

# File contents
uint8[] file
