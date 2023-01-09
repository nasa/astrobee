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
# An argument to a command sent through RAPID
#
# Note that this is approximating a union in DDS. However, this is an
# inefficient union, and thus each instance will take up at least 89 bytes.
# However, even with the maximum of 16 arguments to a command, we only have
# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.

uint8 DATA_TYPE_BOOL     = 0
uint8 DATA_TYPE_DOUBLE   = 1
uint8 DATA_TYPE_FLOAT    = 2
uint8 DATA_TYPE_INT      = 3
uint8 DATA_TYPE_LONGLONG = 4
uint8 DATA_TYPE_STRING   = 5
uint8 DATA_TYPE_VEC3D    = 6
uint8 DATA_TYPE_MAT33F   = 7

uint8 data_type

bool b
float64 d
float32 f
int32 i
int64 ll
string s
float64[3] vec3d
float32[9] mat33f

