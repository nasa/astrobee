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
# Fault data messsage contains information of why the fault occurred

uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float
uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int
uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string

string key  # Specifies what the data in the msg is, can only be 32 chars long

uint8 data_type   # Specifies the type of data in the message

float32 f   # Value used for fault analysis, data_type must be 0
int32 i     # Value used for fault analysis, data_type must be 1
string s    # String used for fault analysis, data_type must be 2
