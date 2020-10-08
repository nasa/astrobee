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
# Memory state message with timestamp.

# Header with timestamp
std_msgs/Header header

# Machine name (llp, hlp, mlp, etc)
string name

# Physical Memory (RAM)
uint32 ram_total   # total physical memeory in system Mb
uint32 ram_used    # totalphysical memeory used in Mb

# Virtual Memory
uint32 virt_total  # total virtual memeory in system in Mb
uint32 virt_used   # total virtual memeory used in Mb

# Individual nodes
ff_msgs/MemState[] nodes


