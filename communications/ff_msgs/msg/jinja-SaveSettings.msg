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

# The save settings message contains information about the topics currently
# being recorded.

# Name of topic
string topic_name

# Topic saved to disk; upon docking it is downlinked
uint8 IMMEDIATE   = 0

# Topic saved to disk; upon docking it is transferred to ISS server for later
# downlink
uint8 DELAYED     = 1

# Downlink option indicates if and when the data in the rostopic is downlinked
{%- if USE_ROS1 -%}
uint8 downlinkOption
{%- else -%}
uint8 downlink_option
{%- endif -%}
# Times per second to save the data (Hz)
float32 frequency
