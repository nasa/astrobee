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
# An observation of a handrail from a depth image.

std_msgs/Header header                                  # Image header, with time stamp
uint32 camera_id                               # Image ID, associated with registration
uint8 end_seen                                 # Whether the handrail endpoint was detected
uint8 update_global_pose                       # Whether to update the global pose
{% if ROS1 %}
geometry_msgs/Pose sensor_T_handrail           # Handrail center in the sensor frame
{% else %}
geometry_msgs/Pose sensor_t_handrail           # Handrail center in the sensor frame
{% endif %}
geometry_msgs/Point32[] sensor_t_line_points   # Detected line points
geometry_msgs/Point[] sensor_t_line_endpoints  # Detected line endpoints
geometry_msgs/Point32[] sensor_t_plane_points  # Detected plane points
ff_msgs/DepthLandmark[] landmarks              # List of landmarks seen TODO(rsoussan): This should be removed
