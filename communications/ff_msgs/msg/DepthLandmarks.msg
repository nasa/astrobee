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

Header header # image header, with time stamp
uint32 camera_id # image ID, associated with registration
uint8 end_seen # whether an end of the handrail was detected
uint8 update_global_pose # whether to update the global pose
geometry_msgs/Pose local_pose
ff_msgs/DepthLandmark[] landmarks # a list of landmarks seen
