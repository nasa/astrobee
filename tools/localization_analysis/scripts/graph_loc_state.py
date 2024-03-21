#!/usr/bin/python3
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

# GraphLocState object containing information from a GraphLocState Msg
class GraphLocState():
    def __init__(self):
        self.timestamp = None
        self.pose_with_covariance = None
        self.num_detected_ar_features = None
        self.num_detected_ml_features = None
        self.optimization_iterations = None
        self.optimization_time = None
        self.update_time = None
        self.num_factors = None
        self.num_ml_projection_factors = None
        self.num_ml_pose_factors = None
        self.num_states = None
        # TODO: change this using bag start time??
