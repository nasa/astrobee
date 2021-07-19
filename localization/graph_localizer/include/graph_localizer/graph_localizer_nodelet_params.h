/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_PARAMS_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_PARAMS_H_

namespace graph_localizer {
struct GraphLocalizerNodeletParams {
  int max_imu_buffer_size;
  int max_optical_flow_buffer_size;
  int max_vl_buffer_size;
  int max_ar_buffer_size;
  int max_dl_buffer_size;
  // Used to avoid saving ml/ar poses with too few landmark detections
  int loc_adder_min_num_matches;
  int ar_tag_loc_adder_min_num_matches;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_NODELET_PARAMS_H_
