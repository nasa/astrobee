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
#ifndef DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_PARAMS_H_
#define DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_PARAMS_H_

namespace depth_odometry {
struct LKOpticalFlowFeatureDetectorAndMatcherParams {
  int max_iterations;
  double termination_epsilon;
  int window_width;
  int window_height;
  int max_level;
  double min_eigen_threshold;
  double max_flow_distance;
  double max_backward_match_distance;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_PARAMS_H_
