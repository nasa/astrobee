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
#ifndef VISION_COMMON_GOOD_FEATURES_TO_TRACK_DETECTOR_PARAMS_H_
#define VISION_COMMON_GOOD_FEATURES_TO_TRACK_DETECTOR_PARAMS_H_

namespace vision_common {
struct GoodFeaturesToTrackDetectorParams {
  int max_corners;
  double quality_level;
  double min_distance;
  int block_size;
  bool use_harris_detector;
  double k;
};
}  // namespace vision_common

#endif  // VISION_COMMON_GOOD_FEATURES_TO_TRACK_DETECTOR_PARAMS_H_
