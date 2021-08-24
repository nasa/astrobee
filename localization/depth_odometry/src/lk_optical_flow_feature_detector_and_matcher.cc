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

#include <depth_odometry/good_features_to_track_detector.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher.h>
#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher_params.h>
#include <localization_common/logger.h>

namespace depth_odometry {
LKOpticalFlowFeatureDetectorAndMatcher::LKOpticalFlowFeatureDetectorAndMatcher(
  const LKOpticalFlowFeatureDetectorAndMatcherParams& params)
    : params_(params) {
  detector_.reset(new cv::GoodFeaturesToTrackDetector());
}

std::vector<cv::DMatch> LKOpticalFlowFeatureDetectorAndMatcher::Match(const FeatureImage& image_a,
                                                                      const FeatureImage& image_b) {
  std::vector<cv::DMatch> matches;
  return matches;
}
}  // namespace depth_odometry
