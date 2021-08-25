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
#ifndef DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_H_
#define DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_H_

#include <depth_odometry/lk_optical_flow_feature_detector_and_matcher_params.h>
#include <depth_odometry/feature_image.h>
#include <depth_odometry/feature_detector_and_matcher.h>

namespace depth_odometry {
class LKOpticalFlowFeatureDetectorAndMatcher : public FeatureDetectorAndMatcher {
 public:
  LKOpticalFlowFeatureDetectorAndMatcher(const LKOpticalFlowFeatureDetectorAndMatcherParams& params);
  FeatureMatches Match(const FeatureImage& image_a, const FeatureImage& image_b) final;

 private:
  LKOpticalFlowFeatureDetectorAndMatcherParams params_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_LK_OPTICAL_FLOW_FEATURE_DETECTOR_AND_MATCHER_H_
