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
#ifndef DEPTH_ODOMETRY_BRISK_FEATURE_DETECTOR_AND_MATCHER_H_
#define DEPTH_ODOMETRY_BRISK_FEATURE_DETECTOR_AND_MATCHER_H_

#include <depth_odometry/brisk_feature_detector_and_matcher_params.h>
#include <depth_odometry/feature_detector_and_matcher.h>
#include <localization_measurements/feature_image.h>

namespace depth_odometry {
class BriskFeatureDetectorAndMatcher : public FeatureDetectorAndMatcher {
 public:
  BriskFeatureDetectorAndMatcher(const BriskFeatureDetectorAndMatcherParams& params);
  FeatureMatches Match(const localization_measurements::FeatureImage& source_image,
                       const localization_measurements::FeatureImage& target_image) final;

 private:
  BriskFeatureDetectorAndMatcherParams params_;
  cv::FlannBasedMatcher flann_matcher_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_BRISK_FEATURE_DETECTOR_AND_MATCHER_H_
