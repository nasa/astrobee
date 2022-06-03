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
#ifndef VISION_COMMON_SURF_FEATURE_DETECTOR_AND_MATCHER_H_
#define VISION_COMMON_SURF_FEATURE_DETECTOR_AND_MATCHER_H_

#include <vision_common/feature_detector_and_matcher.h>
#include <vision_common/feature_image.h>
#include <vision_common/surf_feature_detector_and_matcher_params.h>

namespace vision_common {
class SurfFeatureDetectorAndMatcher : public FeatureDetectorAndMatcher {
 public:
  explicit SurfFeatureDetectorAndMatcher(const SurfFeatureDetectorAndMatcherParams& params);
  FeatureMatches Match(const FeatureImage& source_image, const FeatureImage& target_image) final;

 private:
  SurfFeatureDetectorAndMatcherParams params_;
  cv::FlannBasedMatcher flann_matcher_;
};
}  // namespace vision_common

#endif  // VISION_COMMON_SURF_FEATURE_DETECTOR_AND_MATCHER_H_
