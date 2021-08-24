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

#include <depth_odometry/surf_feature_detector_and_matcher.h>
#include <depth_odometry/surf_feature_detector_and_matcher_params.h>
#include <localization_common/logger.h>

#include <opencv2/xfeatures2d.hpp>

namespace depth_odometry {
SurfFeatureDetectorAndMatcher::SurfFeatureDetectorAndMatcher(const SurfFeatureDetectorAndMatcherParams& params)
    : params_(params) {
  detector_ = cv::xfeatures2d::SURF::create(params_.surf_threshold);
}
std::vector<cv::DMatch> SurfFeatureDetectorAndMatcher::Match(const FeatureImage& image_a, const FeatureImage& image_b) {
  std::vector<cv::DMatch> matches;
  flann_matcher_.match(image_a.descriptors(), image_b.descriptors(), matches);
  LogError("matches pre filtering: " << matches.size());
  const auto filtered_end = std::remove_if(matches.begin(), matches.end(), [this](const cv::DMatch& match) {
    return match.distance > params_.max_match_distance;
  });
  matches.erase(filtered_end, matches.end());
  LogError("keypoints a: " << image_a.keypoints().size() << ", b: " << image_b.keypoints().size());
  LogError("matches post filtering: " << matches.size());
  return matches;
}
}  // namespace depth_odometry
