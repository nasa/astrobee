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

#include <opencv2/video/tracking.hpp>

namespace depth_odometry {
LKOpticalFlowFeatureDetectorAndMatcher::LKOpticalFlowFeatureDetectorAndMatcher(
  const LKOpticalFlowFeatureDetectorAndMatcherParams& params)
    : params_(params) {
  detector_.reset(new cv::GoodFeaturesToTrackDetector());
}

FeatureMatches LKOpticalFlowFeatureDetectorAndMatcher::Match(const FeatureImage& image_a, const FeatureImage& image_b) {
  const cv::TermCriteria termination_criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, params_.max_iterations,
                                              params_.termination_epsilon);
  const cv::Size window_size(params_.window_width, params_.window_height);
  // Perform forward and backward passes on detected features
  std::vector<cv::Point2f> matched_forward_features;
  std::vector<uchar> forward_status;
  std::vector<float> forward_errors;
  cv::calcOpticalFlowPyrLK(image_a.image(), image_b.image(), image_a.feature_points(), matched_forward_features,
                           forward_status, forward_errors, window_size, params_.max_level, termination_criteria, 0,
                           params_.min_eigen_threshold);
  std::vector<cv::Point2f> matched_backward_features;
  std::vector<uchar> backward_status;
  std::vector<float> backward_errors;
  cv::calcOpticalFlowPyrLK(image_b.image(), image_a.image(), matched_forward_features, matched_backward_features,
                           backward_status, backward_errors, window_size, params_.max_level, termination_criteria, 0,
                           params_.min_eigen_threshold);

  // Find matches between forward and backward passes
  FeatureMatches matches;
  for (int i = 0; i < static_cast<int>(matched_forward_features.size()); ++i) {
    const auto& point_a = image_a.feature_points()[i];
    const auto& forward_point = matched_forward_features[i];
    if (!(forward_status[i] && backward_status[i])) continue;
    const bool valid_forward_match = cv::norm(point_a - forward_point) <= params_.max_flow_distance;
    const double backward_match_distance = cv::norm(point_a - matched_backward_features[i]);
    const bool valid_backward_match = backward_match_distance <= params_.max_backward_match_distance;
    if (!(valid_forward_match && valid_backward_match)) continue;
    matches.emplace_back(Eigen::Vector2d(point_a.x, point_a.y), Eigen::Vector2d(forward_point.x, forward_point.y),
                         backward_match_distance);
  }
  return matches;
}
}  // namespace depth_odometry
