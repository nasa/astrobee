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

std::vector<cv::DMatch> LKOpticalFlowFeatureDetectorAndMatcher::Match(const FeatureImage& image_a,
                                                                      const FeatureImage& image_b) {
  // TODO(rsoussan): Make these params!
  const int max_iterations = 10;
  const double termination_epsilon = 0.03;
  const cv::TermCriteria termination_criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, max_iterations, termination_epsilon);
  const int window_width = 31;
  const int window_height = 31;
  const cv::Size window_size(window_width, window_height);
  const int max_level = 3;
  const double min_eigen_threshold = 0.001;
  // Perform forward and backward passes on detected features
  std::vector<cv::Point2f> matched_forward_features;
  std::vector<uchar> forward_status;
  std::vector<float> forward_errors;
  cv::calcOpticalFlowPyrLK(image_a.image(), image_b.image(), image_a.feature_points(), matched_forward_features,
                           forward_status, forward_errors, window_size, max_level, termination_criteria, 0,
                           min_eigen_threshold);
  std::vector<cv::Point2f> matched_backward_features;
  std::vector<uchar> backward_status;
  std::vector<float> backward_errors;
  cv::calcOpticalFlowPyrLK(image_b.image(), image_a.image(), matched_forward_features, matched_backward_features,
                           backward_status, backward_errors, window_size, max_level, termination_criteria, 0,
                           min_eigen_threshold);

  // Remove corners with false status and with large displacements
  // RefineCorners();

  std::vector<cv::DMatch> matches;
  return matches;
}

/*void LKOpticalFlowFeatureDetectorAndMatcher::RefineCorners() {
  // Remove corners with 'false' status and with more displacement than max_flow_magnitude
  size_t k = 0;
  for (size_t i = 0; i < curr_corners_.size(); ++i) {
    // ignore pixels that weren't tracked, were too far away, and are near the image border
    bool too_far = cv::norm(prev_corners_[i] - curr_corners_[i]) > max_flow_magnitude_;
    //int x_border_dist = std::min(curr_corners_[i].x, image_curr_.cols - curr_corners_[i].x);
    //int y_border_dist = std::min(curr_corners_[i].y, image_curr_.rows - curr_corners_[i].y);
    //bool on_border = x_border_dist < 10 || y_border_dist < 10;
    //bool on_corner = x_border_dist < 60 && y_border_dist < 60;
    bool backwards_ok = cv::norm(prev_corners_[i] - backwards_corners_[i]) < 0.5 &&
                        backwards_status_[i];

    // ignore points that don't have the same matching in both directions
    if (status_[i] && !too_far && !on_border && !on_corner && backwards_ok) {
      curr_corners_[k] = curr_corners_[i];
      prev_corners_[k] = prev_corners_[i];
      id_list_[k] = id_list_[i];
      ++k;
    }
  }
  // printf("Filtered corners from %lu to %lu.\n", corners_[1].size(), k);
  curr_corners_.resize(k);
  prev_corners_.resize(k);
  id_list_.resize(k);
}*/

}  // namespace depth_odometry
