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

#include <vision_common/test_utilities.h>

#include <opencv2/imgproc.hpp>

namespace vision_common {
LKOpticalFlowFeatureDetectorAndMatcherParams DefaultLKOpticalFlowFeatureDetectorAndMatcherParams() {
  LKOpticalFlowFeatureDetectorAndMatcherParams params;
  params.max_iterations = 10;
  params.termination_epsilon = 0.03;
  params.window_width = 10;
  params.window_height = 10;
  params.max_level = 3;
  params.min_eigen_threshold = 0.2;
  params.max_flow_distance = 50;
  params.max_backward_match_distance = 0.1;
  params.good_features_to_track.max_corners = 100;
  params.good_features_to_track.quality_level = 0.01;
  params.good_features_to_track.min_distance = 20;
  params.good_features_to_track.block_size = 3;
  params.good_features_to_track.use_harris_detector = false;
  params.good_features_to_track.k = 0.04;
  return params;
}

cv::Mat MarkerImage(const int row_spacing, const int col_spacing, int& num_markers_added, const cv::Point2i& offset) {
  cv::Mat image(cv::Mat(cv::Size(640, 480), CV_8UC1, cv::Scalar(255)));
  num_markers_added = AddMarkers(row_spacing, col_spacing, image, offset);
  return image;
}

int AddMarkers(const int row_spacing, const int col_spacing, cv::Mat& image, const cv::Point2i& offset) {
  int num_markers = 0;
  // Don't start at zero so all markers are candidates for matches
  // End before edge to add some buffer to markers
  for (int row = row_spacing; row < image.rows - row_spacing; row += row_spacing) {
    for (int col = col_spacing; col < image.cols - col_spacing; col += col_spacing) {
      cv::drawMarker(image, offset + cv::Point2i(col, row), cv::Scalar(0), cv::MARKER_CROSS);
      ++num_markers;
    }
  }
  return num_markers;
}
}  // namespace vision_common
