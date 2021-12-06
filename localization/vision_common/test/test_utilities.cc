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

#include "test_utilities.h"  // NOLINT

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
  return params;
}

void AddMarkers(const int row_spacing, const int col_spacing, cv::Mat& image, const cv::Point2i& offset) {
  for (int row = 0; row < image.rows; row += row_spacing) {
    for (int col = 0; col < image.cols; col += col_spacing) {
      cv::drawMarker(image, offset + cv::Point2i(row, col), cv::Scalar(255), cv::MARKER_CROSS);
    }
  }
}
}  // namespace vision_common
