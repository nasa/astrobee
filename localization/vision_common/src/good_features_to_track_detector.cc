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

#include <vision_common/good_features_to_track_detector.h>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace cv {
GoodFeaturesToTrackDetector::GoodFeaturesToTrackDetector(const vision_common::GoodFeaturesToTrackDetectorParams& params)
    : params_(params) {}
void GoodFeaturesToTrackDetector::detectAndCompute(cv::InputArray image, cv::InputArray mask,
                                                   std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors,
                                                   bool useProvidedKeypoints) {
  std::vector<cv::Point2f> features;
  cv::goodFeaturesToTrack(image, features, params_.max_corners, params_.quality_level, params_.min_distance, mask,
                          params_.block_size, params_.use_harris_detector, params_.k);
  cv::KeyPoint::convert(features, keypoints);
}
}  // namespace cv
