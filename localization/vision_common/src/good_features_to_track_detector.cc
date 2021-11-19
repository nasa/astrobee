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
void GoodFeaturesToTrackDetector::detectAndCompute(cv::InputArray image, cv::InputArray mask,
                                                   std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors,
                                                   bool useProvidedKeypoints) {
  std::vector<cv::Point2f> features;
  // TODO(rsoussan): make params for these!!!
  cv::goodFeaturesToTrack(image, features, 100, 0.01, 20, mask, 3, false, 0.04);
  cv::KeyPoint::convert(features, keypoints);
}
}  // namespace cv
