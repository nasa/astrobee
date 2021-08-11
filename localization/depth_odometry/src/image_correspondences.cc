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

#include <depth_odometry/image_correspondences.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

ImageCorrespondences::ImageCorrespondences(const std::vector<cv::DMatch>& matches,
                                           const std::vector<cv::KeyPoint>& source_keypoints,
                                           const std::vector<cv::KeyPoint>& target_keypoints,
                                           const lc::Time source_time, const lc::Time target_time)
    : source_time_(source_time), target_time_(target_time) {
  for (const auto& match : matches) {
    const auto& source_point_cv = source_keypoints[match.queryIdx].pt;
    const auto& target_point_cv = target_keypoints[match.trainIdx].pt;
    const lm::ImagePoint source_point(source_point_cv.x, source_point_cv.y);
    const lm::ImagePoint target_point(target_point_cv.x, target_point_cv.y);
    correspondences_.emplace_back(source_point, target_point);
  }
}
}  // namespace depth_odometry
