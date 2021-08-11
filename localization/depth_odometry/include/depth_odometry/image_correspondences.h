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
#ifndef DEPTH_ODOMETRY_IMAGE_CORRESPONDENCES_H_
#define DEPTH_ODOMETRY_IMAGE_CORRESPONDENCES_H_

#include <localization_common/time.h>
#include <localization_measurements/image_point.h>

#include <opencv2/core/types.hpp>

namespace depth_odometry {
struct ImageCorrespondence {
  ImageCorrespondence(const localization_measurements::ImagePoint& source_point,
                      const localization_measurements::ImagePoint& target_point)
      : source_point(source_point), target_point(target_point) {}
  localization_measurements::ImagePoint source_point;
  localization_measurements::ImagePoint target_point;
};

class ImageCorrespondences {
 public:
  ImageCorrespondences(const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& source_keypoints,
                       const std::vector<cv::KeyPoint>& target_keypoints, const localization_common::Time source_time,
                       const localization_common::Time target_time);
  const localization_common::Time source_time() const { return source_time_; }
  const localization_common::Time target_time() const { return target_time_; }
  const std::vector<ImageCorrespondence>& correspondences() const { return correspondences_; }

 private:
  localization_common::Time source_time_;
  localization_common::Time target_time_;
  std::vector<ImageCorrespondence> correspondences_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_IMAGE_CORRESPONDENCES_H_
