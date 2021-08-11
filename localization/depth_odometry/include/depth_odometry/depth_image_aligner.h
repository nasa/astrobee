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
#ifndef DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_
#define DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_

#include <depth_odometry/brisk_image.h>
#include <depth_odometry/depth_image_aligner_params.h>
#include <localization_common/time.h>

#include <boost/optional.hpp>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <utility>

namespace depth_odometry {
class DepthImageAligner {
 public:
  DepthImageAligner(const DepthImageAlignerParams& params);
  boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> ComputeRelativeTransform();
  void AddLatestImage(const cv::Mat& latest_image);
  const std::vector<cv::DMatch>& matches() const { return matches_; }

 private:
  DepthImageAlignerParams params_;
  std::unique_ptr<BriskImage> previous_brisk_image_;
  std::unique_ptr<BriskImage> latest_brisk_image_;
  cv::Ptr<cv::BRISK> brisk_detector_;
  std::unique_ptr<cv::FlannBasedMatcher> flann_matcher_;
  std::vector<cv::DMatch> matches_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_
