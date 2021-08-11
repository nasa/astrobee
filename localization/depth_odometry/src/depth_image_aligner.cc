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

#include <depth_odometry/depth_image_aligner.h>
#include <localization_common/logger.h>
#include <localization_common/timer.h>

namespace depth_odometry {
namespace lc = localization_common;

DepthImageAligner::DepthImageAligner(const DepthImageAlignerParams& params) : params_(params) {
  brisk_detector_ = cv::BRISK::create();
  flann_matcher_.reset(new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(
    params_.flann_table_number, params_.flann_key_size, params_.flann_multi_probe_level)));
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>>
DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_brisk_image_ || !latest_brisk_image_) return boost::none;
  std::vector<cv::DMatch> matches;
  flann_matcher_->match(previous_brisk_image_->descriptors(), latest_brisk_image_->descriptors(), matches);
  LogError("matches pre filtering: " << matches.size());
  const auto filtered_end = std::remove_if(matches.begin(), matches.end(), [this](const cv::DMatch& match) {
    return match.distance > params_.max_match_hamming_distance;
  });
  matches.erase(filtered_end, matches.end());
  correspondences_.reset(new ImageCorrespondences(matches, previous_brisk_image_->keypoints(),
                                                  latest_brisk_image_->keypoints(), previous_image_time_,
                                                  latest_image_time_));
  LogError("keypoints a: " << previous_brisk_image_->keypoints().size()
                           << ", b: " << latest_brisk_image_->keypoints().size());
  LogError("matches post filtering: " << matches.size());
  return std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>{Eigen::Isometry3d::Identity(),
                                                                   Eigen::Matrix<double, 6, 6>::Zero()};
}

void DepthImageAligner::AddLatestImage(const cv::Mat& latest_image, const lc::Time latest_image_time) {
  previous_brisk_image_ = std::move(latest_brisk_image_);
  latest_brisk_image_.reset(new BriskImage(latest_image, brisk_detector_));
  previous_image_time_ = latest_image_time_;
  latest_image_time_ = latest_image_time;
}
}  // namespace depth_odometry
