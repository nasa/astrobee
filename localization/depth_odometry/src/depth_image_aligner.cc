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
namespace lm = localization_measurements;

DepthImageAligner::DepthImageAligner(const DepthImageAlignerParams& params) : params_(params) {
  brisk_detector_ =
    cv::BRISK::create(params_.brisk_threshold, params_.brisk_octaves, params_.brisk_float_pattern_scale);
  flann_matcher_.reset(new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(
    params_.flann_table_number, params_.flann_key_size, params_.flann_multi_probe_level)));
  clahe_ = cv::createCLAHE(params_.clahe_clip_limit, cv::Size(params_.clahe_grid_length, params_.clahe_grid_length));
  const auto focal_lengths = params_.camera_params->GetFocalVector();
  const auto distortion_params = params_.camera_params->GetDistortion();
  const auto principal_points = params_.camera_params->GetOpticalOffset();
  intrinsics_ = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
  intrinsics_.at<double>(0, 0) = focal_lengths[0];
  intrinsics_.at<double>(1, 1) = focal_lengths[1];
  intrinsics_.at<double>(0, 2) = principal_points[0];
  intrinsics_.at<double>(1, 2) = principal_points[1];
  intrinsics_.at<double>(2, 2) = 1;
  distortion_params_ = cv::Mat(4, 1, cv::DataType<double>::type);
  for (int i = 0; i < 4; ++i) {
    distortion_params_.at<double>(i, 0) = distortion_params[i];
  }
}

boost::optional<std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>>
DepthImageAligner::ComputeRelativeTransform() {
  if (!previous_brisk_depth_image_ || !latest_brisk_depth_image_) return boost::none;
  std::vector<cv::DMatch> matches;
  flann_matcher_->match(previous_brisk_depth_image_->descriptors(), latest_brisk_depth_image_->descriptors(), matches);
  LogError("matches pre filtering: " << matches.size());
  const auto filtered_end = std::remove_if(matches.begin(), matches.end(), [this](const cv::DMatch& match) {
    return match.distance > params_.max_match_hamming_distance;
  });
  matches.erase(filtered_end, matches.end());
  correspondences_ =
    ImageCorrespondences(matches, previous_brisk_depth_image_->keypoints(), latest_brisk_depth_image_->keypoints(),
                         previous_brisk_depth_image_->timestamp, latest_brisk_depth_image_->timestamp);
  LogError("keypoints a: " << previous_brisk_depth_image_->keypoints().size()
                           << ", b: " << latest_brisk_depth_image_->keypoints().size());
  LogError("matches post filtering: " << matches.size());
  return std::pair<Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>{Eigen::Isometry3d::Identity(),
                                                                   Eigen::Matrix<double, 6, 6>::Zero()};
}

void DepthImageAligner::AddLatestDepthImage(const lm::DepthImageMeasurement& latest_depth_image) {
  previous_brisk_depth_image_ = std::move(latest_brisk_depth_image_);
  if (params_.use_clahe) {
    lm::DepthImageMeasurement clahe_depth_image = latest_depth_image;
    clahe_->apply(latest_depth_image.image, clahe_depth_image.image);
    latest_brisk_depth_image_.reset(new BriskDepthImageMeasurement(clahe_depth_image, brisk_detector_));
  } else
    latest_brisk_depth_image_.reset(new BriskDepthImageMeasurement(latest_depth_image, brisk_detector_));
}
}  // namespace depth_odometry
