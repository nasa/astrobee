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

#include <depth_odometry/image_feature_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;

DepthOdometry::DepthOdometry(const DepthOdometryParams& params) : params_(params) {
  depth_image_aligner_.reset(new DepthImageAligner(params_.depth_image_aligner));
}

boost::optional<lc::PoseWithCovarianceAndMatches> DepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  if (!previous_depth_image_ && !latest_depth_image_) {
    latest_depth_image_ = depth_image;
    return boost::none;
  }
  if (depth_image.timestamp < latest_depth_image_->timestamp) {
    LogWarning("GetDepthImageAlignerRelativeTransform: Out of order measurement received.");
    return boost::none;
  }
  previous_depth_image_ = latest_depth_image_;
  latest_depth_image_ = depth_image;
  const double time_diff = latest_depth_image_->timestamp - previous_depth_image_->timestamp;
  if (time_diff > params_.max_time_diff) {
    LogWarning("GetDepthImageAlignerRelativeTransform: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }

  depth_image_aligner_->AddLatestDepthImage(*latest_depth_image_);
  auto relative_transform = depth_image_aligner_->ComputeRelativeTransform();
  if (!relative_transform) {
    LogError("GetDepthImageAlignerRelativeTransform: Failed to get relative transform.");
    return boost::none;
  }
  return relative_transform;
}
}  // namespace depth_odometry
