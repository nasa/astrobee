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
#ifndef LOCALIZATION_ANALYSIS_FEATURE_TRACK_IMAGE_ADDER_H_
#define LOCALIZATION_ANALYSIS_FEATURE_TRACK_IMAGE_ADDER_H_

#include <camera/camera_params.h>
#include <factor_adders/vo_smart_projection_factor_adder.h>
#include <vision_common/spaced_feature_tracker.h>

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/Image.h>

#include <vector>

namespace localization_analysis {
void FeatureTrackImage(const vision_common::SpacedFeatureTracker& feature_tracker,
                       const camera::CameraParameters& camera_params, cv::Mat& feature_track_image);

void MarkSmartFactorPoints(const std::vector<boost::shared_ptr<const factor_adders::RobustSmartFactor>> smart_factors,
                           const camera::CameraParameters& camera_params, const gtsam::Values& values,
                           cv::Mat& feature_track_image);

boost::optional<sensor_msgs::ImagePtr> CreateFeatureTrackImage(
  const sensor_msgs::ImageConstPtr& image_msg, const vision_common::SpacedFeatureTracker& feature_tracker,
  const camera::CameraParameters& camera_params,
  const std::vector<boost::shared_ptr<const factor_adders::RobustSmartFactor>>& smart_factors,
  const gtsam::Values& values);

cv::Point2f Distort(const Eigen::Vector2d& undistorted_point, const camera::CameraParameters& params);
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_FEATURE_TRACK_IMAGE_ADDER_H_
