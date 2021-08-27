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

#include <camera/camera_model.h>
#include <depth_odometry/feature_depth_image_measurement.h>
#include <depth_odometry/feature_detector_and_matcher.h>
#include <depth_odometry/depth_image_aligner_params.h>
#include <depth_odometry/depth_matches.h>
#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>
#include <localization_measurements/depth_image_measurement.h>

#include <boost/optional.hpp>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <utility>

namespace depth_odometry {
class DepthImageAligner {
 public:
  DepthImageAligner(const DepthImageAlignerParams& params);
  boost::optional<localization_common::PoseWithCovariance> ComputeRelativeTransform();
  void AddLatestDepthImage(const localization_measurements::DepthImageMeasurement& latest_depth_image);
  const boost::optional<DepthMatches>& matches() const { return matches_; }

 private:
  bool ValidImagePoint(const Eigen::Vector2d& image_point) const;
  void CorrectLandmarks(const std::vector<Eigen::Vector2d>& observations, std::vector<Eigen::Vector3d>& landmarks);

  DepthImageAlignerParams params_;
  std::unique_ptr<FeatureDepthImageMeasurement> previous_feature_depth_image_;
  std::unique_ptr<FeatureDepthImageMeasurement> latest_feature_depth_image_;
  std::unique_ptr<FeatureDetectorAndMatcher> feature_detector_and_matcher_;
  boost::optional<DepthMatches> matches_;
  cv::Ptr<cv::CLAHE> clahe_;
  camera::CameraModel cam_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_DEPTH_IMAGE_ALIGNER_H_
