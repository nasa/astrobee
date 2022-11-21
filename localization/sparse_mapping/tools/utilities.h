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
#ifndef LOCALIZATION_SPARSE_MAPPING_TOOLS_UTILITIES_H_
#define LOCALIZATION_SPARSE_MAPPING_TOOLS_UTILITIES_H_

#include <camera/camera_params.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace sparse_mapping {
cv::Point2f CvPoint2(const Eigen::Vector2d& point);

void CreateSubdirectory(const std::string& directory, const std::string& subdirectory);

boost::optional<vision_common::FeatureMatches> Matches(
  const vision_common::FeatureImage& current_image, const vision_common::FeatureImage& next_image,
  vision_common::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher);

Eigen::Affine3d EstimateAffine3d(const vision_common::FeatureMatches& matches,
                                 const camera::CameraParameters& camera_params, std::vector<cv::DMatch>& inliers);

vision_common::FeatureImage LoadImage(const int index, const std::vector<std::string>& image_names,
                                      cv::Feature2D& detector);

vision_common::LKOpticalFlowFeatureDetectorAndMatcherParams LoadParams();

bool LowMovementImageSequence(const vision_common::FeatureMatches& matches,
                              const double max_low_movement_mean_distance);

std::vector<std::string> GetImageNames(const std::string& image_directory, const std::string& image_extension = ".jpg");
}  // namespace sparse_mapping

#endif  // LOCALIZATION_SPARSE_MAPPING_TOOLS_UTILITIES_H_
