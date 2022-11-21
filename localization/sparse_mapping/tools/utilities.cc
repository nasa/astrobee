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
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <sparse_mapping/tensor.h>
#include <vision_common/utilities.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <mutex>

#include "utilities.h"  // NOLINT

namespace fs = boost::filesystem;
namespace lc = localization_common;
namespace sm = sparse_mapping;
namespace vc = vision_common;

namespace sparse_mapping {
cv::Point2f CvPoint2(const Eigen::Vector2d& point) { return cv::Point2f(point.x(), point.y()); }

void CreateSubdirectory(const std::string& directory, const std::string& subdirectory) {
  const auto subdirectory_path = fs::path(directory) / fs::path(subdirectory);
  if (!boost::filesystem::exists(subdirectory_path)) {
    boost::filesystem::create_directories(subdirectory_path);
  }
}

boost::optional<vc::FeatureMatches> Matches(const vc::FeatureImage& current_image, const vc::FeatureImage& next_image,
                                            vc::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher) {
  const auto& matches = detector_and_matcher.Match(current_image, next_image);
  if (matches.size() < 5) {
    LogDebug("Too few matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                                 << ", next image keypoints: " << next_image.keypoints().size());
    return boost::none;
  }
  LogDebug("Found matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                             << ", next image keypoints: " << next_image.keypoints().size());
  return matches;
}

Eigen::Affine3d EstimateAffine3d(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                                 std::vector<cv::DMatch>& inliers) {
  Eigen::Matrix2Xd source_image_points(2, matches.size());
  Eigen::Matrix2Xd target_image_points(2, matches.size());
  std::vector<cv::DMatch> cv_matches;
  for (int i = 0; i < matches.size(); ++i) {
    const auto& match = matches[i];
    Eigen::Vector2d undistorted_source_point;
    Eigen::Vector2d undistorted_target_point;
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(match.source_point, &undistorted_source_point);
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(match.target_point, &undistorted_target_point);
    source_image_points.col(i) = undistorted_source_point;
    target_image_points.col(i) = undistorted_target_point;
    cv_matches.emplace_back(cv::DMatch(i, i, i, 0));
  }

  std::mutex mutex;
  CIDPairAffineMap affines;
  BuildMapFindEssentialAndInliers(source_image_points, target_image_points, cv_matches, camera_params, false, 0, 0,
                                  &mutex, &affines, &inliers, false, nullptr);
  const Eigen::Affine3d target_T_source = affines[std::make_pair(0, 0)];
  return target_T_source.inverse();
}

vc::FeatureImage LoadImage(const int index, const std::vector<std::string>& image_names, cv::Feature2D& detector) {
  auto image = cv::imread(image_names[index], cv::IMREAD_GRAYSCALE);
  if (image.empty()) LogFatal("Failed to load image " << image_names[index]);
  cv::resize(image, image, cv::Size(), 0.5, 0.5);
  // TODO(rsoussan): Add option to undistort image, use histogram equalization
  return vc::FeatureImage(image, detector);
}

vc::LKOpticalFlowFeatureDetectorAndMatcherParams LoadParams() {
  vc::LKOpticalFlowFeatureDetectorAndMatcherParams params;
  // TODO(rsoussan): Add config file for these
  params.max_iterations = 10;
  params.termination_epsilon = 0.03;
  params.window_length = 31;
  params.max_level = 3;
  params.min_eigen_threshold = 0.001;
  params.max_flow_distance = 180;
  params.max_backward_match_distance = 0.5;
  params.good_features_to_track.max_corners = 100;
  params.good_features_to_track.quality_level = 0.01;
  params.good_features_to_track.min_distance = 40;
  params.good_features_to_track.block_size = 3;
  params.good_features_to_track.use_harris_detector = false;
  params.good_features_to_track.k = 0.04;
  return params;
}

bool LowMovementImageSequence(const vision_common::FeatureMatches& matches,
                              const double max_low_movement_mean_distance) {
  if (matches.size() < 5) {
    LogDebug("Too few matches: " << matches.size());
    return false;
  }
  LogDebug("Found matches: " << matches.size());
  lc::Averager distance_averager;
  for (const auto& match : matches) {
    distance_averager.Update(match.distance);
  }
  LogDebug("Mean distance: " << distance_averager.average());
  if (distance_averager.average() <= max_low_movement_mean_distance) return true;
  return false;
}

// Order absolute paths using just the filename
struct filename_ordering {
  inline bool operator()(const std::string& filepath_a, const std::string& filepath_b) {
    const auto path_a = fs::path(filepath_a);
    const std::string file_a = path_a.filename().string();
    const auto path_b = fs::path(filepath_b);
    const std::string file_b = path_b.filename().string();
    return (file_a < file_b);
  }
};

std::vector<std::string> GetImageNames(const std::string& image_directory, const std::string& image_extension) {
  std::vector<std::string> image_names;
  for (const auto& file : fs::recursive_directory_iterator(image_directory)) {
    if (fs::is_regular_file(file) && file.path().extension() == image_extension)
      image_names.emplace_back(fs::absolute(file.path()).string());
  }
  std::sort(image_names.begin(), image_names.end(), filename_ordering());
  LogInfo("Found " << image_names.size() << " images.");
  return image_names;
}
}  // namespace sparse_mapping
