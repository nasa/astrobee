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
#include <camera/camera_params.h>
#include <ff_common/init.h>
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <sparse_mapping/ransac.h>
#include <sparse_mapping/tensor.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>
#include <vision_common/utilities.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <mutex>

namespace fs = boost::filesystem;
namespace lc = localization_common;
namespace po = boost::program_options;
namespace sm = sparse_mapping;
namespace vc = vision_common;

boost::optional<vc::FeatureMatches> Matches(const vc::FeatureImage& current_image, const vc::FeatureImage& next_image,
                                            vc::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher) {
  const auto& matches = detector_and_matcher.Match(current_image, next_image);
  if (matches.size() < 5) {
    LogError("Too few matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
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
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED>(match.source_point, &undistorted_source_point);
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED>(match.target_point, &undistorted_target_point);
    source_image_points.col(i) = undistorted_source_point;
    target_image_points.col(i) = undistorted_target_point;
    cv_matches.emplace_back(cv::DMatch(i, i, i, 0));
  }


  std::mutex mutex;
  sm::CIDPairAffineMap affines;
  std::vector<cv::DMatch> inlier_matches;
  sm::BuildMapFindEssentialAndInliers(source_image_points, target_image_points, cv_matches, camera_params, false, 0, 0,
                                      &mutex, &affines, &inlier_matches, false, nullptr);
}

bool RotationOnlyImageSequence(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                               const double rotation_inlier_threshold, const double min_percent_rotation_inliers) {
  std::vector<cv::DMatch> inliers;
  // TODO(rsoussan): is affine3d scaled?
  // TODO(rsoussan): source_T_target or target_T_source??
  const auto source_T_target = EstimateAffine3d(matches, camera_params, inliers);
  // TODO(rsoussan): check rotation inlier percentage! make sure small enough!
  const Eigen::Matrix3d source_R_target = source_T_target.linear();
  const Eigen::Isometry3d target_T_source_rotation_only =
    lc::Isometry3d(Eigen::Vector3d::Zero(), source_R_target.transpose());
  // TODO(rsoussan): undistorted here?
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED>();
  double total_error = 0;
  for (const auto& inlier_match : inliers) {
    const auto& match = matches[inlier_match.imgIdx];
    const Eigen::Vector2d& source_point = match.source_point;
    Eigen::Vector2d undistorted_source_point;
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED>(source_point, &undistorted_source_point);
    const auto source_t_source_point = vc::Backproject(undistorted_source_point, intrinsics, 1.0);
    const Eigen::Vector3d target_t_source_point = target_T_source_rotation_only*source_t_source_point;
    const Eigen::Vector2d projected_source_point = vc::Project(target_t_source_point, intrinsics);
    const double error = (projected_source_point - match.target_point).norm();
    total_error += error;
  }
  const double mean_error = total_error/static_cast<double>(inliers.size());
  return mean_error < rotation_inlier_threshold;
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

int RemoveRotationOnlyImages(const std::vector<std::string>& image_names, const camera::CameraParameters& camera_params,
                             const double rotation_inlier_threshold, const double min_percent_rotation_inliers) {
  const vc::LKOpticalFlowFeatureDetectorAndMatcherParams params = LoadParams();
  vc::LKOpticalFlowFeatureDetectorAndMatcher detector_and_matcher(params);
  auto& detector = *(detector_and_matcher.detector());
  // Compare current image with subsequent image and mark subsequent image for removal if it displays rotation only
  // movement. Repeat and create image directories around rotation only sequences.
  int current_image_index = 0;
  int next_image_index = 1;
  auto current_image = LoadImage(current_image_index, image_names, detector);
  auto next_image = LoadImage(next_image_index, image_names, detector);
  int num_removed_images = 0;
  while (current_image_index < image_names.size()) {
    bool removed_rotation_sequence = false;
    while (next_image_index < image_names.size()) {
      const auto matches = Matches(current_image, next_image, detector_and_matcher);
      if (matches &&
          RotationOnlyImageSequence(*matches, camera_params, rotation_inlier_threshold, min_percent_rotation_inliers)) {
        LogDebug("Removing image index: " << next_image_index << ", current image index: " << current_image_index);
        std::remove((image_names[next_image_index]).c_str());
        ++num_removed_images;
        current_image = next_image;
        current_image_index = next_image_index;
        // Don't load next image if index is past the end of the sequence
        if (++next_image_index >= image_names.size()) break;
        next_image = LoadImage(next_image_index, image_names, detector);
        removed_rotation_sequence = true;
      } else {
        break;
      }
    }
    current_image = next_image;
    current_image_index = next_image_index;
    // Exit if current image is the last image in the sequence
    if (current_image_index >= image_names.size() - 1) break;
    next_image = LoadImage(++next_image_index, image_names, detector);
  }

  return num_removed_images;
}

std::vector<std::string> GetImageNames(const std::string& image_directory,
                                       const std::string& image_extension = ".jpg") {
  std::vector<std::string> image_names;
  for (const auto& file : fs::recursive_directory_iterator(image_directory)) {
    if (fs::is_regular_file(file) && file.path().extension() == image_extension)
      image_names.emplace_back(fs::absolute(file.path()).string());
  }
  std::sort(image_names.begin(), image_names.end());
  LogInfo("Found " << image_names.size() << " images.");
  return image_names;
}

int main(int argc, char** argv) {
  double max_low_movement_mean_distance;
  double rotation_inlier_threshold;
  double min_percent_rotation_inliers;
  std::string robot_config_file;
  po::options_description desc(
    "Removes any rotation only image sequences. Splits images into subdirectories when a sequence is removed.");
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "--rotation-inlier-threshold,t", po::value<double>(&rotation_inlier_threshold)->default_value(0.01),
    "Threshold for a point to be an inlier for the RANSAC rotation estimate. Computed using the dot "
    "product of the rotated point and matching point, so perfect alignment yields a value of 0 and "
    "the worst aligment yields a value of 1."
    "sequential images to be classified as a rotation only pair.")(
    "--min-rotation-only-inliers-percent,p", po::value<double>(&min_percent_rotation_inliers)->default_value(0.95),
    "Min percent of matches that are inliers to rotation only movement between "
    "sequential images to be classified as a rotation only pair.")("config-path,c",
                                                                   po::value<std::string>()->required(), "Config path")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "robot config file");
  po::positional_options_description p;
  p.add("image-directory", 1);
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    if (vm.count("help") || (argc <= 1)) {
      std::cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  const std::string image_directory = vm["image-directory"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);
  lc::SetEnvironmentConfigs(config_path, "iss", robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): Allow for other cameras?
  const camera::CameraParameters camera_parameters(&config, "nav_cam");

  if (!fs::exists(image_directory) || !fs::is_directory(image_directory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const auto image_names = GetImageNames(image_directory);
  if (image_names.empty()) LogFatal("No images found.");

  const int num_original_images = image_names.size();
  const int num_removed_images =
    RemoveRotationOnlyImages(image_names, camera_parameters, rotation_inlier_threshold, min_percent_rotation_inliers);
  LogInfo("Removed " << num_removed_images << " of " << num_original_images << " images.");
}
