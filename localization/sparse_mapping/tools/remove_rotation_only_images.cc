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
#include <unordered_map>
#include <unordered_set>

#include "utilities.h"  // NOLINT

namespace fs = boost::filesystem;
namespace lc = localization_common;
namespace po = boost::program_options;
namespace sm = sparse_mapping;
namespace vc = vision_common;

enum class ResultType { kValid, kInvalid, kRotation };

struct Result {
  Result(const double ratio, const std::string& image_name, const ResultType& type)
      : ratio(ratio), image_name(image_name), type(type) {}
  static Result InvalidResult(const std::string& image_name) { return Result(0, image_name, ResultType::kInvalid); }
  static Result RotationResult(const double ratio, const std::string& image_name) {
    return Result(ratio, image_name, ResultType::kRotation);
  }
  static Result ValidResult(const double ratio, const std::string& image_name) {
    return Result(ratio, image_name, ResultType::kValid);
  }
  bool Valid() const { return type == ResultType::kValid; }
  bool Rotation() const { return type == ResultType::kRotation; }
  double ratio;
  std::string image_name;
  ResultType type;
  int cluster_id = 0;
  int cluster_start_index = 0;
};

void Move(const std::string& image_name, const std::string& cluster_id, const ResultType type) {
  fs::path image_path(image_name);
  const auto parent_directory_path = image_path.parent_path();
  auto subdirectory_path = parent_directory_path;
  // Save rotation and invalid images in separate subdirectories.
  // Valid images go in parent subdirectory.
  if (type == ResultType::kRotation) {
    subdirectory_path = subdirectory_path / fs::path("rotation");
  } else if (type == ResultType::kInvalid) {
    subdirectory_path = subdirectory_path / fs::path("invalid");
  }

  subdirectory_path = subdirectory_path / fs::path(cluster_id);
  const auto new_image_path = subdirectory_path / image_path.filename();
  std::rename((image_name).c_str(), new_image_path.string().c_str());
}

void CreateSubdirectory(const std::string& directory, const std::string& cluster_directory, const ResultType type) {
  auto parent_directory_path = fs::path(directory);
  // Save rotation and invalid images in separate subdirectories.
  // Valid images go in parent subdirectory.
  if (type == ResultType::kRotation) {
    parent_directory_path = parent_directory_path / fs::path("rotation");
  } else if (type == ResultType::kInvalid) {
    parent_directory_path = parent_directory_path / fs::path("invalid");
  }
  sm::CreateSubdirectory(parent_directory_path.string(), cluster_directory);
}

Result RotationOnlyImage(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                         const double max_rotation_error_ratio, const double min_relative_pose_inliers_ratio,
                         const std::string image_name, const bool view_images, const cv::Mat& image) {
  if (matches.size() < 10) {
    LogDebug("Too few matches found between images. Matches: " << matches.size());
    const auto result = Result::InvalidResult(image_name);
    return result;
  }
  std::vector<cv::DMatch> inliers;
  const auto source_T_target = sm::EstimateAffine3d(matches, camera_params, inliers);
  const double relative_pose_inliers_ratio = static_cast<double>(inliers.size()) / static_cast<double>(matches.size());
  if (relative_pose_inliers_ratio < min_relative_pose_inliers_ratio) {
    LogDebug("Too few inliers found. Inliers: " << inliers.size() << ", total matches: " << matches.size()
                                                << ", ratio: " << relative_pose_inliers_ratio
                                                << ", threshold: " << min_relative_pose_inliers_ratio);
    const auto result = Result::InvalidResult(image_name);
    return result;
  }
  const Eigen::Matrix3d source_R_target = source_T_target.linear();
  const Eigen::Isometry3d target_T_source_rotation_only =
    lc::Isometry3d(Eigen::Vector3d::Zero(), source_R_target.transpose());
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED>();
  double total_rotation_corrected_error = 0;
  double total_optical_flow_error = 0;
  int good_triangulation_count = 0;
  cv::Mat projection_img;
  if (view_images) {
    cv::cvtColor(image.clone(), projection_img, cv::COLOR_GRAY2RGB);
  }
  for (const auto& inlier_match : inliers) {
    const auto& match = matches[inlier_match.imgIdx];
    const Eigen::Vector2d& source_point = match.source_point;
    Eigen::Vector2d undistorted_source_point;
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED>(source_point, &undistorted_source_point);
    const auto source_t_source_point = vc::Backproject(undistorted_source_point, intrinsics, 1.0);
    const Eigen::Vector3d target_t_source_point = target_T_source_rotation_only * source_t_source_point;
    const Eigen::Vector2d projected_source_point = vc::Project(target_t_source_point, intrinsics);
    Eigen::Vector2d distorted_projected_source_point;
    camera_params.Convert<camera::UNDISTORTED, camera::DISTORTED>(projected_source_point,
                                                                  &distorted_projected_source_point);
    // Compute error in distorted space since distorting a point is more accurate than undistorting it
    const double error = (distorted_projected_source_point - match.target_point).norm();
    total_rotation_corrected_error += error;
    total_optical_flow_error += (match.source_point - match.target_point).norm();
    if (view_images) {
      cv::circle(projection_img, sm::CvPoint2(match.target_point), 1, cv::Scalar(255, 255, 255), -1, 8);
      Eigen::Vector2d distorted_projected_source_point;
      camera_params.Convert<camera::UNDISTORTED, camera::DISTORTED>(projected_source_point,
                                                                    &distorted_projected_source_point);

      cv::circle(projection_img, sm::CvPoint2(distorted_projected_source_point), 1, cv::Scalar(255, 255, 255), -1, 8);
      cv::line(projection_img, sm::CvPoint2(distorted_projected_source_point), sm::CvPoint2(match.target_point),
               cv::Scalar(255, 255, 255), 2, 8, 0);
    }
  }

  const double size = static_cast<double>(inliers.size());
  const double mean_rotation_corrected_error = total_rotation_corrected_error / size;
  const double mean_optical_flow_error = total_optical_flow_error / size;
  const double error_ratio = mean_rotation_corrected_error / mean_optical_flow_error;
  const ResultType type = error_ratio < max_rotation_error_ratio ? ResultType::kRotation : ResultType::kValid;
  const Result result(error_ratio, image_name, type);
  static int index = 0;
  std::stringstream ss;
  if (result.Rotation())
    ss << "Remove ";
  else
    ss << "Keep ";
  ss << "img: " << image_name << ", index: " << index++ << ", num inliers: " << inliers.size()
     << ", error ratio: " << error_ratio << ", mean rot error: " << mean_rotation_corrected_error
     << ", mean of error: " << mean_optical_flow_error;
  const auto result_message = ss.str();
  LogDebug(result_message);
  if (view_images) {
    const int color = 40.0 / error_ratio;
    const cv::Mat gray_color(1, 1, CV_8UC1, color);
    cv::Mat heatmap_color;
    cv::applyColorMap(gray_color, heatmap_color, cv::COLORMAP_JET);
    const cv::Scalar cv_color(heatmap_color.data[0], heatmap_color.data[1], heatmap_color.data[2]);
    cv::putText(projection_img,
                "ratio: " + std::to_string(error_ratio) + ", rot: " + std::to_string(mean_rotation_corrected_error) +
                  ", of: " + std::to_string(mean_optical_flow_error),
                cv::Point(projection_img.cols / 2 - 320, projection_img.rows - 20), CV_FONT_NORMAL, 0.9, cv_color, 4,
                cv::LINE_AA);
    if (result.Rotation())
      cv::putText(projection_img, "Removing", cv::Point(projection_img.cols / 2 - 100, projection_img.rows - 200),
                  CV_FONT_NORMAL, 2, cv_color, 4, cv::LINE_AA);
    cv::Mat resized_projection_img;
    cv::resize(projection_img, resized_projection_img, cv::Size(projection_img.cols * 2, projection_img.rows * 2));
    cv::imshow("ratio_image", resized_projection_img);
    cv::waitKey(0);
  }

  return result;
}

void RemoveRotationSequences(const int max_distance_between_removed_images, std::vector<Result>& results) {
  // check for largest consecutive chunk in between removed images, keep going until chunk is too large
  int num_removed_images = 0;
  for (int start_index = 0; start_index < static_cast<int>(results.size());) {
    const auto& result = results[start_index];
    // Find end of non-valid chunk after reaching the potential start of a chunk
    if (!result.Valid()) {
      const auto type = result.type;
      boost::optional<int> end_index;
      int current_distance_between_removed_images = 0;
      // Add all images from start to end that have a max distance less than the provided threshold to other rotation
      // images
      for (int query_index = start_index + 1;
           (query_index < static_cast<int>(results.size()) &&
            (current_distance_between_removed_images < max_distance_between_removed_images));
           ++query_index) {
        if (results[query_index].type == type) {
          end_index = query_index;
          current_distance_between_removed_images = 0;
        } else {
          ++current_distance_between_removed_images;
        }
      }
      // Remove all images in between start and end index if end index found
      if (end_index) {
        for (int i = start_index; i < *end_index; ++i) {
          auto& result = results[i];
          if (result.type != type) {
            LogDebug("Rotation or invalid image in rotation sequence. Img: " << result.image_name << ", index: " << i
                                                                             << ", start index: " << start_index
                                                                             << ", end index: " << *end_index);
          }
          result.type = type;
        }
        start_index = *end_index + 1;
        continue;
      }
    }
    ++start_index;
  }
}

// Cluster results using the start index of each cluster
void ClusterResults(const int min_separation_between_sets, const ResultType& type, std::vector<Result>& results) {
  boost::optional<int> start_index;
  int current_separation = 0;
  for (int i = 0; i < results.size(); ++i) {
    auto& result = results[i];
    if (result.type == type) {
      if (!start_index) start_index = i;
      result.cluster_start_index = *start_index;
      current_separation = 0;
      continue;
    } else if (++current_separation > min_separation_between_sets) {  // NOLINT
      start_index = boost::none;
    }
  }
}

// Assign cluster ids to results ordered using the start index of each cluster
void OrderClusters(std::vector<Result>& results) {
  int cluster_id = -1;
  std::unordered_map<int, int> start_index_to_cluster_id;
  for (auto& result : results) {
    if (start_index_to_cluster_id.count(result.cluster_start_index) == 0) {
      start_index_to_cluster_id.emplace(result.cluster_start_index, ++cluster_id);
    }
    result.cluster_id = start_index_to_cluster_id[result.cluster_start_index];
  }
}

int CountResults(const std::vector<Result>& results, const ResultType type) {
  int count = 0;
  for (const auto& result : results) {
    if (result.type == type) ++count;
  }
  return count;
}

void MarkSmallSequencesInvalid(const int min_sequence_size, std::vector<Result>& results) {
  std::vector<ResultType> types = {ResultType::kValid, ResultType::kRotation};
  for (const auto type : types) {
    int current_cluster_size = 0;
    boost::optional<int> first_cluster_index;
    for (int i = 0; i < results.size(); ++i) {
      const auto& result = results[i];
      if (result.type == type) {
        if (!first_cluster_index) {
          first_cluster_index = result.cluster_start_index;
          ++current_cluster_size;
        } else if (result.cluster_start_index != *first_cluster_index ||
                   i == results.size() - 1) {  // New cluster started or last images, check size of previous cluster
          // Mark cluster as invalid if it is too small
          if (current_cluster_size < min_sequence_size) {
            // Mark up to the end if checking the last images in results, otherwise stop at the start of
            // the new cluster
            const int end_index = (i == results.size() - 1) ? i : i - 1;
            for (int j = *first_cluster_index; j <= end_index; ++j) {
              auto& result = results[j];
              if (result.type == type) {
                result.type = ResultType::kInvalid;
              }
            }
          }
          first_cluster_index = result.cluster_start_index;
          current_cluster_size = 1;
        } else {
          ++current_cluster_size;
        }
      }
    }
  }
}

void MoveResultsToSubdirectories(const std::string& image_directory, const int min_separation_between_sets,
                                 std::vector<Result>& results) {
  ClusterResults(min_separation_between_sets, ResultType::kValid, results);
  ClusterResults(min_separation_between_sets, ResultType::kRotation, results);
  ClusterResults(min_separation_between_sets, ResultType::kInvalid, results);
  OrderClusters(results);
  // TODO(rsoussan): make this a param
  const int min_sequence_size = 5;
  MarkSmallSequencesInvalid(min_sequence_size, results);
  std::unordered_set<int> created_cluster_directories;
  for (const auto& result : results) {
    if (created_cluster_directories.count(result.cluster_id) == 0) {
      CreateSubdirectory(image_directory, std::to_string(result.cluster_id), result.type);
    }
    Move(result.image_name, std::to_string(result.cluster_id), result.type);
  }
}

std::vector<Result> RemoveRotationOnlyImages(const std::vector<std::string>& image_names,
                                             const camera::CameraParameters& camera_params,
                                             const double rotation_inlier_threshold,
                                             const double min_relative_pose_inliers_ratio, const bool view_images) {
  std::vector<Result> results;
  const vc::LKOpticalFlowFeatureDetectorAndMatcherParams params = sm::LoadParams();
  vc::LKOpticalFlowFeatureDetectorAndMatcher detector_and_matcher(params);
  auto& detector = *(detector_and_matcher.detector());
  // Compare current image with subsequent image and mark subsequent image for removal if it displays rotation only
  // movement. Repeat and create image directories around rotation only sequences.
  int current_image_index = 0;
  int next_image_index = 1;
  auto current_image = sm::LoadImage(current_image_index, image_names, detector);
  auto next_image = sm::LoadImage(next_image_index, image_names, detector);
  int num_removed_images = 0;
  if (view_images) {
    const std::string window_name("ratio_image");
    cv::namedWindow(window_name);
    cv::moveWindow(window_name, 50, 30);
  }

  // Manually add first image
  results.emplace_back(Result::ValidResult(0, image_names[0]));
  while (current_image_index < image_names.size()) {
    while (next_image_index < image_names.size()) {
      const auto matches = sm::Matches(current_image, next_image, detector_and_matcher);
      const auto& image_name = image_names[next_image_index];
      if (!matches)
        results.emplace_back(Result::InvalidResult(image_name));
      else
        results.emplace_back(RotationOnlyImage(*matches, camera_params, rotation_inlier_threshold,
                                               min_relative_pose_inliers_ratio, image_name, view_images,
                                               next_image.image()));
      // If valid, try next image
      if (results.back().Valid()) {
        current_image = next_image;
        current_image_index = next_image_index;
        // Don't load next image if index is past the end of the sequence
        if (++next_image_index >= image_names.size()) break;
        next_image = sm::LoadImage(next_image_index, image_names, detector);
      } else {  // Move on to next image and test a new sequence
        break;
      }
    }
    current_image = next_image;
    current_image_index = next_image_index;
    // Exit if current image is the last image in the sequence
    if (current_image_index >= image_names.size() - 1) break;
    next_image = sm::LoadImage(++next_image_index, image_names, detector);
  }

  return results;
}

int main(int argc, char** argv) {
  double max_low_movement_mean_distance;
  double max_rotation_error_ratio;
  double min_relative_pose_inliers_ratio;
  std::string robot_config_file;
  int max_separation_in_sequence;
  bool view_images;
  int min_separation_between_sets;
  po::options_description desc(
    "Removes any rotation-only image sequences. Checks sequential images and removes the subsequent image if it fits a "
    "rotation-only movement model. Further prunes results by removing any images bounded within a provided threhsold "
    "by detected rotations. By default saves results to subdirectories where each subdirectory contains a different "
    "sequence of non-rotation movement, where each sequence is separated by a set of rotation or erroneous images.");
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "max-rotation-error-ratio,e", po::value<double>(&max_rotation_error_ratio)->default_value(0.25),
    "Maximum ratio of rotation corrected error to optical flow error for matched points in a sequential set "
    "of images to be considered rotation only movement. The lower the ratio, the more the rotation must fully explain "
    "the movement between the images and the fewer images will be labeled as rotation only.")(
    "min-relative-pose-inliers-ratio,p", po::value<double>(&min_relative_pose_inliers_ratio)->default_value(0.7),
    "Minimum ratio of matches that are inliers in the estimated relative pose between images. Setting to a lower value "
    "enables less accurate pose estimates to be used to determine if movement is rotation only. Images that do not "
    "have enough matches are labeled erroneous.")(
    "max-separation-in-sequence,d", po::value<int>(&max_separation_in_sequence)->default_value(10),
    "Maximum distance between detected rotations for sequence removal. Setting to a larger value removes more images "
    "in between detected rotations.")(
    "min-separation-between-sets,b", po::value<int>(&min_separation_between_sets)->default_value(10),
    "Minimum separation between non-rotation image sets. Setting to a larger "
    "value enables more movements separated by detected rotations to be combined into the same subdirectories. This is "
    "useful if some of the rotations are short and the movements before and after the rotation should be considered "
    "the same continous movement.")(
    "view-images,v", po::bool_switch(&view_images)->default_value(false),
    "View images with projected features and error ratios for each provided image in the image directory. "
    "Helpful for tuning the error ratio or visualizing rotation movement while the script is running.")(
    "config-path,c", po::value<std::string>()->required(),
    "Full path to astrobee/src/astrobee directory location, e.g. ~/astrobee/src/astrobee.")(
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
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  const camera::CameraParameters camera_parameters(&config, "nav_cam");

  if (!fs::exists(image_directory) || !fs::is_directory(image_directory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const auto image_names = sm::GetImageNames(image_directory);
  if (image_names.empty()) LogFatal("No images found.");

  sm::CreateSubdirectory(image_directory, "rotation");
  sm::CreateSubdirectory(image_directory, "invalid");

  const int num_original_images = image_names.size();
  LogInfo("Removing rotation only images, max rotation error ratio: " + std::to_string(max_rotation_error_ratio));

  auto results = RemoveRotationOnlyImages(image_names, camera_parameters, max_rotation_error_ratio,
                                          min_relative_pose_inliers_ratio, view_images);

  LogInfo("Removing rotation sequences, max allowed separation: " + std::to_string(max_separation_in_sequence));
  RemoveRotationSequences(max_separation_in_sequence, results);
  {
    const int valid_count = CountResults(results, ResultType::kValid);
    const int invalid_count = CountResults(results, ResultType::kInvalid);
    const int rotation_count = CountResults(results, ResultType::kRotation);
    LogInfo("pre small seq Valid images: " << valid_count << " , rotation images: " << rotation_count
                                           << ", invalid images: " << invalid_count);
  }
  LogInfo("Moving results to subdirectories.");
  MoveResultsToSubdirectories(image_directory, min_separation_between_sets, results);
  const int valid_count = CountResults(results, ResultType::kValid);
  const int invalid_count = CountResults(results, ResultType::kInvalid);
  const int rotation_count = CountResults(results, ResultType::kRotation);
  LogInfo("Valid images: " << valid_count << " , rotation images: " << rotation_count
                           << ", invalid images: " << invalid_count);
}
