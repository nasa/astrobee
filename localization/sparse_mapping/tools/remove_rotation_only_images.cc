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
#include <gtsam/geometry/triangulation.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <mutex>

namespace fs = boost::filesystem;
namespace lc = localization_common;
namespace po = boost::program_options;
namespace sm = sparse_mapping;
namespace vc = vision_common;

// TODO(rsoussan): remove this
cv::Mat kImg;

// TODO(rsoussan): rename this
struct Result {
  Result(const double ratio, const std::string& image_name, const bool removed)
      : ratio(ratio), image_name(image_name), removed(removed) {}
  double ratio;
  std::string image_name;
  bool removed;
};

// TODO(rsoussan): put this in somewhere common
cv::Point2f CvPoint2(const Eigen::Vector2d& point) { return cv::Point2f(point.x(), point.y()); }

void CreateSubdirectory(const std::string& directory, const std::string& subdirectory) {
  const auto subdirectory_path = fs::path(directory) / fs::path(subdirectory);
  if (!boost::filesystem::exists(subdirectory_path)) {
    boost::filesystem::create_directories(subdirectory_path);
  }
}

void Move(const std::string& image_name, const std::string& subdirectory) {
  fs::path image_path(image_name);
  const auto parent_directory = image_path.parent_path();
  const auto subdirectory_parent_directory = parent_directory / fs::path(subdirectory);
  const auto subdirectory_image_path = subdirectory_parent_directory / image_path.filename();
  std::rename((image_name).c_str(), subdirectory_image_path.string().c_str());
}

void RemoveOrMove(const bool move, const std::string& image_name) {
  if (move) {
    Move(image_name, "removed");
  } else {
    std::remove(image_name.c_str());
  }
}

void RemoveOrMove(const bool move, Result& result) {
  RemoveOrMove(move, result.image_name);
  result.removed = true;
}

// TODO(rsoussan): put this in somewhere common
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

// TODO(rsoussan): put this in somewhere common?
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
  sm::CIDPairAffineMap affines;
  sm::BuildMapFindEssentialAndInliers(source_image_points, target_image_points, cv_matches, camera_params, false, 0, 0,
                                      &mutex, &affines, &inliers, false, nullptr);
  const Eigen::Affine3d target_T_source = affines[std::make_pair(0, 0)];
  return target_T_source.inverse();
}

bool RotationOnlyImageSequence(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                               const double max_rotation_error_ratio, const double min_relative_pose_inliers_ratio,
                               const std::string image_name, const bool remove_erroneous_images,
                               const bool move_removed_images, const bool view_images, std::vector<Result>& results) {
  if (matches.size() < 10) {
    std::cout << "Too few matches found between images. Matches: " << matches.size() << std::endl;
    results.emplace_back(Result(0, image_name, remove_erroneous_images));
    if (remove_erroneous_images) {
      std::cout << "Removed erroneous image: " << image_name << std::endl;
    }
    return remove_erroneous_images;
  }
  std::vector<cv::DMatch> inliers;
  const auto source_T_target = EstimateAffine3d(matches, camera_params, inliers);
  const double relative_pose_inliers_ratio = static_cast<double>(inliers.size()) / static_cast<double>(matches.size());
  if (relative_pose_inliers_ratio < min_relative_pose_inliers_ratio) {
    std::cout << "Too few inliers found. Inliers: " << inliers.size() << ", total matches: " << matches.size()
              << ", ratio: " << relative_pose_inliers_ratio << ", threshold: " << min_relative_pose_inliers_ratio
              << std::endl;
    results.emplace_back(Result(0, image_name, remove_erroneous_images));
    if (remove_erroneous_images) {
      std::cout << "Removed erroneous image: " << image_name << std::endl;
    }
    return remove_erroneous_images;
  }
  const Eigen::Matrix3d source_R_target = source_T_target.linear();
  const Eigen::Isometry3d target_T_source_rotation_only =
    lc::Isometry3d(Eigen::Vector3d::Zero(), source_R_target.transpose());
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED>();
  double total_rotation_corrected_error = 0;
  double total_optical_flow_error = 0;
  int good_triangulation_count = 0;
  cv::Mat projection_img;
  if (view_images) cv::cvtColor(kImg.clone(), projection_img, cv::COLOR_GRAY2RGB);
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
      cv::circle(projection_img, CvPoint2(match.target_point), 1, cv::Scalar(255, 255, 255), -1, 8);
      Eigen::Vector2d distorted_projected_source_point;
      camera_params.Convert<camera::UNDISTORTED, camera::DISTORTED>(projected_source_point,
                                                                    &distorted_projected_source_point);

      cv::circle(projection_img, CvPoint2(distorted_projected_source_point), 1, cv::Scalar(255, 255, 255), -1, 8);
      cv::line(projection_img, CvPoint2(distorted_projected_source_point), CvPoint2(match.target_point),
               cv::Scalar(255, 255, 255), 2, 8, 0);
    }
  }

  const double size = static_cast<double>(inliers.size());
  const double mean_rotation_corrected_error = total_rotation_corrected_error / size;
  const double mean_optical_flow_error = total_optical_flow_error / size;
  const double error_ratio = mean_rotation_corrected_error / mean_optical_flow_error;
  const bool remove_image = error_ratio < max_rotation_error_ratio;
  results.emplace_back(Result(error_ratio, image_name, remove_image));
  static int index = 0;
  if (remove_image)
    std::cout << "Remove ";
  else
    std::cout << "Keep ";
  std::cout << "img: " << image_name << ", index: " << index++ << ", num inliers: " << inliers.size()
            << ", error ratio: " << error_ratio << ", mean rot error: " << mean_rotation_corrected_error
            << ", mean of error: " << mean_optical_flow_error << std::endl;
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
    if (remove_image)
      cv::putText(projection_img, "Removing", cv::Point(projection_img.cols / 2 - 100, projection_img.rows - 200),
                  CV_FONT_NORMAL, 2, cv_color, 4, cv::LINE_AA);
    cv::Mat resized_projection_img;
    cv::resize(projection_img, resized_projection_img, cv::Size(projection_img.cols * 2, projection_img.rows * 2));
    const std::string window_name("ratio_image");
    cv::namedWindow(window_name);
    cv::moveWindow(window_name, 0, 0);
    cv::imshow(window_name, resized_projection_img);
    cv::waitKey(0);
  }

  return remove_image;
}

int RemoveRotationSequences(const int max_distance_between_removed_images, const bool move_removed_images,
                            std::vector<Result>& results) {
  // check for largest consecutive chunk in between removed images, keep going until chunk is too large
  int num_removed_images = 0;
  for (int start_index = 0; start_index < static_cast<int>(results.size());) {
    const auto& result = results[start_index];
    // Find end of rotation chunk after reaching the potential start of a chunk
    if (result.removed) {
      boost::optional<int> end_index;
      int current_distance_between_removed_images = 0;
      // Add all images from start to end that have a max distance less than the provided threshold to other rotation
      // images
      for (int query_index = start_index + 1;
           (query_index < static_cast<int>(results.size()) &&
            (current_distance_between_removed_images < max_distance_between_removed_images));
           ++query_index) {
        if (results[query_index].removed) {
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
          if (!result.removed) {
            RemoveOrMove(move_removed_images, result);
            std::cout << "Removed image in rotation sequence. Img: " << result.image_name << ", index: " << i
                      << ", start index: " << start_index << ", end index: " << *end_index << std::endl;
            ++num_removed_images;
          }
        }
        start_index = *end_index + 1;
        continue;
      }
    }
    ++start_index;
  }
  return num_removed_images;
}

// TODO(rsoussan): put this in somwhere common
vc::FeatureImage LoadImage(const int index, const std::vector<std::string>& image_names, cv::Feature2D& detector) {
  auto image = cv::imread(image_names[index], cv::IMREAD_GRAYSCALE);
  if (image.empty()) LogFatal("Failed to load image " << image_names[index]);
  cv::resize(image, image, cv::Size(), 0.5, 0.5);
  // TODO(rsoussan): Add option to undistort image, use histogram equalization
  return vc::FeatureImage(image, detector);
}

// TODO(rsoussan): put this in somewhere common
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

void SaveResultsToSubdirectories(const std::vector<Result>& results) {
  // Save continous sets of non-removed sequences to different subdirectories
  int subdirectory_index = 0;
  bool previous_result_removed = false;
  for (const auto& result : results) {
    if (result.removed) {
      if (previous_result_removed) {
        continue;
      } else {
        ++subdirectory_index;
        previous_result_removed = true;
      }
    } else {
      previous_result_removed = false;
      Move(result.image_name, std::to_string(subdirectory_index));
    }
  }
}

int RemoveRotationOnlyImages(const std::vector<std::string>& image_names, const camera::CameraParameters& camera_params,
                             const double rotation_inlier_threshold, const double min_relative_pose_inliers_ratio,
                             const bool remove_erroneous_images, const bool move_removed_images, const bool view_images,
                             std::vector<Result>& results) {
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
      kImg = next_image.image().clone();
      const auto matches = Matches(current_image, next_image, detector_and_matcher);
      if (matches && RotationOnlyImageSequence(*matches, camera_params, rotation_inlier_threshold,
                                               min_relative_pose_inliers_ratio, image_names[next_image_index],
                                               remove_erroneous_images, move_removed_images, view_images, results)) {
        RemoveOrMove(move_removed_images, results.back());
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

// TODO(rsoussan): put this in somwhere common
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
  double max_rotation_error_ratio;
  double min_relative_pose_inliers_ratio;
  std::string robot_config_file;
  bool remove_sequences;
  int max_seperation_in_sequence;
  bool remove_erroneous_images;
  bool move_removed_images;
  bool view_images;
  bool save_results_to_subdirectories;
  po::options_description desc("Removes any rotation only image sequences.");
  // TODO(rsoussan): Tune rotation sequence distance
  // TODO(rsoussan): Add option to print debug info? just use LogDebug!!!
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "--max-rotation-error-ratio,e", po::value<double>(&max_rotation_error_ratio)->default_value(0.1),
    "Maximum ratio of rotation corrected error to optical flow error for matched points in a sequential set "
    "of images to be considered rotation only movement. The lower the ratio, the more the rotation fully explains "
    "the movement between the images.")(
    "--min-relative-pose-inliers-ratio,p", po::value<double>(&min_relative_pose_inliers_ratio)->default_value(0.7),
    "Minimum ratio of matches that are inliers in the estimated relative pose between images.")(
    "--keep-sequences,k", po::bool_switch(&remove_sequences)->default_value(true),
    "Don't remove images between detected rotations. If enabled, use --max-seperation-in-sequence to set the max "
    "distance from detected "
    "rotation images to classify an image as in a rotation sequence.")(
    "--max-seperation-in-sequence,d", po::value<int>(&max_seperation_in_sequence)->default_value(2),
    "Maximum distance between detected rotations for sequence removal. Only used if --remove-sequences enabled.")(
    "--move-removed-images,m", po::bool_switch(&move_removed_images)->default_value(false),
    "Move removed images to a directory called removed instead of deleting them.")(
    "--remove-erroneous-images,x", po::bool_switch(&remove_erroneous_images)->default_value(false),
    "Remove images with too few relative pose inliers or too few matches between images.")(
    "--view-images,v", po::bool_switch(&view_images)->default_value(false),
    "View images with projected features and error ratios.")(
    "--save-to-subdirectories,s", po::bool_switch(&save_results_to_subdirectories)->default_value(false),
    "Save results to subdirectories, where each continous set of images seperated by a rotation seqeunce is saved to a "
    "different subdirectory.")

    ("config-path,c", po::value<std::string>()->required(), "Config path")(
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

  if (move_removed_images) {
    CreateSubdirectory(image_directory, "removed");
  }

  const int num_original_images = image_names.size();
  std::vector<Result> results;
  LogInfo("Removing rotation only images, max rotation error ratio: " + std::to_string(max_rotation_error_ratio));
  int num_removed_images =
    RemoveRotationOnlyImages(image_names, camera_parameters, max_rotation_error_ratio, min_relative_pose_inliers_ratio,
                             remove_erroneous_images, move_removed_images, view_images, results);
  if (remove_sequences) {
    LogInfo("Removing rotation sequences, max allowed seperation: " + std::to_string(max_seperation_in_sequence));
    num_removed_images += RemoveRotationSequences(max_seperation_in_sequence, move_removed_images, results);
  }
  LogInfo("Removed " << num_removed_images << " of " << num_original_images << " images.");
  if (save_results_to_subdirectories) {
    LogInfo("Saving results to subdirectories.");
    SaveResultsToSubdirectories(results);
  }
}
