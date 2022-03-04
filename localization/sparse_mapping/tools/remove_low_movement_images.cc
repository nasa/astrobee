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
#include <ff_common/init.h>
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>

#include <opencv2/imgcodecs.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace lc = localization_common;
namespace po = boost::program_options;
namespace vc = vision_common;

bool LowMovementImageSequence(const vc::FeatureImage& current_image, const vc::FeatureImage& next_image,
                              const double max_low_movement_mean_distance,
                              vc::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher) {
  const auto& matches = detector_and_matcher.Match(current_image, next_image);
  lc::Averager distance_averager;
  for (const auto& match : matches) {
    distance_averager.Update(match.distance);
  }
  if (distance_averager.average() <= max_low_movement_mean_distance) return true;
  return false;
}

vc::LKOpticalFlowFeatureDetectorAndMatcherParams LoadParams() {
  vc::LKOpticalFlowFeatureDetectorAndMatcherParams params;
  // TODO(rsoussan): Add some of these as command line args?
  params.max_iterations = 10;
  params.termination_epsilon = 0.03;
  params.window_length = 10;
  params.max_level = 3;
  params.min_eigen_threshold = 0.2;
  params.max_flow_distance = 50;
  params.max_backward_match_distance = 0.1;
  params.good_features_to_track.max_corners = 100;
  params.good_features_to_track.quality_level = 0.01;
  params.good_features_to_track.min_distance = 10;
  params.good_features_to_track.block_size = 5;
  params.good_features_to_track.use_harris_detector = false;
  params.good_features_to_track.k = 0.04;
  return params;
}

int RemoveLowMovementImages(const std::vector<std::string>& image_names, const double max_low_movement_mean_distance) {
  vc::LKOpticalFlowFeatureDetectorAndMatcherParams params = LoadParams();
  vc::LKOpticalFlowFeatureDetectorAndMatcher detector_and_matcher(params);

  // Compare current image with subsequent image and remove subsequent image if it has low movement.
  // If a subsequent image is removed, check the next image compared with the current image for low movement.
  // If a subsequent image does not have low movement, advance current image and start the process over.
  int current_image_index = 0;
  int next_image_index = 1;
  vc::FeatureImage current_image(cv::imread(image_names[current_image_index], cv::IMREAD_GRAYSCALE),
                                 *(detector_and_matcher.detector()));

  vc::FeatureImage next_image(cv::imread(image_names[next_image_index], cv::IMREAD_GRAYSCALE),
                              *(detector_and_matcher.detector()));
  int num_removed_images = 0;
  while (current_image_index < image_names.size()) {
    while (next_image_index < image_names.size() &&
           LowMovementImageSequence(current_image, next_image, max_low_movement_mean_distance, detector_and_matcher)) {
      std::remove((image_names[next_image_index++]).c_str());
      ++num_removed_images;
      // Don't load next image if index is past the end of the sequence
      if (next_image_index >= image_names.size()) break;
      next_image = vc::FeatureImage(cv::imread(image_names[next_image_index], cv::IMREAD_GRAYSCALE),
                                    *(detector_and_matcher.detector()));
    }
    current_image_index = next_image_index++;
    // Exit if current image is the last image in the sequence
    if (current_image_index >= image_names.size() - 1) break;
    current_image = vc::FeatureImage(cv::imread(image_names[current_image_index], cv::IMREAD_GRAYSCALE),
                                     *(detector_and_matcher.detector()));
    next_image = vc::FeatureImage(cv::imread(image_names[next_image_index], cv::IMREAD_GRAYSCALE),
                                  *(detector_and_matcher.detector()));
  }

  return num_removed_images;
}

std::vector<std::string> GetImageNames(const std::string& image_directory,
                                       const std::string& image_extension = ".jpg") {
  std::vector<std::string> image_names;
  for (const auto& file : boost::filesystem::recursive_directory_iterator(image_directory)) {
    if (boost::filesystem::is_regular_file(file) && file.path().extension() == image_extension)
      image_names.emplace_back(file.path().filename().string());
  }
  LogInfo("Found " << image_names.size() << " images.");
  return image_names;
}

int main(int argc, char** argv) {
  double max_low_movement_mean_distance;
  po::options_description desc(
    "Removes any images with too little movement.  Computes relative movement using sequential images.");
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "--max-low-movement-mean-distance,m", po::value<double>(&max_low_movement_mean_distance)->default_value(1.0),
    "Max mean distance for optical flow tracks between sequential images to be classified as a low movement pair.");
  po::positional_options_description p;
  p.add("image-directory", 1);
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

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(image_directory) || !boost::filesystem::is_directory(image_directory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const auto image_names = GetImageNames(image_directory);
  if (image_names.empty()) LogFatal("No images found.");

  const int num_original_images = image_names.size();
  const int num_removed_images = RemoveLowMovementImages(image_names, max_low_movement_mean_distance);
  LogInfo("Removed " << num_removed_images << " of " << num_original_images << " images.");
}
