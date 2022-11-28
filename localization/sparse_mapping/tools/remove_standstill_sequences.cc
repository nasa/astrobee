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
#include <ff_common/utils.h>
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "utilities.h"  // NOLINT

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace sm = sparse_mapping;
namespace vc = vision_common;

int RemoveStandstillSequences(const std::vector<bool>& standstill_images, const std::vector<std::string>& image_names) {
  int num_standstill_images = 0;
  for (int i = 0; i < image_names.size(); ++i) {
    if (standstill_images[i]) {
      std::remove((image_names[i]).c_str());
      ++num_standstill_images;
    }
  }
  return num_standstill_images;
}

void GroupStandstillSequences(const int max_distance_between_standstill_images, std::vector<bool>& standstill_images) {
  // check for largest consecutive chunk in between standstill images, keep going until chunk is too large
  int num_standstill_images = 0;
  for (int start_index = 0; start_index < static_cast<int>(standstill_images.size());) {
    // Find end of standstill chunk after reaching the potential start of a chunk
    if (standstill_images[start_index]) {
      boost::optional<int> end_index;
      int current_distance_between_standstill_images = 0;
      // Add all images from start to end that have a max distance less than the provided threshold to other standstill
      // images
      for (int query_index = start_index + 1;
           (query_index < static_cast<int>(standstill_images.size()) &&
            (current_distance_between_standstill_images < max_distance_between_standstill_images));
           ++query_index) {
        if (standstill_images[query_index]) {
          end_index = query_index;
          current_distance_between_standstill_images = 0;
        } else {
          ++current_distance_between_standstill_images;
        }
      }
      // Mark all images in between start and end index as standstill if end index found
      if (end_index) {
        for (int i = start_index; i < *end_index; ++i) {
          if (!standstill_images[i]) {
            LogDebug("Standstill image in standstill sequence. Index: " << i << ", start index: " << start_index
                                                                        << ", end index: " << *end_index);
          }
          standstill_images[i] = true;
        }
        start_index = *end_index + 1;
        continue;
      }
    }
    ++start_index;
  }
}

// TODO(rsoussan): move this to utils, update remove low movemnet images
bool LowMovementImagePair(const vc::FeatureImage& current_image, const vc::FeatureImage& next_image,
                          const double max_low_movement_mean_distance,
                          vc::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher) {
  const auto& matches = detector_and_matcher.Match(current_image, next_image);
  if (matches.size() < 5) {
    LogDebug("Too few matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                                 << ", next image keypoints: " << next_image.keypoints().size());
    return false;
  }
  LogDebug("Found matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                             << ", next image keypoints: " << next_image.keypoints().size());
  // TODO(rsoussan): rename this pair
  return sm::LowMovementImageSequence(matches, max_low_movement_mean_distance);
}

std::vector<bool> IdentifyStandstillImages(const std::vector<std::string>& image_names,
                                           const double max_standstill_mean_distance) {
  const vc::LKOpticalFlowFeatureDetectorAndMatcherParams params = sm::LoadParams();
  vc::LKOpticalFlowFeatureDetectorAndMatcher detector_and_matcher(params);
  auto& detector = *(detector_and_matcher.detector());
  // Compare current image with subsequent image and remove subsequent image if it has low movement.
  // If a subsequent image is removed, check the next image compared with the current image for low movement.
  // If a subsequent image does not have low movement, advance current image and start the process over.
  int current_image_index = 0;
  int next_image_index = 1;
  auto current_image = sm::LoadImage(current_image_index, image_names, detector);
  auto next_image = sm::LoadImage(next_image_index, image_names, detector);
  int num_removed_images = 0;
  std::vector<bool> standstill_images(image_names.size(), false);
  while (current_image_index < image_names.size()) {
    ff_common::PrintProgressBar(stdout,
                                static_cast<float>(current_image_index) / static_cast<float>(image_names.size() - 1));
    while (next_image_index < image_names.size() &&
           LowMovementImagePair(current_image, next_image, max_standstill_mean_distance, detector_and_matcher)) {
      LogDebug("Standstill image index: " << next_image_index << ", current image index: " << current_image_index);
      standstill_images[next_image_index++] = true;
      // Don't load next image if index is past the end of the sequence
      if (next_image_index >= image_names.size()) break;
      next_image = sm::LoadImage(next_image_index, image_names, detector);
    }
    current_image = next_image;
    current_image_index = next_image_index;
    // Exit if current image is the last image in the sequence
    if (current_image_index >= image_names.size() - 1) break;
    next_image = sm::LoadImage(++next_image_index, image_names, detector);
  }

  return standstill_images;
}

int main(int argc, char** argv) {
  double max_standstill_mean_distance;
  int max_separation_in_sequence;
  po::options_description desc(
    "Removes standstill image sequences with too little movement.  Computes relative movement using sequential "
    "images.");
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "--max-standstill-mean-distance,m", po::value<double>(&max_standstill_mean_distance)->default_value(0.006),
    "Max mean distance for optical flow tracks between sequential images to be classified as a standstill pair.")(
    "max-separation-in-sequence,d", po::value<int>(&max_separation_in_sequence)->default_value(5),
    "Maximum distance between detected standstill images for sequence removal. "
    "Setting to a larger value marks more images as standstill"
    "in between already detected standstill images.");
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

  if (!fs::exists(image_directory) || !fs::is_directory(image_directory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const auto image_names = sm::GetImageNames(image_directory);
  if (image_names.empty()) LogFatal("No images found.");

  LogInfo("Removing standstill image sequences, max standstill mean distance: " << max_standstill_mean_distance);
  const int num_original_images = image_names.size();
  auto standstill_images = IdentifyStandstillImages(image_names, max_standstill_mean_distance);
  GroupStandstillSequences(max_separation_in_sequence, standstill_images);
  const int num_standstill_images = RemoveStandstillSequences(standstill_images, image_names);
  LogInfo("Removed " << num_standstill_images << " of " << num_original_images << " images.");
}
