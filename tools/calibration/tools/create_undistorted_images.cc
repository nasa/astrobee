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
#include <ff_common/utils.h>
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <optimization_common/fov_distorter.h>
#include <optimization_common/rad_distorter.h>
#include <optimization_common/radtan_distorter.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/imgcodecs.hpp>

namespace lc = localization_common;
namespace oc = optimization_common;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  std::string output_undistorted_images_directory;
  po::options_description desc(
    "Creates undistorted images using the intrinsics and distortion params of the selected camera.");
  desc.add_options()("help", "produce help message")("distorted-images-directory", po::value<std::string>()->required(),
                                                     "Distorted images directory")(
    "distortion-type", po::value<std::string>()->required(), "fov, rad, or radtan.")(
    "camera-name", po::value<std::string>()->required(), "Selected camera name from robot config file.")(
    "config-path", po::value<std::string>()->required(), "Config path.")(
    "output-directory,o",
    po::value<std::string>(&output_undistorted_images_directory)->default_value("undistorted_images"),
    "Output directory for the undistorted images.")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name.");
  po::positional_options_description p;
  p.add("distorted-images-directory", 1);
  p.add("distortion-type", 1);
  p.add("camera-name", 1);
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  const std::string distorted_images_directory = vm["distorted-images-directory"].as<std::string>();
  const std::string distortion_type = vm["distortion-type"].as<std::string>();
  const std::string camera_name = vm["camera-name"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  config.AddFile("cameras.config");
  config.AddFile("tools/camera_target_based_intrinsics_calibrator.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  if (!fs::is_directory(distorted_images_directory)) {
    LogFatal("Distorted images directory " << distorted_images_directory << " not found.");
  }
  if (fs::is_directory(output_undistorted_images_directory)) {
    LogFatal("Output undistorted images directory " << output_undistorted_images_directory << " already exists.");
  }
  fs::create_directory(output_undistorted_images_directory);

  const camera::CameraParameters camera_params(&config, camera_name.c_str());
  std::vector<cv::String> image_file_names;
  const std::string images_file_path = (fs::path(distorted_images_directory) / fs::path("*.jpg")).string();
  cv::glob(images_file_path, image_file_names, false);
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::DISTORTED>();
  const Eigen::VectorXd distortion_params = camera_params.GetDistortion();
  for (const auto& image_file_name : image_file_names) {
    const cv::Mat distorted_image = cv::imread(image_file_name);
    cv::Mat undistorted_image;
    // TODO(rsoussan): Avoid checking distortion param value/recreating distortion object each time
    if (distortion_type == "fov") {
      oc::FovDistorter distorter;
      undistorted_image = distorter.Undistort(distorted_image, intrinsics, distortion_params);
    } else if (distortion_type == "rad") {
      oc::RadDistorter distorter;
      undistorted_image = distorter.Undistort(distorted_image, intrinsics, distortion_params);
    } else if (distortion_type == "radtan") {
      oc::RadTanDistorter distorter;
      undistorted_image = distorter.Undistort(distorted_image, intrinsics, distortion_params);
    } else {
      LogFatal("Invalid distortion type provided.");
    }
    const fs::path image_filepath(image_file_name);
    const std::string undistorted_image_filename =
      (fs::path(output_undistorted_images_directory) / image_filepath.filename()).string();
    cv::imwrite(undistorted_image_filename, undistorted_image);
  }
}
