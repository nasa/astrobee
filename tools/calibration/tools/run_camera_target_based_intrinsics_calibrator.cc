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

#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/camera_target_based_intrinsics_calibrator.h>
#include <ff_common/init.h>
#include <ff_common/utils.h>
#include <ff_util/ff_names.h>
#include <localization_common/image_correspondences.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace ca = calibration;
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace po = boost::program_options;

lc::ImageCorrespondences LoadTargetMatches(const std::string& match_file) {
  std::vector<Eigen::Vector2d> image_points;
  std::vector<Eigen::Vector3d> points_3d;
  std::cout << "Reading: " << match_file << std::endl;
  std::ifstream cr(match_file.c_str());
  std::string line;
  while (std::getline(cr, line)) {
    double val;
    std::vector<double> vals;
    std::istringstream is(line);
    while (is >> val) vals.push_back(val);
    if (vals.size() != 5) LogFatal("Each line of " << match_file << " must have 5 entries.");
    points_3d.emplace_back(Eigen::Vector3d(vals[1], vals[2], 0));
    image_points.emplace_back(Eigen::Vector2d(vals[3], vals[4]));
  }

  return lc::ImageCorrespondences(image_points, points_3d);
}

// TODO(rsoussan): unify with graph_bag, put this in common location
bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

std::vector<lc::ImageCorrespondences> LoadAllTargetMatches(const std::string& corners_directory,
                                                           const int max_num_match_sets) {
  std::vector<std::string> corners_files;
  std::vector<lc::ImageCorrespondences> all_matches;
  ff_common::ListFiles(corners_directory, "txt", &corners_files);
  int i = 0;
  for (const auto& corners_file : corners_files) {
    const auto& matches = LoadTargetMatches(corners_file);
    if (matches.image_points.size() < 4) {
      LogError("Too few matches, only " << matches.image_points.size() << ".");
      continue;
    }
    all_matches.emplace_back(matches);
    if (++i > max_num_match_sets) break;
  }
  return all_matches;
}

void LoadCalibratorParams(config_reader::ConfigReader& config,
                          ca::CameraTargetBasedIntrinsicsCalibratorParams& params) {
  params.max_num_iterations = mc::LoadInt(config, "max_num_iterations");
  params.max_num_match_sets = mc::LoadInt(config, "max_num_match_sets");
  params.function_tolerance = mc::LoadDouble(config, "function_tolerance");
  params.calibrate_intrinsics = mc::LoadBool(config, "calibrate_intrinsics");
  params.calibrate_distortion = mc::LoadBool(config, "calibrate_distortion");
  const std::string camera = mc::LoadString(config, "camera");
  params.camera_params.reset(new camera::CameraParameters(&config, camera.c_str()));
  params.distortion_type = mc::LoadString(config, "distortion_type");
}

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  std::string corners_directory;
  po::options_description desc("Calibrates camera intrinsics using target detections.");
  desc.add_options()("help", "produce help message")(
    "corners-directory", po::value<std::string>(&corners_directory)->required(), "Corners Directory")(
    "config-path,c", po::value<std::string>()->required(), "Config path")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name");
  po::positional_options_description p;
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

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
  ca::CameraTargetBasedIntrinsicsCalibratorParams params;
  LoadCalibratorParams(config, params);

  std::vector<lc::ImageCorrespondences> target_matches;
  if (!boost::filesystem::is_directory(corners_directory)) {
    LogFatal("Corners directory " << corners_directory << " not found.");
  }
  target_matches = LoadAllTargetMatches(corners_directory, params.max_num_match_sets);
  LogError("num target match sets: " << target_matches.size());
  ca::CameraTargetBasedIntrinsicsCalibrator calibrator(params);
  const Eigen::Matrix3d initial_intrinsics = calibrator.params().camera_params->GetIntrinsicMatrix<camera::DISTORTED>();
  const Eigen::VectorXd initial_distortion = calibrator.params().camera_params->GetDistortion();
  LogError("init intrinsics: " << std::endl << initial_intrinsics.matrix());
  LogError("init distortion: " << std::endl << initial_distortion.matrix());
  Eigen::Matrix3d calibrated_intrinsics;
  Eigen::Matrix<double, 4, 1> calibrated_distortion;
  calibrator.Calibrate(target_matches, *(calibrator.params().camera_params), initial_intrinsics, initial_distortion,
                       calibrated_intrinsics, calibrated_distortion);
  LogError("calibrated intrinsics: " << std::endl << calibrated_intrinsics.matrix());
  LogError("calibrated distortion: " << std::endl << calibrated_distortion.matrix());
  // TODO(rsoussan): write this to file! also write summary stats? time take? final total error? etc?*/
}
