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

#include <depth_odometry/calibrator.h>
#include <depth_odometry/depth_matches.h>
#include <ff_common/init.h>
#include <ff_msgs/DepthCorrespondences.h>
#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace po = boost::program_options;
namespace lc = localization_common;
namespace mc = msg_conversions;

// TODO(rsoussan): unify with graph_bag, put this in common location
bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

std::vector<depth_odometry::DepthMatches> LoadMatches(const rosbag::Bag& bag) {
  std::vector<std::string> topics;
  topics.push_back(TOPIC_LOCALIZATION_DEPTH_IMAGE_CORRESPONDENCES);
  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_DEPTH_IMAGE_CORRESPONDENCES);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<depth_odometry::DepthMatches> depth_matches;
  for (const rosbag::MessageInstance msg : view) {
    if (string_ends_with(msg.getTopic(), TOPIC_LOCALIZATION_DEPTH_IMAGE_CORRESPONDENCES)) {
      const ff_msgs::DepthCorrespondencesConstPtr& correspondences_msg =
        msg.instantiate<ff_msgs::DepthCorrespondences>();
      std::vector<Eigen::Vector2d> source_image_points;
      std::vector<Eigen::Vector2d> target_image_points;
      std::vector<Eigen::Vector3d> source_3d_points;
      std::vector<Eigen::Vector3d> target_3d_points;
      for (const auto& correspondence : correspondences_msg->correspondences) {
        source_image_points.emplace_back(mc::Vector2dFromMsg<Eigen::Vector2d>(correspondence.source_image_point));
        target_image_points.emplace_back(mc::Vector2dFromMsg<Eigen::Vector2d>(correspondence.target_image_point));
        source_3d_points.emplace_back(mc::VectorFromMsg<Eigen::Vector3d>(correspondence.source_3d_point));
        target_3d_points.emplace_back(mc::VectorFromMsg<Eigen::Vector3d>(correspondence.target_3d_point));
      }
      const auto source_time = lc::TimeFromRosTime(correspondences_msg->source_time);
      const auto target_time = lc::TimeFromRosTime(correspondences_msg->target_time);
      depth_matches.emplace_back(depth_odometry::DepthMatches(
        source_image_points, target_image_points, source_3d_points, target_3d_points, source_time, target_time));
    }
  }
  return depth_matches;
}

void LoadCalibratorParams(config_reader::ConfigReader& config, depth_odometry::CalibratorParams& params) {
  params.max_num_iterations = mc::LoadInt(config, "max_num_iterations");
  params.max_num_match_sets = mc::LoadInt(config, "max_num_match_sets");
  params.function_tolerance = mc::LoadDouble(config, "function_tolerance");
  params.calibrate_intrinsics = mc::LoadBool(config, "calibrate_intrinsics");
  params.calibrate_depth_image_A_depth_haz = mc::LoadBool(config, "calibrate_depth_image_A_depth_haz");
  params.calibrate_distortion = mc::LoadBool(config, "calibrate_distortion");
  params.camera_params.reset(new camera::CameraParameters(&config, "haz_cam"));
}

int main(int argc, char** argv) {
  std::string robot_config_file;
  std::string world;
  po::options_description desc("Calibrates depth camera parameters using correspondences from a bagfile.");
  desc.add_options()("help", "produce help message")("bagfile", po::value<std::string>()->required(), "Bagfile")(
    "config-path,c", po::value<std::string>()->required(), "Config path")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name");
  po::positional_options_description p;
  p.add("bagfile", 1);
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

  const std::string input_bagfile = vm["bagfile"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bagfile)) {
    LogFatal("Bagfile " << input_bagfile << " not found.");
  }

  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  config.AddFile("cameras.config");
  config.AddFile("localization/depth_odometry_calibrator.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  depth_odometry::CalibratorParams params;
  LoadCalibratorParams(config, params);

  const rosbag::Bag input_bag(input_bagfile, rosbag::bagmode::Read);
  const auto depth_matches = LoadMatches(input_bag);
  LogError("num depth match sets: " << depth_matches.size());
  depth_odometry::Calibrator calibrator(params);
  const Eigen::Affine3d initial_depth_image_A_depth_cloud(Eigen::Affine3d::Identity());
  const Eigen::Matrix3d initial_intrinsics = calibrator.params().camera_params->GetIntrinsicMatrix<camera::DISTORTED>();
  const Eigen::VectorXd distortion = calibrator.params().camera_params->GetDistortion();
  const Eigen::Matrix<double, 4, 1> initial_distortion =
    distortion.size() == 4 ? distortion : Eigen::Matrix<double, 4, 1>::Zero();
  LogError("init depth_A_depth: " << std::endl << initial_depth_image_A_depth_cloud.matrix());
  LogError("init intrinsics: " << std::endl << initial_intrinsics.matrix());
  LogError("init distortion: " << std::endl << initial_distortion.matrix());
  Eigen::Affine3d calibrated_depth_image_A_depth_cloud;
  Eigen::Matrix3d calibrated_intrinsics;
  Eigen::Matrix<double, 4, 1> calibrated_distortion;
  calibrator.Calibrate(depth_matches, initial_depth_image_A_depth_cloud, initial_intrinsics, initial_distortion,
                       calibrated_depth_image_A_depth_cloud, calibrated_intrinsics, calibrated_distortion);
  LogError("calibrated depth_A_depth: " << std::endl << calibrated_depth_image_A_depth_cloud.matrix());
  LogError("calibrated intrinsics: " << std::endl << calibrated_intrinsics.matrix());
  LogError("calibrated distortion: " << std::endl << calibrated_distortion.matrix());
  // TODO: write this to file! also write summary stats? time take? final total error? etc?
}
