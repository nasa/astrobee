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
#include <localization_analysis/depth_odometry_adder.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  std::string output_bagfile;
  std::string robot_config_file;
  std::string world;
  bool save_all_topics = false;
  std::string config_path_prefix;
  po::options_description desc("Adds depth odometry relative poses to a new bag file");
  desc.add_options()("help,h", "produce help message")(
    "bagfile", po::value<std::string>()->required(),
    "Input bagfile containing point cloud and image messages for a depth camera.")(
    "config-path,c", po::value<std::string>()->required(), "Path to config directory.")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name")(
    "output-bagfile,o", po::value<std::string>(&output_bagfile)->default_value(""),
    "Output bagfile, defaults to input_bag + with_depth_odometry.bag")(
    "save-all-topics,s", po::bool_switch(&save_all_topics),
    "Save all topics in input bagfile to bagfile with depth odometry. Otherwise just save localization relevant (but "
    "no raw data) topics.")("config-path-prefix,p",
                            po::value<std::string>(&config_path_prefix)->default_value("localization/"),
                            "Config path prefix");
  po::positional_options_description p;
  p.add("bagfile", 1);
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

  const std::string input_bag = vm["bagfile"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bag)) {
    LogFatal("Bagfile " << input_bag << " not found.");
  }

  boost::filesystem::path input_bag_path(input_bag);
  if (vm["output-bagfile"].defaulted()) {
    boost::filesystem::path output_bag_path =
      boost::filesystem::current_path() /
      boost::filesystem::path(input_bag_path.stem().string() + "_with_depth_odometry.bag");
    output_bagfile = output_bag_path.string();
  }
  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }

  localization_analysis::DepthOdometryAdder depth_odometry_adder(input_bag, output_bagfile, save_all_topics,
                                                                 config_path_prefix);
  depth_odometry_adder.AddDepthOdometry();
}
