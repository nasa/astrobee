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
#include <localization_analysis/map_matcher.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  std::string output_bagfile;
  std::string image_topic;
  std::string robot_config_file;
  std::string world;
  std::string config_path_prefix;
  std::string save_noloc_imgs;
  std::string verbose;
  std::string num_iterations;
  po::options_description desc("Matches images to provided map and saves matches features and poses to a new bag file");
  desc.add_options()("help,h", "produce help message")("bagfile,b", po::value<std::string>()->required(),
                                                       "Input bagfile containing image messages.")(
    "map-file,m", po::value<std::string>()->required(), "Map file")(
    "config-path,c", po::value<std::string>()->required(), "Path to config directory.")(
    "image-topic,i", po::value<std::string>(&image_topic)->default_value("mgt/img_sampler/nav_cam/image_record"),
    "Image topic")("robot-config-file,r",
                   po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
                   "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name")(
    "output-bagfile,o", po::value<std::string>(&output_bagfile)->default_value(""),
    "Output bagfile, defaults to input_bag + _map_matches.bag")(
    "config-path-prefix,p", po::value<std::string>(&config_path_prefix)->default_value(""), "Config path prefix")(
    "save-noloc-imgs,s", po::value<std::string>(&save_noloc_imgs)->default_value("")->implicit_value(""),
    "Save non-localized images to a bag, defaults to input_bag + _nonloc_imgs.bag")(
    "verbose,v", po::value<std::string>(&verbose)->default_value("")->implicit_value(""), "Verbose mode")(
    "num_ransac_iterations,n", po::value<std::string>(&num_iterations)->default_value("1000"),
    "Number of RANSAC iterations");
  po::positional_options_description p;
  p.add("bagfile", 1);
  p.add("map-file", 1);
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
  const std::string map_file = vm["map-file"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bag)) {
    LogFatal("Bagfile " << input_bag << " not found.");
  }

  if (!boost::filesystem::exists(map_file)) {
    LogFatal("Map file " << map_file << " not found.");
  }

  boost::filesystem::path input_bag_path(input_bag);
  if (vm["output-bagfile"].defaulted()) {
    boost::filesystem::path output_bag_path =
      boost::filesystem::current_path() / boost::filesystem::path(input_bag_path.stem().string() + "_map_matches.bag");
    output_bagfile = output_bag_path.string();
  }

  if (!vm["save-noloc-imgs"].defaulted() && save_noloc_imgs.empty()) {
    save_noloc_imgs =
      boost::filesystem::current_path().string() + "/" + input_bag_path.stem().string() + "_nonloc_imgs.bag";
  }

  if (!vm["verbose"].defaulted() && !vm["verbose"].empty()) {
    google::SetCommandLineOption("verbose_localization", "true");
  }

  if (!vm["num_ransac_iterations"].defaulted() && !vm["num_ransac_iterations"].empty()) {
    google::SetCommandLineOption("num_ransac_iterations", num_iterations.data());
  }



  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;
  localization_analysis::MapMatcher map_matcher(input_bag, map_file, image_topic, output_bagfile, config_path_prefix,
                                                save_noloc_imgs);

  map_matcher.AddMapMatches();
  ROS_INFO_STREAM("Using " << input_bag << " on map " << map_file);
  map_matcher.LogResults();
}
