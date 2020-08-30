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
#include <graph_bag/graph_bag.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <glog/logging.h>

// #include <gperftools/profiler.h>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  // TODO(rsoussan): pass this as command line argument
  // ProfilerStart("/home/rsoussan/graph_bag_tests/prof.txt");
  std::string image_topic;
  std::string output_bagfile;
  po::options_description desc("Runs graph localization on a bagfile and saves the results to a new bagfile.");
  desc.add_options()("help", "produce help message")("bagfile", po::value<std::string>()->required(), "Input bagfile")(
      "map-file", po::value<std::string>()->required(), "Map file")(
      "image-topic,i", po::value<std::string>(&image_topic)->default_value("mgt/img_sampler/nav_cam/image_record"),
      "Image topic")("output-bagfile,o", po::value<std::string>(&output_bagfile)->default_value("results.bag"),
                     "Output bagfile")("verbosity,v", po::value<int>(&FLAGS_v)->default_value(0),
                                       "Set verbose logging level, defaults to 0");

  po::positional_options_description p;
  p.add("bagfile", 1);
  p.add("map-file", 1);
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

  const std::string input_bag = vm["bagfile"].as<std::string>();
  const std::string map_file = vm["map-file"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  // TODO(rsoussan): better way to call this function?
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bag)) {
    LOG(FATAL) << "Bagfile " << input_bag << " not found.";
  }

  if (!boost::filesystem::exists(map_file)) {
    LOG(FATAL) << "Map file " << map_file << " not found.";
  }

  if (vm["output-bagfile"].defaulted()) {
    const auto current_path = boost::filesystem::current_path();
    boost::filesystem::path output_bagfile_path(output_bagfile);
    const auto output_bagfile_full_path = current_path / output_bagfile_path;
    output_bagfile = output_bagfile_full_path.string();
  }

  // Set environment configs
  // TODO(rsoussan): make these command line args
  const std::string astrobee_configs_path = "/home/rsoussan/astrobee/astrobee";
  const std::string world = "iss";
  lc::SetEnvironmentConfigs(astrobee_configs_path, world);
  config_reader::ConfigReader config;

  graph_bag::GraphBag graph_bag(input_bag, map_file, image_topic, output_bagfile);
  graph_bag.Run();
  // ProfilerFlush();
  // ProfilerStop();
}
