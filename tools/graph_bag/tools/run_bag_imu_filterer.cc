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

#include <graph_bag/bag_imu_filterer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  std::string output_bagfile;
  std::string filter_name;
  po::options_description desc(
    "Reads through a bag file and filters imu measurements, replacing the old imu measurements with new filtered "
    "ones.  Saves output to a new bagfile.");
  desc.add_options()("help", "produce help message")("bagfile", po::value<std::string>()->required(), "Input bagfile")(
    "output-bagfile,o", po::value<std::string>(&output_bagfile)->default_value("filtered_imu.bag"), "Output bagfile")(
    "filter-name,f", po::value<std::string>(&filter_name)->default_value("none"), "Imu filter name");
  po::positional_options_description p;
  p.add("bagfile", 1);
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

  if (!boost::filesystem::exists(input_bag)) {
    LogFatal("Bagfile " << input_bag << " not found.");
  }

  if (vm["output-bagfile"].defaulted()) {
    const auto current_path = boost::filesystem::current_path();
    boost::filesystem::path output_bagfile_path(output_bagfile);
    const auto output_bagfile_full_path = current_path / output_bagfile_path;
    output_bagfile = output_bagfile_full_path.string();
  }

  graph_bag::BagImuFilterer bag_imu_filterer(input_bag, output_bagfile, filter_name);
  bag_imu_filterer.Convert();
}
