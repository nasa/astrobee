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

// #include <gperftools/profiler.h>

int main(int argc, char **argv) {
  // ProfilerStart("/home/rsoussan/graph_bag_tests/prof.txt");
  // TODO(rsoussan): pass this from the command line using arg parse!
  // const std::string input_bag =
  // "/home/rsoussan/Downloads/nasa_data/bags/20160211_1701_nav_sampled_shortened.bag";
  // const std::string input_bag =
  // "/home/rsoussan/Downloads/nasa_data/bags/20160211_1707_nav_sampled.bag";
  // const std::string input_bag =
  // "/home/rsoussan/Downloads/nasa_data/bags/20160211_1711_nav_sampled.bag";
  //  const std::string input_bag =
  //  "/home/rsoussan/Downloads/nasa_data/bags/20160211_1718_nav_sampled.bag";
  const std::string input_bag =
      "/home/rsoussan/Downloads/nasa_data/docking/"
      "20200702_1853_mobNavSpd_trimmed.bag";
  // const std::string map_file =
  // "/home/rsoussan/Downloads/nasa_data/merged_20190523_0614_0712_0724_0828_1101_20200513_0521.brisk.hist.map";
  const std::string map_file =
      "/home/rsoussan/Downloads/nasa_data/"
      "merged_0523_0614_0712_0724_0828_1101.brisk.hist.map";
  const std::string image_topic = "mgt/img_sampler/nav_cam/image_record";
  const std::string results_bag = "/home/rsoussan/graph_bag_tests/results.bag";
  // TODO(rsoussan): why is this needed?
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  graph_bag::GraphBag graph_bag(input_bag, map_file, image_topic, results_bag);
  graph_bag.Run();
  // ProfilerFlush();
  // ProfilerStop();
}
