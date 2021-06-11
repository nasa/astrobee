/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#include <ff_common/init.h>
#include <ff_common/utils.h>

#include <gflags/gflags.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <fstream>

// Merge bags
// Usage: merge_bags -output_bag output.bag input1.bag input2.bag ...

DEFINE_string(output_bag, "",
              "The output bag.");

// Read the list of topics in a bag while avoiding repetitions
void readTopicsInBag(std::string const& bag_file, std::vector<std::string> & topics) {
  topics.clear();

  rosbag::Bag bag(bag_file.c_str());
  rosbag::View view(bag);
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();

  // Read first in a set to deal with any repetitions
  std::set<std::string> topics_set;
  BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
    topics_set.insert(info->topic);
  }

  // Copy the unique names to a vector
  for (auto it = topics_set.begin(); it != topics_set.end() ; it++) {
    topics.push_back(*it);
  }
}

int main(int argc, char ** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_output_bag == "") {
    std::cout << "The output bag was not specified.\n";
    return 1;
  }

  // Make the directory where the output will go
  std::string out_dir = boost::filesystem::path(FLAGS_output_bag).parent_path().string();
  if (out_dir == "")
    out_dir = ".";
  if (!boost::filesystem::exists(out_dir)) {
    int status = mkdir(out_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status && errno != EEXIST) {
      std::cout << "Failed to create directory: " << out_dir << ".";
      return 1;
    }
  }

  // Open the output bag
  std::cout << "Opening the output bag file: " << FLAGS_output_bag << "\n";
  rosbag::Bag output_bag;
  output_bag.open(FLAGS_output_bag, rosbag::bagmode::Write);

  // Append the input bags
  for (int it = 1; it < argc; it++) {
    std::string input_bag_file = argv[it];

    std::cout << "Opening the input bag file: " << input_bag_file << "\n";

    // Get the topics from the input bag
    std::vector<std::string> topics;
    readTopicsInBag(input_bag_file, topics);

    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const m : view) {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }

    input_bag.close();
  }

  output_bag.close();

  return 0;
}
