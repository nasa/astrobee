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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <fstream>

// Merge bags
// Usage: merge_bags -output_bag output.bag input1.bag input2.bag ...

DEFINE_string(output_bag, "", "The output bag.");
DEFINE_double(start_time, -1.0, "If non-negative, start at this time, measured in "
              "seconds since epoch.");
DEFINE_double(stop_time, -1.0, "If non-negative, stop before this time, measured in "
              "seconds since epoch.");
DEFINE_string(save_topics, "", "If non-empty, the topics to write. Use quotes and a space "
              "as separator. Example: '/hw/cam_nav /hw/cam_sci'.");
DEFINE_double(min_time_spacing, -1.0,
              "If non-negative, for each input bag, keep only messages for a given topic which differ by no less than "
              "this time amount, in seconds. Assumes that messages in a bag are stored in ascending order of time.");

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

// For images and point cloud messages, the timestamp in the header is more accurate
// than the one in the message, as the latter can be messed up by some tools.
// Try to get the more accurate one.
double headerOrMessageTime(rosbag::MessageInstance const & m) {
  // Message time
  double curr_time = m.getTime().toSec();

  // Check for uncompressed images
  sensor_msgs::Image::ConstPtr image_msg
    = m.instantiate<sensor_msgs::Image>();
  if (image_msg)
    curr_time = image_msg->header.stamp.toSec();

  // Check for compressed images
  sensor_msgs::CompressedImage::ConstPtr comp_image_msg
    = m.instantiate<sensor_msgs::CompressedImage>();
  if (comp_image_msg)
    curr_time = comp_image_msg->header.stamp.toSec();

  // Check for cloud
  sensor_msgs::PointCloud2::ConstPtr curr_pc_msg
    = m.instantiate<sensor_msgs::PointCloud2>();
  if (curr_pc_msg) curr_time = curr_pc_msg->header.stamp.toSec();

  return curr_time;
}

int main(int argc, char ** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_output_bag == "") {
    std::cout << "The output bag was not specified.\n";
    return 1;
  }

  bool time_range_filter = (FLAGS_start_time >= 0 && FLAGS_stop_time >= 0);

  std::set<std::string> save_topics;
  std::istringstream iss(FLAGS_save_topics);
  std::string val;
  while (iss >> val) {
    std::cout << "Will save topic: " << val << std::endl;
    save_topics.insert(val);
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

  // First reorder the bags so that they are merged in the order of
  // advanced time.
  std::cout << "Determining the order of bags in time." << std::endl;
  std::map<double, std::string> time2bag;
  for (int it = 1; it < argc; it++) {
    std::string input_bag_file = argv[it];

    // Get the topics from the input bag
    std::vector<std::string> topics;
    readTopicsInBag(input_bag_file, topics);

    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const m : view) {
      time2bag[m.getTime().toSec()] = input_bag_file;
      break;
    }

    input_bag.close();
  }

  // Open the output bag
  std::cout << "Opening the output bag file: " << FLAGS_output_bag << "\n";
  rosbag::Bag output_bag;
  output_bag.open(FLAGS_output_bag, rosbag::bagmode::Write);

  // Append the input bags chronologically.
  for (auto it = time2bag.begin(); it != time2bag.end(); it++) {
    std::string input_bag_file = it->second;

    std::cout << "Opening the input bag file: " << input_bag_file << "\n";

    // Get the topics from the input bag
    std::vector<std::string> topics;
    readTopicsInBag(input_bag_file, topics);

    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    std::map<std::string, double> last_time_map;

    for (rosbag::MessageInstance const m : view) {
      // Filter by topic, if specified
      if (!save_topics.empty() && save_topics.find(m.getTopic()) == save_topics.end()) continue;

      double curr_time = headerOrMessageTime(m);

      // Filter by time range, if specified
      if (time_range_filter && (curr_time < FLAGS_start_time || curr_time >= FLAGS_stop_time))
        continue;

      // Filter by spacing
      if (FLAGS_min_time_spacing > 0) {
        double last_time = -1.0;
        std::string const& topic = m.getTopic();  // alias
        auto it = last_time_map.find(topic);
        if (it != last_time_map.end()) {
          last_time = it->second;
        }

        if (last_time >= 0.0 && curr_time - last_time < FLAGS_min_time_spacing)
          continue;

        last_time_map[topic] = curr_time;  // last time the message was saved, for next time
      }

      output_bag.write(m.getTopic(), m.getTime(), m);
    }

    input_bag.close();
  }

  output_bag.close();

  return 0;
}
