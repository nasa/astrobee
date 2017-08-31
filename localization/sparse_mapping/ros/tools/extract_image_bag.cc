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
#include <common/init.h>
#include <common/utils.h>

#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include <string>
#include <vector>

DEFINE_string(image_topic, "/hw/cam_nav",
              "Image topic that you want to write to disk.");
DEFINE_string(output_directory, "",
              "Directory for writing the output imagery.");
DEFINE_string(output_format, "%06i.jpg",
              "Format string for writing the output imagery.");

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    LOG(INFO) << "Usage: " << argv[0] << " bag.bag";
    exit(0);
  }

  char filename_buffer[1024];

  rosbag::Bag bag;
  LOG(INFO) << "Opening bag file " << argv[1] << ".";
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(FLAGS_image_topic);
  LOG(INFO) << "Looking for topic: " << FLAGS_image_topic;

  std::string output_directory = FLAGS_output_directory;
  if (output_directory.empty()) {
    char* temp = strdup(argv[1]);
    std::string base = std::string(basename(temp));
    output_directory = base.substr(0, base.length() - 4) + "_images";
  }

  // Create the output directory, if missing
  int status = mkdir(output_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (status && errno != EEXIST) {
    LOG(ERROR) << "Failed to create directory " << output_directory << ".";
    exit(1);
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  LOG(INFO) << "Copying " << view.size() << " images from bag file.";
  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      // Convert to an opencv image
      cv::Mat image;
      try {
        image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      } catch (cv_bridge::Exception const& e) {
        LOG(ERROR) << "Unable to convert " << image_msg->encoding.c_str() << " image to bgr8";
        continue;
      }
      snprintf(filename_buffer, sizeof(filename_buffer),
               FLAGS_output_format.c_str(), image_msg->header.seq);
      std::string name(filename_buffer);
      name = output_directory + "/" + name;
      cv::imwrite(name, image);
    }
  }

  bag.close();
}
