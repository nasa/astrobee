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
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <fstream>

// Extract images from a ROS bag.

DEFINE_string(image_topic, "/hw/cam_nav",
              "Image topic that you want to write to disk.");
DEFINE_string(output_directory, "",
              "Directory for writing the output imagery.");
DEFINE_string(output_format, "%06i",
              "Format string for writing the output data.");
DEFINE_double(start, 0,
             "Start extracting this many seconds into the bag.");
DEFINE_double(duration, 1e+100,
             "Extract this many seconds from the bag. Default: extract the full bag.");
DEFINE_bool(use_timestamp_as_image_name, false,
             "Let the acquisition timestamp (in seconds since Epoch) be the output image name.");

void form_filename(int seq, double timestamp,
                   char* filename_buffer, int buffer_len) {
  if (!FLAGS_use_timestamp_as_image_name)
    snprintf(filename_buffer, buffer_len,
             FLAGS_output_format.c_str(), seq);
  else
    snprintf(filename_buffer, buffer_len,
             "%10.7f", timestamp);
}

int main(int argc, char ** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " bag.bag\n";
    exit(0);
  }

  char filename_buffer[1024];

  rosbag::Bag bag;
  std::cout << "Opening bag file " << argv[1] << ".\n";
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(FLAGS_image_topic);
  std::cout << "Looking for topic: " << FLAGS_image_topic << "\n";

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

  double beg_time = -1, curr_time = -1;
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::cout << "Copying at most " << view.size() << " frames from the bag file.\n";
  for (rosbag::MessageInstance const m : view) {
    // Extract a regular image
    sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      ros::Time stamp = image_msg->header.stamp;
      curr_time = stamp.toSec();

      // Set up the output filename
      form_filename(image_msg->header.seq, curr_time,
                    filename_buffer, sizeof(filename_buffer));
      std::string name(filename_buffer);
      name = output_directory + "/" + name + ".jpg";

      if (beg_time < 0)
        beg_time = curr_time;
      if (curr_time - beg_time < FLAGS_start ||
          curr_time - beg_time > FLAGS_start + FLAGS_duration) {
        continue;
      }

      // Convert to an opencv image
      cv::Mat image;
      try {
        // Note that the assignment below is shallow, the image
        // will be valid only for as long as image_msg is valid,
        // which can result in issues in other situations.
        image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      } catch (cv_bridge::Exception const& e) {
        try {
          // Note the same comment as earlier.
          image = cv_bridge::toCvShare(image_msg, "32FC1")->image;
        } catch (cv_bridge::Exception const& e) {
          LOG(ERROR) << "Unable to convert " << image_msg->encoding.c_str()
                     << " image to bgr8 or 32FC1";
          continue;
        }
      }

      std::cout << "Writing: " << name << "\n";
      cv::imwrite(name, image);
    }

    // Extract a compressed image
    sensor_msgs::CompressedImage::ConstPtr comp_image_msg
      = m.instantiate<sensor_msgs::CompressedImage>();
    if (comp_image_msg) {
      ros::Time stamp = comp_image_msg->header.stamp;
      curr_time = stamp.toSec();

      // Set up the output filename
      form_filename(comp_image_msg->header.seq, curr_time,
                    filename_buffer, sizeof(filename_buffer));
      std::string name(filename_buffer);
      name = output_directory + "/" + name + ".jpg";

      if (beg_time < 0)
        beg_time = curr_time;
      if (curr_time - beg_time < FLAGS_start ||
          curr_time - beg_time > FLAGS_start + FLAGS_duration) {
        continue;
      }

      cv::Mat image;
      try {
        // convert compressed image data to cv::Mat
        image = cv::imdecode(cv::Mat(comp_image_msg->data), cv::IMREAD_COLOR);
      } catch (cv_bridge::Exception const& e) {
        LOG(ERROR) << "Unable to convert compressed image to bgr8.";
        continue;
      }
      std::cout << "Writing: " << name << "\n";
      cv::imwrite(name, image);
    }
  }

  bag.close();
}
