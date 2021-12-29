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

#include <ff_msgs/PicoflexxIntermediateData.h>
#include <ff_util/ff_names.h>
#include <graph_bag/utilities.h>
#include <localization_common/logger.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace po = boost::program_options;

// TODO(rsoussan): Is this necessary?
struct null_deleter {
  void operator()(void const*) const {}
};

void ConvertBag(const std::string& input_bagfile, const std::string& output_bagfile) {
  const std::string depth_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED);
  const std::string depth_intensity_topic =
    static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) + static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
    static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) + "/amplitude_int";

  rosbag::Bag input_bag(input_bagfile, rosbag::bagmode::Read);
  rosbag::Bag output_bag(output_bagfile, rosbag::bagmode::Write);
  rosbag::View view(input_bag);
  for (const rosbag::MessageInstance msg : view) {
    if (graph_bag::string_ends_with(msg.getTopic(), depth_topic)) {
      const ff_msgs::PicoflexxIntermediateData::ConstPtr& depth_msg =
        msg.instantiate<ff_msgs::PicoflexxIntermediateData>();
      // TODO(rsoussan): Unify this with pico_driver code
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
        sensor_msgs::ImageConstPtr(&depth_msg->raw, null_deleter()), sensor_msgs::image_encodings::TYPE_32FC4);
      std::vector<cv::Mat> layers(4);
      cv::split(cv_ptr->image, layers);
      constexpr double amplitude_factor = 100;
      cv::Mat intenisty_image;
      layers[1].convertTo(intenisty_image, CV_16UC1, amplitude_factor, 0);
      std_msgs::Header header;
      header.stamp = depth_msg->header.stamp;
      header.frame_id = depth_msg->header.frame_id;
      cv_bridge::CvImage intensity_image_msg(header, sensor_msgs::image_encodings::MONO16, intenisty_image);
      output_bag.write(depth_intensity_topic, msg.getTime(), intensity_image_msg.toImageMsg());
    } else {
      output_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
  }
}

int main(int argc, char** argv) {
  po::options_description desc("Converts depth topic in bagfile to intensity image.");
  desc.add_options()("help", "produce help message")("bagfile", po::value<std::string>()->required(), "Bagfile");
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
  boost::filesystem::path input_bag_path(input_bag);
  boost::filesystem::path output_bag_path =
    input_bag_path.parent_path() / boost::filesystem::path(input_bag_path.stem().string() + "_depth_converted.bag");

  ConvertBag(input_bag, output_bag_path.string());
}
