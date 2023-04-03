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

#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>
#include <ff_msgs/msg/compressed_file.hpp>
#include <ff_msgs/msg/compressed_file_ack.hpp>
#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/command_stamped.hpp>
#include <ff_util/ff_timer.h>

#include <boost/filesystem.hpp>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <gflags/gflags.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace fs = boost::filesystem;
namespace io = boost::iostreams;

FF_DEFINE_LOGGER("data_to_disk_pub")

DEFINE_string(compression, "none",
              "Type of compression [none, deflate, gzip]");

constexpr uintmax_t kMaxSize = 128 * 1024;
rclcpp::Time data_pub_time;
Publisher<ff_msgs::msg::CommandStamped> command_pub;
Publisher<ff_msgs::msg::CompressedFile> data_to_disk_pub;
ff_util::FreeFlyerTimer data_sub_connected_timer;
ff_msgs::msg::CompressedFile cf;

bool ValidateCompression(const char* name, std::string const &value) {
  if (value == "none" || value == "gzip" || value == "deflate")
    return true;

  std::cerr << "Compression must be one of: nono, deflate, or gzip"
            << std::endl;
  return false;
}

void on_connect() {  // NOLINT
  FF_INFO("subscriber present: sending data to disk");
  data_to_disk_pub->publish(cf);
}

void on_cf_ack(ff_msgs::msg::CompressedFileAck::SharedPtr const cf_ack) {
  FF_INFO("ack received: sending set data to disk command");
  // compressed file ack is latched so we need to check the timestamp to make
  // sure this plan is being acked
  if (data_pub_time < cf_ack->header.stamp) {
    ff_msgs::msg::CommandStamped cmd;
    cmd.cmd_name = ff_msgs::msg::CommandConstants::CMD_NAME_SET_DATA_TO_DISK;
    cmd.subsys_name = "Astrobee";
    command_pub->publish(cmd);
    ros::shutdown();
  }
}

void TimerCallback() {
  if (data_to_disk_pub->get_subscription_count() > 0) {
    on_connect();
    data_sub_connected_timer.stop();
  }
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  rclcpp::init(argc, argv);

  if (!google::RegisterFlagValidator(&FLAGS_compression, &ValidateCompression)) {
    std::cerr << "Failed to register compression flag validator." << std::endl;
    return -1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc <= 1) {
    std::cerr << "error: must provide at least one file to send as zones"
              << std::endl;
    return -1;
  }

  cf.header.frame_id = "world";

  // Load the plan
  io::file_descriptor_source source(argv[1], std::ios::in | std::ios::binary);
  std::vector<char> data;
  data.reserve(kMaxSize);

  io::filtering_ostream out;
  if (FLAGS_compression == "deflate") {
    cf.type = ff_msgs::msg::CompressedFile::TYPE_DEFLATE;
    out.push(io::zlib_compressor());
  } else if (FLAGS_compression == "gzip") {
    cf.type = ff_msgs::msg::CompressedFile::TYPE_GZ;
    out.push(io::gzip_compressor());
  } else {
    cf.type = ff_msgs::msg::CompressedFile::TYPE_NONE;
  }
  out.push(io::back_inserter(data));

  io::copy(source, out);
  if (data.size() >= kMaxSize) {
    std::cerr << "error: even after using a '" << FLAGS_compression
              << "' compression, the file is too big." << std::endl
              << "[file must be less that 128k in size, after compression]"
              << std::endl;
    return -1;
  }

  // Copy the data into a CompressedFile (because char and 'unsigned' char)
  // are totally different.
  // NOTE: I also tried setting the msg to 'int8', but ROS made it 'signed',
  // which was just as unhelpful. Thus, copying.
  cf.file.reserve(data.size());
  for (char c : data) {
    cf.file.push_back(static_cast<unsigned char>(c));
  }

  NodeHandle nh;
  data_pub_time = nh->get_clock()->now();

  data_sub_connected_timer.createTimer(1.0, &TimerCallback, nh);

  // Publishes the data to disk to the executive using subscriber status
  // callbacks
  data_to_disk_pub = FF_CREATE_PUBLISHER(nh,
                                         ff_msgs::msg::CompressedFile,
                                         TOPIC_COMMUNICATIONS_DDS_DATA,
                                         10);

  // After the zones are received, send command to set zones to the executive
  command_pub = FF_CREATE_PUBLISHER(nh,
                                    ff_msgs::msg::CommandStamped,
                                    TOPIC_COMMAND,
                                    5);

  // Subscriber that receives confirmation that the zones were received
  Subscriber<ff_msgs::msg::CompressedFileAck> cf_acK_sub =
      FF_CREATE_SUBSCRIBER(nh,
                           ff_msgs::msg::CompressedFileAck,
                           TOPIC_MANAGEMENT_EXEC_CF_ACK,
                           10,
                           std::bind(&on_cf_ack, std::placeholders::_1));

  FF_INFO("waiting for a subscriber...");
  rclcpp::spin(nh);

  return 0;
}
