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

#include <ros/ros.h>
#include <ff_msgs/CompressedFile.h>
#include <ff_msgs/CompressedFileAck.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_common/ff_names.h>

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

DEFINE_string(compression, "none",
              "Type of compression [none, deflate, gzip]");

constexpr uintmax_t kMaxSize = 128 * 1024;
ros::Time zones_pub_time;
ros::Publisher command_pub;

bool ValidateCompression(const char* name, std::string const &value) {
  if (value == "none" || value == "gzip" || value == "deflate")
    return true;

  std::cerr << "Compression must be one of: nono, deflate, or gzip"
            << std::endl;
  return false;
}

void on_connect(ros::SingleSubscriberPublisher const& sub,
                ff_msgs::CompressedFile &cf) {  // NOLINT
  ROS_INFO("subscriber present: sending zones");
  cf.header.stamp = ros::Time::now();
  sub.publish(cf);
}

void on_cf_ack(ff_msgs::CompressedFileAckConstPtr const& cf_ack) {
  ROS_INFO("ack received: sending zone update command");
  // compressed file ack is latched so we need to check the timestamp to make
  // sure this plan is being acked
  if (zones_pub_time < cf_ack->header.stamp) {
    ff_msgs::CommandStamped cmd;
    cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SET_ZONES;
    cmd.subsys_name = "Astrobee";
    command_pub.publish(cmd);
    ros::shutdown();
  }
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  ros::init(argc, argv, "zone_publisher");

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

  ff_msgs::CompressedFile cf;
  cf.header.seq = 1;
  cf.header.frame_id = "world";

  // Load the plan
  io::file_descriptor_source source(argv[1], std::ios::in | std::ios::binary);
  std::vector<char> data;
  data.reserve(kMaxSize);

  io::filtering_ostream out;
  if (FLAGS_compression == "deflate") {
    cf.type = ff_msgs::CompressedFile::TYPE_DEFLATE;
    out.push(io::zlib_compressor());
  } else if (FLAGS_compression == "gzip") {
    cf.type = ff_msgs::CompressedFile::TYPE_GZ;
    out.push(io::gzip_compressor());
  } else {
    cf.type = ff_msgs::CompressedFile::TYPE_NONE;
  }
  out.push(io::back_inserter(data));

  io::copy(source, out);
  if (data.size() >= kMaxSize) {
    std::cerr << "error: even after using a '" << FLAGS_compression
              << "' compression, the plan is too big." << std::endl
              << "[plans must be less that 128k in size, after compression]"
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

  ros::NodeHandle n;
  zones_pub_time = ros::Time::now();

  // Publishes the zones when to the executive using subscriber status callbacks
  ros::Publisher zone_pub = n.advertise<ff_msgs::CompressedFile>(
                              TOPIC_COMMUNICATIONS_DDS_ZONES, 10,
                              std::bind(&on_connect, std::placeholders::_1, cf));

  // After the zones are received, commands a set zones to the executive
  command_pub = n.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND, 5);

  // Subscriber that receives confirmation that the zones were received
  ros::Subscriber cf_acK_pub = n.subscribe(TOPIC_MANAGEMENT_EXEC_CF_ACK, 10, &on_cf_ack);

  ROS_INFO("waiting for a subscriber...");
  ros::spin();

  return 0;
}
