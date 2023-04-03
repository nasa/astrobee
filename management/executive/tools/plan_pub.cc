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

#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>
#include <ff_common/init.h>

#include <ff_msgs/msg/command_constants.hpp>
#include <ff_msgs/msg/CommandStamped.hpp>
#include <ff_msgs/msg/compressed_file.hpp>
#include <ff_msgs/msg/compressed_file_ack.hpp>
#include <ff_msgs/msg/plan_status_stamped.hpp>

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

FF_DEFINE_LOGGER("plan_pub")

DEFINE_string(compression, "none",
              "Type of compression [none, deflate, gzip]");

constexpr uintmax_t kMaxSize = 128 * 1024;

rclcpp::Time plan_pub_time;

Publisher<ff_msgs::msg::CommandStamped> command_pub;

ff_msgs::msg::CompressedFile cf;
ff_util::FreeFlyerTimer data_sub_connected_timer;

bool ValidateCompression(const char* name, std::string const &value) {
  if (value == "none" || value == "gzip" || value == "deflate")
    return true;

  std::cerr << "Compression must be one of: nono, deflate, or gzip"
            << std::endl;
  return false;
}

void on_connect() {
  FF_INFO("subscriber present: sending plan");
  data_to_disk_pub->publish(cf);
}

void on_cf_ack(ff_msgs::msg::CompressedFileAck::SharedPtr const cf_ack) {
  FF_INFO("Got compressed file ack!");
  // compressed file ack is latched so we need to check the timestamp to make
  // sure this plan is being acked
  // ROS_WARN_STREAM(plan_pub_time << " : " << cf_ack->header.stamp);
  if (plan_pub_time <= cf_ack->header.stamp) {
    FF_INFO("Compressed file ack is valid! Sending set plan!");
    ff_msgs::msg::CommandStamped cmd;
    cmd.cmd_name = ff_msgs::msg::CommandConstants::CMD_NAME_SET_PLAN;
    cmd.subsys_name = "Astrobee";
    command_pub->publish(cmd);
  }
}

// Plan status is published after the plan is set so send run plan to start plan
void on_plan_status(ff_msgs::msg::PlanStatusStamped::SharedPtr const& ps) {
  FF_INFO("Got plan status!");
  // plan status is latched so we need to check the timestamp to make sure this
  // plan is loaded
  // ROS_WARN_STREAM(plan_pub_time << " : " << ps->header.stamp);
  if (plan_pub_time <= ps->header.stamp) {
    ff_msgs::CommandStamped cmd;
    cmd.cmd_name = ff_msgs::msg::CommandConstants::CMD_NAME_RUN_PLAN;
    cmd.subsys_name = "Astrobee";
    command_pub->publish(cmd);

    FF_INFO_STREAM("received plan status: " << ps->name);
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  rclcpp::init(argc, argv);
  NodeHandle nh;

  ros::Time::waitForValid();

  if (!google::RegisterFlagValidator(&FLAGS_compression, &ValidateCompression)) {
    std::cerr << "Failed to register compression flag validator." << std::endl;
    return -1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc <= 1) {
    std::cerr << "error: must provide at least one file to send as a plan"
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


  plan_pub_time = nh->get_clock()->now();
  std::string sub_topic_plan = TOPIC_COMMUNICATIONS_DDS_PLAN;

  std::string sub_topic_command = TOPIC_COMMAND;
  command_pub = n.advertise<ff_msgs::CommandStamped>(sub_topic_command, 5);

  ros::Subscriber cf_ack_sub = n.subscribe(TOPIC_MANAGEMENT_EXEC_CF_ACK, 10,
                                           &on_cf_ack);

  ros::Subscriber plan_status_sub =
    n.subscribe(TOPIC_MANAGEMENT_EXEC_PLAN_STATUS, 10, &on_plan_status);

  while (cf_ack_sub.getNumPublishers() == 0 ||
         plan_status_sub.getNumPublishers() == 0 ||
         command_pub.getNumSubscribers() == 0) {
    ros::Duration(0.5).sleep();
  }

  ros::Publisher plan_pub =
    n.advertise<ff_msgs::CompressedFile>(sub_topic_plan, 10,
      std::bind(&on_connect, std::placeholders::_1, cf));

  ROS_INFO("waiting for a subscriber...");
  ros::spin();
  return 0;
}
