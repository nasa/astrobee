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

#include <ff_util/ff_names.h>
#include <graph_bag/depth_odometry_adder.h>
#include <graph_bag/utilities.h>
#include <localization_common/utilities.h>

#include <vector>

namespace graph_bag {
namespace lc = localization_common;
DepthOdometryAdder::DepthOdometryAdder(const std::string& input_bag_name, const std::string& output_bag_name,
                                       const bool save_all_topics)
    : input_bag_(input_bag_name, rosbag::bagmode::Read),
      output_bag_(output_bag_name, rosbag::bagmode::Write),
      save_all_topics_(save_all_topics) {}

void DepthOdometryAdder::AddDepthOdometry() {
  const std::string point_cloud_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                        static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                        static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX);
  const std::string image_topic = static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_PREFIX) +
                                  static_cast<std::string>(TOPIC_HARDWARE_NAME_HAZ_CAM) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_EXTENDED) +
                                  static_cast<std::string>(TOPIC_HARDWARE_PICOFLEXX_SUFFIX_AMPLITUDE_IMAGE);
  // Other localization specific topics
  std::vector<std::string> topics;
  topics.push_back(std::string("/") + point_cloud_topic);
  topics.push_back(std::string("/") + image_topic);
  topics.push_back(std::string("/") + TOPIC_SPARSE_MAPPING_POSE);
  topics.push_back("/tf");
  topics.push_back("/tf_static");
  topics.push_back("/loc/ml/features");
  topics.push_back("/loc/of/features");
  topics.push_back("/loc/ar/features");
  topics.push_back("/hw/imu");
  topics.push_back("/mob/flight_mode");
  topics.push_back("/gnc/ekf");
  topics.push_back("/graph_loc/state");

  // Use unique ptr since ternary initialization fails as view construction from another view is private
  std::unique_ptr<rosbag::View> view;
  if (save_all_topics_)
    view.reset(new rosbag::View(input_bag_));
  else
    view.reset(new rosbag::View(input_bag_, rosbag::TopicQuery(topics)));
  for (const rosbag::MessageInstance msg : *view) {
    if (string_ends_with(msg.getTopic(), point_cloud_topic)) {
      const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
      const auto pose_msgs = depth_odometry_wrapper_.PointCloudCallback(point_cloud_msg);
      for (const auto& pose_msg : pose_msgs) {
        const ros::Time timestamp = lc::RosTimeFromHeader(point_cloud_msg->header);
        output_bag_.write(std::string("/") + TOPIC_LOCALIZATION_DEPTH_ODOM, timestamp, pose_msg);
      }
      if (!save_all_topics_) continue;
    } else if (string_ends_with(msg.getTopic(), image_topic)) {
      const sensor_msgs::ImageConstPtr& image_msg = msg.instantiate<sensor_msgs::Image>();
      const auto pose_msgs = depth_odometry_wrapper_.ImageCallback(image_msg);
      for (const auto& pose_msg : pose_msgs) {
        const ros::Time timestamp = lc::RosTimeFromHeader(image_msg->header);
        output_bag_.write(std::string("/") + TOPIC_LOCALIZATION_DEPTH_ODOM, timestamp, pose_msg);
      }
      if (!save_all_topics_) continue;
    }
    output_bag_.write(msg.getTopic(), msg.getTime(), msg);
  }
}
}  // namespace graph_bag
