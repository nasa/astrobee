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

// General ROS
#include <ff_common/ff_ros.h>

// Static transform broadcaster
#include <tf2_ros/static_transform_broadcaster.h>

// Config reader
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

namespace geometry_msgs {
typedef msg::TransformStamped TransformStamped;
}  // namespace geometry_msgs

FF_DEFINE_LOGGER("framestore");

// Main entry point of application
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);   \
  auto node = rclcpp::Node::make_shared("global_transforms");
  // Parse the framestore config
  config_reader::ConfigReader cfg;
  cfg.AddFile("transforms.config");
  if (!cfg.ReadFiles()) {
    FF_FATAL("Error loading global transform parameters.");
    return 1;
  }
  // Read all transforms, broadcasting only world -> xxx
  tf2_ros::StaticTransformBroadcaster bc(node);
  geometry_msgs::TransformStamped tf;
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  std::string parent, child;
  bool global = false;
  config_reader::ConfigReader::Table table, t_tf, t_rot, t_trans;
  config_reader::ConfigReader::Table group;
  if (cfg.GetTable("transforms", &table)) {
    for (int i = 0; i < table.GetSize(); i++) {
      if (table.GetTable(i + 1, &group)) {
        if ( group.GetBool("global", &global)
          && group.GetStr("parent", &parent)
          && group.GetStr("child", &child)
          && group.GetTable("transform", &t_tf)
          && t_tf.GetTable("rot", &t_rot)
          && t_tf.GetTable("trans", &t_trans)
          && msg_conversions::config_read_quat(&t_rot, &rot)
          && msg_conversions::config_read_vector(&t_trans, &trans)) {
          // Only send global transforms
          if (global) {
            tf.header.stamp = node->get_clock()->now();
            tf.header.frame_id = parent;
            tf.child_frame_id = child;
            tf.transform.translation =
              msg_conversions::eigen_to_ros_vector(trans);
            tf.transform.rotation =
              msg_conversions::eigen_to_ros_quat(rot.normalized());
            bc.sendTransform(tf);
          }
        }
      }
    }
  }
  rclcpp::spin(node);
  return 0;
}
