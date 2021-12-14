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

// Standard includes
#include <ros/ros.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/config_server.h>

// For plugin loading
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Config reader
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

// TF2
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>

/**
 * \ingroup mob
 */
namespace mobility {

class FrameStore : public ff_util::FreeFlyerNodelet {
 public:
  FrameStore() : ff_util::FreeFlyerNodelet(NODE_FRAMESTORE, true) {}
  ~FrameStore() {}

 protected:
  void Initialize(ros::NodeHandle *nh) {
    // Set custom config path
    char *path;
    if ((path = getenv("CUSTOM_CONFIG_DIR")) != NULL)
      config_.SetPath(path);
    // Read the config
    config_.AddFile("transforms.config");
    if (!ReadParams())
      return InitFault("Could not read config");
    timer_ = nh->createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&FrameStore::ReadParams, this));},
      false, true);
  }

  bool ReadParams() {
    if (!config_.ReadFiles()) {
      ROS_FATAL("Error loading framestore parameters.");
      return false;
    }
    // Read all transforms
    geometry_msgs::TransformStamped tf;
    Eigen::Vector3d trans;
    Eigen::Quaterniond rot;
    bool global = false;
    std::string parent, child;
    config_reader::ConfigReader::Table table, t_tf, t_rot, t_trans;
    config_reader::ConfigReader::Table group;
    // Get the name of this platform
    std::string platform = GetPlatform();
    // Do the body-frame transforms
    if (config_.GetTable("transforms", &table)) {
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
            // Add the transform
            tf.header.stamp = ros::Time::now();
            // GLobal transforms
            if (global || platform.empty()) {
              tf.header.frame_id = parent;
              tf.child_frame_id = child;
            } else {
              tf.header.frame_id = platform + "/" + parent;
              tf.child_frame_id = platform + "/" + child;
            }
            // Transform conversion
            tf.transform.translation =
              msg_conversions::eigen_to_ros_vector(trans);
            tf.transform.rotation =
              msg_conversions::eigen_to_ros_quat(rot.normalized());
            // Broadcast!
            tf_.sendTransform(tf);
          }
        }
      }
    }
    // All read successfully
    return true;
  }

  // Deal with a fault in a responsible manner
  void InitFault(std::string const& msg ) {
    NODELET_ERROR_STREAM(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg);
    return;
  }

 protected:
  ros::Timer timer_;
  config_reader::ConfigReader config_;
  tf2_ros::StaticTransformBroadcaster tf_;
};

PLUGINLIB_EXPORT_CLASS(mobility::FrameStore, nodelet::Nodelet);

}  // namespace mobility
