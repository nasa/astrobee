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

/**
 * \ingroup hw
 */
namespace framestore {

class FrameStore : public ff_util::FreeFlyerNodelet {
 public:
  FrameStore() : ff_util::FreeFlyerNodelet(NODE_FRAMESTORE, true) {}
  virtual ~FrameStore() {}

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
    config_.AddFile("hw/framestore.config");
    ReadParams();
    timer_ = nh->createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&FrameStore::ReadParams, this));},
      false, true);
  }

  void ReadParams() {
    if (!config_.ReadFiles()) {
      ROS_FATAL("Error loading executive parameters.");
      return;
    }
    geometry_msgs::TransformStamped tf;
    Eigen::Vector3d trans;
    Eigen::Quaterniond rot;
    std::string parent, child;
    config_reader::ConfigReader::Table table, t_tf, t_rot, t_trans;
    config_reader::ConfigReader::Table group;
    // Get the name of this platform
    std::string platform = GetPlatform();
    // Do the body-frame transforms
    if (config_.GetTable("transforms", &table)) {
      for (int i = 0; i < table.GetSize(); i++) {
        if (table.GetTable(i + 1, &group)) {
          if ( group.GetStr("parent", &parent)                            // Parent is specified
            && group.GetStr("child", &child)                              // Child frame is specified
            && group.GetTable("transform", &t_tf)                         // Transform table is specified
            && t_tf.GetTable("rot", &t_rot)                               // Rotation table is specified
            && t_tf.GetTable("trans", &t_trans)                           // Translation table is specified
            && msg_conversions::config_read_quat(&t_rot, &rot)            // Rotation
            && msg_conversions::config_read_vector(&t_trans, &trans)) {   // Translation
            // Add the transform
            tf.header.stamp = ros::Time::now();
            tf.header.frame_id = (platform.empty() ? parent : platform + "/" + parent);
            tf.child_frame_id = (platform.empty() ? child : platform + "/" + child);
            tf.transform.translation = msg_conversions::eigen_to_ros_vector(trans);
            tf.transform.rotation = msg_conversions::eigen_to_ros_quat(rot.normalized());
            tf_.sendTransform(tf);
          }
        }
      }
    }
  }

 protected:
  ros::Timer timer_;
  config_reader::ConfigReader config_;
  tf2_ros::StaticTransformBroadcaster tf_;
};

PLUGINLIB_DECLARE_CLASS(framestore, FrameStore,
                        framestore::FrameStore, nodelet::Nodelet);

}  // namespace framestore
