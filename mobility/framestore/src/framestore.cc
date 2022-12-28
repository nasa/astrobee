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

#include <ff_common/ff_ros.h>
// Standard includes
#include <ff_util/ff_nodelet.h>
// #include <ff_util/config_server.h>

// Config reader
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

// TF2
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.h>
namespace geometry_msgs {
typedef msg::TransformStamped TransformStamped;
}  // namespace geometry_msgs
static const rclcpp::Logger LOGGER = rclcpp::get_logger("framestore");

#include <memory>

/**
 * \ingroup mob
 */
namespace mobility {

class FrameStore : public ff_util::FreeFlyerNodelet {
 public:
  explicit FrameStore(const rclcpp::NodeOptions & options) : ff_util::FreeFlyerNodelet(options) {}
  ~FrameStore() {}

 protected:
  void Initialize(NodeHandle node) {
    tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // Set custom config path
    char *path;
    if ((path = getenv("CUSTOM_CONFIG_DIR")) != NULL) {
      config_.SetPath(path);
    }
    // Read the config
    config_.AddFile("transforms.config");
    if (!ReadParams())
      return InitFault("Could not read config");

    timer_.createTimer(1.0, [this]() {
      config_.CheckFilesUpdated(std::bind(&FrameStore::ReadParams, this));},
      node_, false, true);
  }

  bool ReadParams() {
    if (!config_.ReadFiles()) {
      FF_FATAL("Error loading framestore parameters.");
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
            tf.header.stamp = ROS_TIME_NOW();
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
            tf_->sendTransform(tf);
          }
        }
      }
    }
    // All read successfully
    return true;
  }

  // Deal with a fault in a responsible manner
  void InitFault(std::string const& msg ) {
    FF_ERROR_STREAM(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg);
    return;
  }

 protected:
  NodeHandle ROS_NODE_VAR;
  ff_util::FreeFlyerTimer timer_;
  config_reader::ConfigReader config_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
};

}  // namespace mobility

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mobility::FrameStore)
