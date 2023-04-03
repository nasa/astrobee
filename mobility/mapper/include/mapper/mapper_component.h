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

#ifndef MAPPER_MAPPER_COMPONENT_H_
#define MAPPER_MAPPER_COMPONENT_H_

// PCL specific includes
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// Transformation helper code
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ROS libraries
#include <ff_common/ff_ros.h>

#include <visualization_msgs/msg/marker_array.hpp>
namespace visualization_msgs {
typedef msg::Marker Marker;
typedef msg::MarkerArray MarkerArray;
}  // namespace visualization_msgs
#include <std_srvs/srv/trigger.hpp>
namespace std_srvs {
typedef srv::Trigger Trigger;
}  // namespace std_srvs

// Keepout zones for the planner
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range_core.hpp>

// FSW utils
#include <ff_util/ff_component.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_serialization.h>
#include <ff_util/config_server.h>

// Service definition for zone registration
#include <ff_msgs/msg/hazard.hpp>
#include <ff_msgs/srv/get_map.hpp>
namespace ff_msgs {
typedef msg::Hazard Hazard;
typedef srv::GetMap GetMap;
}  // namespace ff_msgs

// Service messages
#include <ff_msgs/srv/set_float.hpp>
#include <ff_msgs/srv/get_float.hpp>
namespace ff_msgs {
typedef srv::SetFloat SetFloat;
typedef srv::GetFloat GetFloat;
}  // namespace ff_msgs

// General messages
#include <std_msgs/msg/empty.hpp>
namespace std_msgs {
typedef msg::Empty Empty;
}  // namespace std_msgs

// C++ libraries
#include <fstream>
#include <vector>
#include <string>
#include <exception>
#include <thread>         // std::thread
#include <mutex>
#include <atomic>

// Astrobee message types
#include "ff_msgs/msg/segment.hpp"
#include "ff_msgs/msg/control_state.hpp"
namespace ff_msgs {
typedef msg::Segment Segment;
typedef msg::ControlState ControlState;
}  // namespace ff_msgs

// Classes
#include "mapper/octoclass.h"
#include "mapper/polynomials.h"
#include "mapper/sampled_trajectory.h"

// Data structures
#include "mapper/structs.h"

// My defined libraries
#include "mapper/visualization_functions.h"

namespace mapper {
FF_DEFINE_LOGGER("mapper");

namespace fs = boost::filesystem;

// Convenience declarations

class MapperComponent : public ff_util::FreeFlyerComponent {
 public:
  explicit MapperComponent(const rclcpp::NodeOptions& options);
  ~MapperComponent();

 protected:
  void Initialize(NodeHandle &nh);

  // Callbacks (see callbacks.cpp for implementation) ----------------
  // Callback for handling incoming point cloud messages
  void PclCallback();

  // Callback for handling incoming new trajectory messages
  void SegmentCallback(const std::shared_ptr<ff_msgs::Segment> msg);

  // Send diagnostics
  void DiagnosticsCallback();

  // Configure callback for updating config file
  bool ReconfigureCallback();

  // Called when the EKF resets
  void ResetCallback(const std::shared_ptr<std_msgs::Empty> msg);

  // Assert a fault - katie's fault code handling will eventually go in here
  void Complete(int32_t response);

  void PreemptCallback(void);

  void CancelCallback(void);

  // Services (see services.cpp for implementation) -----------------
  // Update resolution of the map
  bool SetResolution(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                        std::shared_ptr<ff_msgs::SetFloat::Response> res);
  // Update resolution of the map
  bool GetResolution(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                        std::shared_ptr<ff_msgs::GetFloat::Response> res);

  // Update map memory time
  bool SetMemoryTime(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                        std::shared_ptr<ff_msgs::SetFloat::Response> res);
  // Update map memory time
  bool GetMemoryTime(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                        std::shared_ptr<ff_msgs::GetFloat::Response> res);

  // Update collision distance
  bool SetCollisionDistance(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                    std::shared_ptr<ff_msgs::SetFloat::Response> res);
  // Update collision distance
  bool GetMapInflation(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                    std::shared_ptr<ff_msgs::GetFloat::Response> res);

  // Reset the map
  bool ResetMap(const std::shared_ptr<std_srvs::Trigger::Request> req,
                std::shared_ptr<std_srvs::Trigger::Response> res);

  // Callback to get the free space in the map
  bool GetFreeMapCallback(const std::shared_ptr<ff_msgs::GetMap::Request> req,
                          std::shared_ptr<ff_msgs::GetMap::Response> res);

  // Callback to get the obstacles in the map
  bool GetObstacleMapCallback(const std::shared_ptr<ff_msgs::GetMap::Request> req,
                              std::shared_ptr<ff_msgs::GetMap::Response> res);

  // Threads (see threads.cpp for implementation) -----------------
  // Thread for fading memory of the octomap
  void FadeTask();

  // Collision checking
  void CollisionCheckTask();

  // Timer for getting pcl data and populating the octomap
  void OctomappingTask();

  // Initialize fault management
  void InitFault(std::string const& msg);

 private:
  NodeHandle nh_;
  bool disable_mapper_ = false;
  // Declare global variables (structures defined in structs.h)
  GlobalVariables globals_;

  // Timer variables
  ff_util::FreeFlyerTimer timer_o_;  // Octomapping Task
  ff_util::FreeFlyerTimer timer_d_;  // Diagnostics
  ff_util::FreeFlyerTimer timer_f_;  // Fade Task

  // Subscriber variables
  bool use_haz_cam_, use_perch_cam_;
  rclcpp::Subscription<ff_msgs::Segment>::SharedPtr segment_sub_;
  rclcpp::Subscription<std_msgs::Empty>::SharedPtr reset_sub_;

  // Octomap services
  rclcpp::Service<ff_msgs::SetFloat>::SharedPtr set_resolution_srv_, set_memory_time_srv_, set_collision_distance_srv_;
  rclcpp::Service<ff_msgs::GetFloat>::SharedPtr get_resolution_srv_, get_memory_time_srv_, get_collision_distance_srv_;
  rclcpp::Service<std_srvs::Trigger>::SharedPtr reset_map_srv_;

  // Timer rates (hz)
  double octomap_update_rate_, tf_update_rate_, fading_memory_update_rate_;

  // Trajectory validation variables -----------------------------
  State state_;                                       // State of the mapper (structure defined in struct.h)
  ff_util::Segment segment_;                          // Segment

  // Services
  rclcpp::Service<ff_msgs::GetMap>::SharedPtr get_free_map_srv_;               // Get free map service
  rclcpp::Service<ff_msgs::GetMap>::SharedPtr get_obstacle_map_srv_;           // Set obstacle map service

  ff_util::ConfigServer cfg_;                         // Config server

  // TF2
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;

  // Marker publishers
  rclcpp::Publisher<ff_msgs::Hazard>::SharedPtr hazard_pub_;                                      // Collision warnign
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr obstacle_marker_pub_;             // Obstacle map
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr free_space_marker_pub_;           // Obstacle map
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr obstacle_cloud_pub_;                     // Obstacle map
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr free_space_cloud_pub_;                   // Obstacle map
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr inflated_obstacle_marker_pub_;  // Inflated obstacle map
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr inflated_free_space_marker_pub_;  // Inflated free map
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr inflated_obstacle_cloud_pub_;  // Inflated obstacle map
  rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr inflated_free_space_cloud_pub_;          // Inflated free map
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr cam_frustum_pub_;
};

}  // namespace mapper

#endif  // MAPPER_MAPPER_COMPONENT_H_
