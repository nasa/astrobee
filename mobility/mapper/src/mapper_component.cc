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
#include <mapper/mapper_component.h>

/**
 * \ingroup mobility
 */
namespace mapper {

MapperComponent::MapperComponent(const rclcpp::NodeOptions& options) :
    ff_util::FreeFlyerComponent(options, NODE_MAPPER, true), state_(IDLE) {
}

MapperComponent::~MapperComponent() {}

void MapperComponent::Initialize(NodeHandle &nh) {
  // Store the node handle for future use
  nh_ = nh;
  buffer_.reset(new tf2_ros::Buffer(nh->get_clock()));
  listener_.reset(new tf2_ros::TransformListener(*buffer_));

  // Grab some configuration parameters for this node from the config reader
  cfg_.AddFile("mobility/mapper.config");
  if (!cfg_.Initialize(nh)) {
    AssertFault(ff_util::INITIALIZATION_FAILED,
                "Could not start config server", GetTimeNow());
    return;
  }

  // Setup a timer to forward diagnostics
  timer_d_.createTimer(1 / DEFAULT_DIAGNOSTICS_RATE,
      std::bind(&MapperComponent::DiagnosticsCallback, this), nh, false, true);

  // load parameters
  double map_resolution, memory_time, max_range, min_range, collision_distance, robot_radius;
  double cam_fov, aspect_ratio;
  double occupancy_threshold, probability_hit, probability_miss;
  double clamping_threshold_max, clamping_threshold_min;
  double traj_resolution, compression_max_dev;
  disable_mapper_ = cfg_.Get<bool>("disable_mapper");
  map_resolution = cfg_.Get<double>("map_resolution");
  max_range = cfg_.Get<double>("max_range");
  min_range = cfg_.Get<double>("min_range");
  memory_time = cfg_.Get<double>("memory_time");
  collision_distance = cfg_.Get<double>("collision_distance");
  robot_radius = cfg_.Get<double>("robot_radius");
  cam_fov = cfg_.Get<double>("cam_fov");
  aspect_ratio = cfg_.Get<double>("cam_aspect_ratio");
  occupancy_threshold = cfg_.Get<double>("occupancy_threshold");
  probability_hit = cfg_.Get<double>("probability_hit");
  probability_miss = cfg_.Get<double>("probability_miss");
  clamping_threshold_min = cfg_.Get<double>("clamping_threshold_min");
  clamping_threshold_max = cfg_.Get<double>("clamping_threshold_max");
  compression_max_dev = cfg_.Get<double>("traj_compression_max_dev");
  traj_resolution = cfg_.Get<double>("traj_compression_resolution");
  octomap_update_rate_ = cfg_.Get<double>("octomap_update_rate");
  fading_memory_update_rate_ = cfg_.Get<double>("fading_memory_update_rate");
  use_haz_cam_ = cfg_.Get<bool>("use_haz_cam");
  use_perch_cam_ = cfg_.Get<bool>("use_perch_cam");

  // update tree parameters
  globals_.octomap.SetResolution(map_resolution);
  globals_.octomap.SetMaxRange(max_range);
  globals_.octomap.SetMinRange(min_range);
  globals_.octomap.SetMemoryTime(memory_time);
  globals_.octomap.SetMapInflation(collision_distance + robot_radius);
  globals_.octomap.SetCamFrustum(cam_fov, aspect_ratio);
  globals_.octomap.SetOccupancyThreshold(occupancy_threshold);
  globals_.octomap.SetHitMissProbabilities(probability_hit, probability_miss);
  globals_.octomap.SetClampingThresholds(
    clamping_threshold_min, clamping_threshold_max);

  // update trajectory discretization parameters (used in collision check)
  globals_.sampled_traj.SetMaxDev(compression_max_dev);
  globals_.sampled_traj.SetResolution(traj_resolution);

  // Publishers
  obstacle_marker_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::MarkerArray,
    TOPIC_MAPPER_OCTOMAP_MARKERS, 1);
  free_space_marker_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::MarkerArray,
    TOPIC_MAPPER_OCTOMAP_FREE_MARKERS, 1);
  inflated_obstacle_marker_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::MarkerArray,
    TOPIC_MAPPER_OCTOMAP_INFLATED_MARKERS, 1);
  inflated_free_space_marker_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::MarkerArray,
    TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_MARKERS, 1);
  path_marker_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::MarkerArray,
    TOPIC_MAPPER_DISCRETE_TRAJECTORY_MARKERS, 1);
  cam_frustum_pub_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::Marker,
    TOPIC_MAPPER_FRUSTRUM_MARKERS, 1);
  free_space_cloud_pub_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::PointCloud2,
    TOPIC_MAPPER_OCTOMAP_FREE_CLOUD, 1);
  obstacle_cloud_pub_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::PointCloud2,
    TOPIC_MAPPER_OCTOMAP_CLOUD, 1);
  inflated_free_space_cloud_pub_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::PointCloud2,
    TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_CLOUD, 1);
  inflated_obstacle_cloud_pub_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::PointCloud2,
    TOPIC_MAPPER_OCTOMAP_INFLATED_CLOUD, 1);
  hazard_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::Hazard,
    TOPIC_MOBILITY_HAZARD, 1);

    // Timers
    timer_o_.createTimer(1 / octomap_update_rate_,
        std::bind(&MapperComponent::PclCallback, this), nh, false, false);
    timer_f_.createTimer(1 / fading_memory_update_rate_,
        std::bind(&MapperComponent::FadeTask, this), nh, false, false);

  if (disable_mapper_) {
    FF_WARN("Mapper disabled, obstacle avoidance not working!");
  } else {
    // Start timers
    timer_o_.start();
    timer_f_.start();
    // Subscribers
    segment_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::Segment, TOPIC_GNC_CTL_SEGMENT, 1,
      std::bind(&MapperComponent::SegmentCallback, this, std::placeholders::_1));
    reset_sub_ = FF_CREATE_SUBSCRIBER(nh, std_msgs::Empty, TOPIC_GNC_EKF_RESET, 1,
      std::bind(&MapperComponent::ResetCallback, this, std::placeholders::_1));
  }

  // Services
  set_resolution_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::SetFloat,
    SERVICE_MOBILITY_SET_MAP_RESOLUTION,
    std::bind(&MapperComponent::SetResolution, this, std::placeholders::_1, std::placeholders::_2));
  set_memory_time_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::SetFloat,
    SERVICE_MOBILITY_SET_MEMORY_TIME,
    std::bind(&MapperComponent::SetMemoryTime, this, std::placeholders::_1, std::placeholders::_2));
  set_collision_distance_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::SetFloat,
    SERVICE_MOBILITY_SET_COLLISION_DISTANCE,
    std::bind(&MapperComponent::SetCollisionDistance, this, std::placeholders::_1, std::placeholders::_2));
  get_resolution_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::GetFloat,
    SERVICE_MOBILITY_GET_MAP_RESOLUTION,
    std::bind(&MapperComponent::GetResolution, this, std::placeholders::_1, std::placeholders::_2));
  get_memory_time_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::GetFloat,
    SERVICE_MOBILITY_GET_MEMORY_TIME,
    std::bind(&MapperComponent::GetMemoryTime, this, std::placeholders::_1, std::placeholders::_2));
  get_collision_distance_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::GetFloat,
    SERVICE_MOBILITY_GET_MAP_INFLATION,
    std::bind(&MapperComponent::GetMapInflation, this, std::placeholders::_1, std::placeholders::_2));
  reset_map_srv_ = FF_CREATE_SERVICE(nh, std_srvs::Trigger,
    SERVICE_MOBILITY_RESET_MAP,
    std::bind(&MapperComponent::ResetMap, this, std::placeholders::_1, std::placeholders::_2));
  get_free_map_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::GetMap,
    SERVICE_MOBILITY_GET_FREE_MAP,
    std::bind(&MapperComponent::GetFreeMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_obstacle_map_srv_ = FF_CREATE_SERVICE(nh, ff_msgs::GetMap,
    SERVICE_MOBILITY_GET_OBSTACLE_MAP,
    std::bind(&MapperComponent::GetObstacleMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Notify initialization complete
  FF_DEBUG_STREAM("Initialization complete");
}
}  // namespace mapper

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mapper::MapperComponent)
