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
#include <mapper/mapper_nodelet.h>

/**
 * \ingroup mobility
 */
namespace mapper {

MapperNodelet::MapperNodelet() :
    ff_util::FreeFlyerNodelet(NODE_MAPPER), state_(IDLE) {
}

MapperNodelet::~MapperNodelet() {}

void MapperNodelet::Initialize(ros::NodeHandle *nh) {
  listener_.reset(new tf2_ros::TransformListener(buffer_));

  // Grab some configuration parameters for this node from the config reader
  cfg_.Initialize(GetPrivateHandle(), "mobility/mapper.config");
  if (!cfg_.Listen(boost::bind(
      &MapperNodelet::ReconfigureCallback, this, _1))) {
    AssertFault(ff_util::INITIALIZATION_FAILED,
                "Could not start config server");
    return;
  }

  // Setup a timer to forward diagnostics
  timer_d_ = nh->createTimer(
    ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
      &MapperNodelet::DiagnosticsCallback, this, false, true);

  // load parameters
  bool disable_mapper = false;
  double map_resolution, memory_time, max_range, min_range, collision_distance, robot_radius;
  double cam_fov, aspect_ratio;
  double occupancy_threshold, probability_hit, probability_miss;
  double clamping_threshold_max, clamping_threshold_min;
  double traj_resolution, compression_max_dev;
  disable_mapper = cfg_.Get<bool>("disable_mapper");
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
  obstacle_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MAPPER_OCTOMAP_MARKERS, 1);
  free_space_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MAPPER_OCTOMAP_FREE_MARKERS, 1);
  inflated_obstacle_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MAPPER_OCTOMAP_INFLATED_MARKERS, 1);
  inflated_free_space_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_MARKERS, 1);
  path_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MAPPER_DISCRETE_TRAJECTORY_MARKERS, 1);
  cam_frustum_pub_ = nh->advertise<visualization_msgs::Marker>(
    TOPIC_MAPPER_FRUSTRUM_MARKERS, 1);
  free_space_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
    TOPIC_MAPPER_OCTOMAP_FREE_CLOUD, 1, true);
  obstacle_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
    TOPIC_MAPPER_OCTOMAP_CLOUD, 1, true);
  inflated_free_space_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
    TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_CLOUD, 1, true);
  inflated_obstacle_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
    TOPIC_MAPPER_OCTOMAP_INFLATED_CLOUD, 1, true);
  hazard_pub_ = nh->advertise<ff_msgs::Hazard>(
    TOPIC_MOBILITY_HAZARD, 1);

  if (disable_mapper) {
    NODELET_WARN("Mapper disabled, obstacle avoidance not working!");
  } else {
    // Timers
    timer_o_ = nh->createTimer(
      ros::Duration(ros::Rate(octomap_update_rate_)),
        &MapperNodelet::PclCallback, this, false, true);
    timer_f_ = nh->createTimer(
      ros::Duration(ros::Rate(fading_memory_update_rate_)),
        &MapperNodelet::FadeTask, this, false, true);

    // Subscribers
    segment_sub_ = nh->subscribe(TOPIC_GNC_CTL_SEGMENT, 1,
      &MapperNodelet::SegmentCallback, this);
    reset_sub_ = nh->subscribe(TOPIC_GNC_EKF_RESET, 1,
      &MapperNodelet::ResetCallback, this);
  }

  // Services
  set_resolution_srv_ = nh->advertiseService(SERVICE_MOBILITY_SET_MAP_RESOLUTION,
    &MapperNodelet::SetResolution, this);
  set_memory_time_srv_ = nh->advertiseService(SERVICE_MOBILITY_SET_MEMORY_TIME,
    &MapperNodelet::SetMemoryTime, this);
  set_collision_distance_srv_ = nh->advertiseService(SERVICE_MOBILITY_SET_COLLISION_DISTANCE,
    &MapperNodelet::SetCollisionDistance, this);
  get_resolution_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_MAP_RESOLUTION,
    &MapperNodelet::GetResolution, this);
  get_memory_time_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_MEMORY_TIME,
    &MapperNodelet::GetMemoryTime, this);
  get_collision_distance_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_MAP_INFLATION,
    &MapperNodelet::GetMapInflation, this);
  reset_map_srv_ = nh->advertiseService(SERVICE_MOBILITY_RESET_MAP,
    &MapperNodelet::ResetMap, this);
  get_free_map_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_FREE_MAP,
    &MapperNodelet::GetFreeMapCallback, this);
  get_obstacle_map_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_OBSTACLE_MAP,
    &MapperNodelet::GetObstacleMapCallback, this);

  // Notify initialization complete
  NODELET_DEBUG_STREAM("Initialization complete");
}


PLUGINLIB_EXPORT_CLASS(mapper::MapperNodelet, nodelet::Nodelet);

}  // namespace mapper
