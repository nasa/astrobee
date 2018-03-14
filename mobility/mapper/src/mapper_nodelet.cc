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

MapperNodelet::~MapperNodelet() {
    // Join all threads
    h_haz_tf_thread_.join();
    h_perch_tf_thread_.join();
    h_body_tf_thread_.join();
    h_octo_thread_.join();
    h_fade_thread_.join();
    h_collision_check_thread_.join();
    // destroy mutexes and semaphores
    mutexes_.destroy();
    semaphores_.destroy();
}

void MapperNodelet::Initialize(ros::NodeHandle *nh) {
  // Grab some configuration parameters for this node from the config reader
    cfg_.Initialize(GetPrivateHandle(), "mobility/mapper.config");
    cfg_.Listen(boost::bind(&MapperNodelet::ReconfigureCallback, this, _1));

    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(
        ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
        &MapperNodelet::DiagnosticsCallback, this, false, true);

    // load parameters
    double map_resolution, memory_time, max_range, min_range, inflate_radius;
    double cam_fov, aspect_ratio;
    double occupancy_threshold, probability_hit, probability_miss;
    double clamping_threshold_max, clamping_threshold_min;
    double traj_resolution, compression_max_dev;
    bool use_haz_cam, use_perch_cam;
    map_resolution = cfg_.Get<double>("map_resolution");
    max_range = cfg_.Get<double>("max_range");
    min_range = cfg_.Get<double>("min_range");
    memory_time = cfg_.Get<double>("memory_time");
    inflate_radius = cfg_.Get<double>("inflate_radius");
    cam_fov = cfg_.Get<double>("cam_fov");
    aspect_ratio = cfg_.Get<double>("cam_aspect_ratio");
    occupancy_threshold = cfg_.Get<double>("occupancy_threshold");
    probability_hit = cfg_.Get<double>("probability_hit");
    probability_miss = cfg_.Get<double>("probability_miss");
    clamping_threshold_min = cfg_.Get<double>("clamping_threshold_min");
    clamping_threshold_max = cfg_.Get<double>("clamping_threshold_max");
    compression_max_dev = cfg_.Get<double>("traj_compression_max_dev");
    traj_resolution = cfg_.Get<double>("traj_compression_resolution");
    tf_update_rate_ = cfg_.Get<double>("tf_update_rate");
    fading_memory_update_rate_ = cfg_.Get<double>("fading_memory_update_rate");
    use_haz_cam = cfg_.Get<bool>("use_haz_cam");
    use_perch_cam = cfg_.Get<bool>("use_perch_cam");

    // update tree parameters
    globals_.octomap.SetResolution(map_resolution);
    globals_.octomap.SetMaxRange(max_range);
    globals_.octomap.SetMinRange(min_range);
    globals_.octomap.SetMemory(memory_time);
    globals_.octomap.SetMapInflation(inflate_radius);
    globals_.octomap.SetCamFrustum(cam_fov, aspect_ratio);
    globals_.octomap.SetOccupancyThreshold(occupancy_threshold);
    globals_.octomap.SetHitMissProbabilities(probability_hit, probability_miss);
    globals_.octomap.SetClampingThresholds(clamping_threshold_min, clamping_threshold_max);

    // update trajectory discretization parameters (used in collision check)
    globals_.sampled_traj.SetMaxDev(compression_max_dev);
    globals_.sampled_traj.SetResolution(traj_resolution);

    // threads --------------------------------------------------
    h_haz_tf_thread_ = std::thread(&MapperNodelet::HazTfTask, this);
    h_perch_tf_thread_ = std::thread(&MapperNodelet::PerchTfTask, this);
    h_body_tf_thread_ = std::thread(&MapperNodelet::BodyTfTask, this);
    h_octo_thread_ = std::thread(&MapperNodelet::OctomappingTask, this);
    h_fade_thread_ = std::thread(&MapperNodelet::FadeTask, this);
    h_collision_check_thread_ = std::thread(&MapperNodelet::CollisionCheckTask, this);

    // Create services ------------------------------------------
    resolution_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_UPDATE_MAP_RESOLUTION, &MapperNodelet::UpdateResolution, this);
    memory_time_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_UPDATE_MEMORY_TIME, &MapperNodelet::UpdateMemoryTime, this);
    map_inflation_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_UPDATE_INFLATION, &MapperNodelet::MapInflation, this);
    reset_map_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_RESET_MAP, &MapperNodelet::ResetMap, this);
    set_zones_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_SET_ZONES, &MapperNodelet::SetZonesCallback, this);
    get_zones_srv_ = nh->advertiseService(
        SERVICE_MOBILITY_GET_ZONES, &MapperNodelet::GetZonesCallback, this);

    // Setup the trajectory validation action
    server_v_.SetGoalCallback(std::bind(&MapperNodelet::GoalCallback, this, std::placeholders::_1));
    server_v_.SetPreemptCallback(std::bind(&MapperNodelet::PreemptCallback, this));
    server_v_.SetCancelCallback(std::bind(&MapperNodelet::CancelCallback, this));
    server_v_.Create(nh, ACTION_MOBILITY_VALIDATE);

    // Subscribers ----------------------------------------------
    std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
    std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;
    if (use_haz_cam) {
        std::string cam = TOPIC_HARDWARE_NAME_HAZ_CAM;
        haz_sub_ = nh->subscribe(cam_prefix + cam + cam_suffix, 10, &MapperNodelet::PclCallback, this);
    }
    if (use_perch_cam) {
        std::string cam = TOPIC_HARDWARE_NAME_PERCH_CAM;
        perch_sub_ = nh->subscribe(cam_prefix + cam + cam_suffix, 10, &MapperNodelet::PclCallback, this);
    }
    segment_sub_ = nh->subscribe(TOPIC_GNC_CTL_SEGMENT, 10, &MapperNodelet::SegmentCallback, this);

    // Publishers -----------------------------------------------
    sentinel_pub_ =
        nh->advertise<geometry_msgs::PointStamped>(TOPIC_MOBILITY_COLLISIONS, 10);
    obstacle_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MAPPER_OCTOMAP_MARKERS, 10);
    free_space_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MAPPER_OCTOMAP_FREE_MARKERS, 10);
    inflated_obstacle_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MAPPER_OCTOMAP_INFLATED_MARKERS, 10);
    inflated_free_space_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MAPPER_OCTOMAP_INFLATED_FREE_MARKERS, 10);
    path_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MAPPER_DISCRETE_TRAJECTORY_MARKERS, 10);
    cam_frustum_pub_ =
        nh->advertise<visualization_msgs::Marker>(TOPIC_MAPPER_FRUSTRUM_MARKERS, 10);
    map_keep_in_out_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(TOPIC_MOBILITY_ZONES, 10, true);

    // Loading keep-in and keep-out zones -------------------------
    LoadKeepInOutZones();

    // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
}

void MapperNodelet::LoadKeepInOutZones() {
    // Grab the zone directory value from the LUA config
    config_reader::ConfigReader *handle = cfg_.GetConfigReader();
    if (handle == nullptr)
      NODELET_FATAL_STREAM("Cannot read LUA config");
    if (!handle->GetStr("zone_directory", &zone_dir_))
      NODELET_FATAL_STREAM("Cannot read zone directory from LUA config");

    // Resolve the zone directory and load the keepins and keepouts files
    fs::path dir(zone_dir_);
    if (!fs::exists(dir) || !fs::is_directory(dir))
      NODELET_FATAL_STREAM("Cannot open zone directory");
    // Set the default time to 1 to check if this changed on read
    bool found = false;
    // Try and read the zone
    auto files = boost::make_iterator_range(fs::directory_iterator(dir),
      fs::directory_iterator());
    for (fs::path const& e : files) {
      if (e.extension() != ".bin")
        continue;
      ff_msgs::SetZones::Request zones;
      if (ff_util::Serialization::ReadFile(e.native(), zones)) {
        if (zones.timestamp >= zones_.timestamp) {
          found = true;
          zones_ = zones;
        }
      } else {
        NODELET_WARN_STREAM("Cannot open zone file " << e.native());
      }
    }
    // Special case: nothing was loaded.
    if (!found)
      NODELET_WARN_STREAM("No zone files loaded");
    // Update the RVIZ markers
    UpdateKeepInOutMarkers();
}

// Publish the markers for the keepins and keepouts
void MapperNodelet::UpdateKeepInOutMarkers() {
    static visualization_msgs::MarkerArray old_markers;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.header.frame_id = "world";
    marker.header.seq = 0;
    marker.ns = "zone_visualization";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    std::vector < ff_msgs::Zone > ::iterator it;
    for (it = zones_.zones.begin(); it != zones_.zones.end(); it++) {
        switch (it->type) {
        case ff_msgs::Zone::KEEPOUT:
            marker.color.a = 0.4;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            break;
        case ff_msgs::Zone::KEEPIN:
            marker.color.a = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        case ff_msgs::Zone::CLUTTER:
            marker.color.a = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            break;
        }
        marker.pose.position.x = 0.5 * (it->min.x + it->max.x);
        marker.pose.position.y = 0.5 * (it->min.y + it->max.y);
        marker.pose.position.z = 0.5 * (it->min.z + it->max.z);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = fabs(it->min.x - it->max.x);
        marker.scale.y = fabs(it->min.y - it->max.y);
        marker.scale.z = fabs(it->min.z - it->max.z);
        markers.markers.push_back(marker);
        marker.id++;
    }
    for (uint i = markers.markers.size(); i < old_markers.markers.size(); i++) {
        markers.markers.push_back(old_markers.markers.at(i));
        markers.markers.back().action = visualization_msgs::Marker::DELETE;
    }
    map_keep_in_out_pub_.publish(markers);
    old_markers = markers;
}

PLUGINLIB_EXPORT_CLASS(mapper::MapperNodelet, nodelet::Nodelet);

}  // namespace mapper
