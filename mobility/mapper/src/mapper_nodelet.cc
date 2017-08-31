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

// FSW utils
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_serialization.h>
#include <ff_util/config_server.h>

// For plugin loading
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Keepout zones for the planner
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range_core.hpp>

// For plotting to RVIZ
#include <visualization_msgs/MarkerArray.h>

// Action servers
#include <ff_msgs/ValidateAction.h>

// Service definition for zone registration
#include <ff_msgs/Zone.h>
#include <ff_msgs/SetZones.h>
#include <ff_msgs/GetZones.h>

// For storing bounding box
#include <fstream>
#include <vector>
#include <string>
#include <exception>

/**
 * \ingroup mobility
 */
namespace mapper {

namespace fs = boost::filesystem;

// Convenience declarations
using RESPONSE = ff_msgs::ValidateResult;

class MapperNodelet : public ff_util::FreeFlyerNodelet {
 public:
  enum State {
    IDLE,
    VALIDATING
  };

  // Constructor
  MapperNodelet() : ff_util::FreeFlyerNodelet(NODE_MAPPER, true), state_(IDLE) {}

  // Destructor
  virtual ~MapperNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config reader
    cfg_.Initialize(GetPrivateHandle(), "mobility/mapper.config");
    cfg_.Listen(boost::bind(&MapperNodelet::ReconfigureCallback, this, _1));

    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
      &MapperNodelet::DiagnosticsCallback, this, false, true);

    // For publishing the keep outs and keep ins to RVIZ (both persistent)
    pub_m_ = nh->advertise < visualization_msgs::MarkerArray > (TOPIC_MOBILITY_ZONES, 10, true);

    // Allow planners to register themselves
    srv_s_ = nh->advertiseService(SERVICE_MOBILITY_SET_ZONES, &MapperNodelet::SetZonesCallback, this);
    srv_g_ = nh->advertiseService(SERVICE_MOBILITY_GET_ZONES, &MapperNodelet::GetZonesCallback, this);

    // Setup the execute action
    server_v_.SetGoalCallback(std::bind(&MapperNodelet::GoalCallback, this, std::placeholders::_1));
    server_v_.SetPreemptCallback(std::bind(&MapperNodelet::PreemptCallback, this));
    server_v_.SetCancelCallback(std::bind(&MapperNodelet::CancelCallback, this));
    server_v_.Create(nh, ACTION_MOBILITY_VALIDATE);

    // Grab the zone directory value from the LUA config
    config_reader::ConfigReader *handle = cfg_.GetConfigReader();
    if (handle == nullptr)
      NODELET_FATAL_STREAM("Cannot read LUA config");
    std::string zone_dir;
    if (!handle->GetStr("zone_directory", &zone_dir))
      NODELET_FATAL_STREAM("Cannot read zone directory from LUA config");

    // Resolve the zone directory and load the keepins and keepouts files
    fs::path dir(zone_dir);
    if (!fs::exists(dir) || !fs::is_directory(dir))
      NODELET_FATAL_STREAM("Cannot open zone directory");
    // Set the default time to 1 to check if this changed on read
    bool found = false;
    // Try and read the zone
    auto files = boost::make_iterator_range(fs::directory_iterator(dir), fs::directory_iterator());
    for (fs::path const& e : files) {
      if (e.extension() != ".bin")
        continue;
      ff_msgs::SetZones::Request zones;
      ff_util::ReadFile(e.native(), zones);
      if (zones.timestamp >= zones_.timestamp) {
        found = true;
        zones_ = zones;
      }
    }
    // Special case: nothing was loaded.
    if (!found)
      NODELET_WARN_STREAM("No zone files loaded");
    // Update the RVIZ markers
    UpdateMarkers();
    // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
  }

  // Send diagnostics
  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  // Configure callback
  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (state_ != IDLE)
      return false;
    cfg_.Reconfigure(config);
    return true;
  }

  // Callback to get the zones
  bool GetZonesCallback(ff_msgs::GetZones::Request& req, ff_msgs::GetZones::Response& res) {
    res.timestamp = zones_.timestamp;
    res.zones = zones_.zones;
    return true;
  }

  // Callback to set the zones
  bool SetZonesCallback(ff_msgs::SetZones::Request& req, ff_msgs::SetZones::Response& res) {  //NOLINT
    if (req.timestamp >= zones_.timestamp) {
      zones_ = req;
      fs::path zone_file(std::to_string(zones_.timestamp.sec) + ".bin");
      ff_util::WriteFile((zone_dir_ / zone_file).native(), zones_);
      UpdateMarkers();
      return true;
    }
    return false;
  }

  // Publish the markers for the keepins and keepouts
  void UpdateMarkers() {
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
    pub_m_.publish(markers);
    old_markers = markers;
  }

  // Assert a fault - katie's fault code handling will eventually go in here
  void Complete(int32_t response) {
    NODELET_DEBUG_STREAM("Complete(" << response <<")");
    switch (state_) {
    case VALIDATING: {
      ff_msgs::ValidateResult validate_result;
      validate_result.response = response;
      validate_result.flight_mode = flight_mode_;
      validate_result.segment = segment_;
      if (response > 0)
        server_v_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, validate_result);
      else if (response < 0)
        server_v_.SendResult(ff_util::FreeFlyerActionState::ABORTED, validate_result);
      else
        server_v_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, validate_result);
      break;
    }
    case IDLE:
    default:
      NODELET_WARN_STREAM("Complete() called in IDLE state");
      break;
    }
    state_ = IDLE;
  }

  // VALIDATE ACTION

  // A new move  goal arrives asynchronously
  void GoalCallback(ff_msgs::ValidateGoalConstPtr const& validate_goal) {
    NODELET_DEBUG_STREAM("GoalCallback()");
    switch (state_) {
    case IDLE: {
      // Cover a special case where not enough info
      if (validate_goal->flight_mode.empty())
        return Complete(RESPONSE::NO_FLIGHT_MODE_SPECIFIED);
      if (validate_goal->segment.empty())
        return Complete(RESPONSE::UNEXPECTED_EMPTY_SEGMENT);
      // Start the validation
      flight_mode_ = validate_goal->flight_mode;
      segment_ = validate_goal->segment;
      state_ = VALIDATING;
      // For now, just do the usual validation tricks
      ff_util::SegmentResult ret = ff_util::SegmentUtil::Check(ff_util::CHECK_ALL, segment_, flight_mode_);
      // Print a nice readable error error
      if (ret != ff_util::SUCCESS) {
        NODELET_DEBUG_STREAM(*validate_goal);
        NODELET_WARN_STREAM("Validate failed: " <<  ff_util::SegmentUtil::GetDescription(ret));
      }
      // Do something based on the result
      switch (ret) {
      case ff_util::SUCCESS:
        return Complete(RESPONSE::SUCCESS);
      case ff_util::ERROR_MINIMUM_FREQUENCY:
        return Complete(RESPONSE::MINIMUM_FREQUENCY_NOT_MET);
      case ff_util::ERROR_STATIONARY_ENDPOINT:
        return Complete(RESPONSE::ENDPOINT_NOT_STATIONARY);
      case ff_util::ERROR_MINIMUM_NUM_SETPOINTS:
        return Complete(RESPONSE::NOT_ENOUGH_SETPOINTS);
      case ff_util::ERROR_TIME_RUNS_BACKWARDS:
        return Complete(RESPONSE::TIME_RUNS_BACKWARDS);
      case ff_util::ERROR_LIMITS_VEL:
        return Complete(RESPONSE::EXCEEDS_LIMITS_VEL);
      case ff_util::ERROR_LIMITS_ACCEL:
        return Complete(RESPONSE::EXCEEDS_LIMITS_ACCEL);
      case ff_util::ERROR_LIMITS_OMEGA:
        return Complete(RESPONSE::EXCEEDS_LIMITS_OMEGA);
      case ff_util::ERROR_LIMITS_ALPHA:
        return Complete(RESPONSE::EXCEEDS_LIMITS_ALPHA);
      case ff_util::ERROR_INVALID_FLIGHT_MODE:
        return Complete(RESPONSE::INVALID_FLIGHT_MODE);
      case ff_util::ERROR_INVALID_GENERAL_CONFIG:
        return Complete(RESPONSE::INVALID_GENERAL_CONFIG);
      }
      break;
    }
    case VALIDATING:
    default:
      NODELET_WARN_STREAM("GoalCallback() called in IDLE state");
      break;
    }
  }

  void PreemptCallback(void) {
    NODELET_DEBUG_STREAM("PreemptCallback()");
    return Complete(RESPONSE::PREEMPTED);
  }

  void CancelCallback(void) {
    NODELET_DEBUG_STREAM("CancelCallback()");
    return Complete(RESPONSE::CANCELLED);
  }


 protected:
  State state_;                                                           // State of the mapper
  std::string zone_dir_;                                                  // Zone file directory
  ff_msgs::SetZones::Request zones_;                                      // Zone set request
  ff_util::FreeFlyerActionServer < ff_msgs::ValidateAction > server_v_;   // Mappere::Validate
  ff_util::Segment segment_;                                              // Segment
  std::string flight_mode_;                                               // Flight mode
  ros::Publisher pub_m_;                                                  // Visualization of map
  ros::ServiceServer srv_g_;                                              // Get zone service
  ros::ServiceServer srv_s_;                                              // Set zone service
  ros::Timer timer_d_;                                                    // Diangostics
  ff_util::ConfigServer cfg_;                                             // Config server
};

PLUGINLIB_DECLARE_CLASS(mapper, MapperNodelet,
                        mapper::MapperNodelet, nodelet::Nodelet);

}  // namespace mapper
