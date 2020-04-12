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

#include <choreographer/validator.h>

#include <vector>

namespace choreographer {

// Check if a point is inside a cuboid
bool Validator::PointInsideCuboid(geometry_msgs::Point const& x,
                                      geometry_msgs::Vector3 const& cubemin,
                                      geometry_msgs::Vector3 const& cubemax) {
  if (x.x < cubemin.x || x.y < cubemin.y || x.z < cubemin.z ||
      x.x > cubemax.x || x.y > cubemax.y || x.z > cubemax.z) return false;
  return true;
}

// Check that we are within a keep in and outside all keep out zones
Validator::Response Validator::CheckSegment(ff_util::Segment const& msg,
  ff_msgs::FlightMode const& flight_mode, bool face_forward) {
  // Calculate a resample rate that ensures we will never have a large enough
  // stride between setpoints that we could pass through an object. At half a
  // meter per second -- the max velocity -- a 10Hz check would cover 5cm,
  // which is smaller than the radius of the freeflyer.
  ff_util::Segment seg;
  if (ff_util::FlightUtil::Resample(msg, seg, 10.0) != ff_util::SUCCESS) {
    ROS_DEBUG("Could not resample segment at 10Hz");
    return VIOLATES_RESAMPLING;
  }
  // Do some basic checks on the segment before zone validation
  ff_util::SegmentResult check_result = ff_util::FlightUtil::Check(
    ff_util::SegmentCheckMask::CHECK_ALL, seg, flight_mode, face_forward);
  switch (check_result) {
  case ff_util::SegmentResult::ERROR_MINIMUM_FREQUENCY:
    return VIOLATES_MINIMUM_FREQUENCY;
  case ff_util::SegmentResult::ERROR_STATIONARY_ENDPOINT:
    return VIOLATES_STATIONARY_ENDPOINT;
  case ff_util::SegmentResult::ERROR_MINIMUM_NUM_SETPOINTS:
    return VIOLATES_FIRST_IN_PAST;
  case ff_util::SegmentResult::ERROR_TIME_RUNS_BACKWARDS:
    return VIOLATES_MINIMUM_SETPOINTS;
  case ff_util::SegmentResult::ERROR_LIMITS_VEL:
    return VIOLATES_HARD_LIMIT_VEL;
  case ff_util::SegmentResult::ERROR_LIMITS_ACCEL:
    return VIOLATES_HARD_LIMIT_ACCEL;
  case ff_util::SegmentResult::ERROR_LIMITS_OMEGA:
    return VIOLATES_HARD_LIMIT_OMEGA;
  case ff_util::SegmentResult::ERROR_LIMITS_ALPHA:
    return VIOLATES_HARD_LIMIT_ALPHA;
  default:
    break;
  }
  // Now, check each setpoint in the segment against the zones
  ff_util::Segment::const_iterator it;
  for (it = seg.begin(); it != seg.end(); it++) {
    std::vector<ff_msgs::Zone>::iterator jt;
    // We must visit at least one keepin to be valid
    bool point_exists_within_keepin = false;
    for (jt = zones_.zones.begin(); jt != zones_.zones.end(); jt++) {
      if (jt->type == ff_msgs::Zone::KEEPIN)
        if (PointInsideCuboid(it->pose.position, jt->min, jt->max))
          point_exists_within_keepin = true;
      if (jt->type == ff_msgs::Zone::KEEPOUT) {
        if (PointInsideCuboid(it->pose.position, jt->min, jt->max)) {
          ROS_DEBUG_STREAM("KEEPOUT violation at time" << it->when.toSec());
          ROS_DEBUG_STREAM(it->pose.position);
          ROS_DEBUG_STREAM(zones_);
          return VIOLATES_KEEP_OUT;
        }
      }
    }
    // Check that we are in a keepin
    if (!point_exists_within_keepin)
      return VIOLATES_KEEP_IN;
  }
  return SUCCESS;
}

// Load the keep in and keepout zones into memory
bool Validator::Init(ros::NodeHandle *nh, ff_util::ConfigServer & cfg) {
  // Create the zone publisher and service getter/setter
  pub_zones_ = nh->advertise<visualization_msgs::MarkerArray>(
    TOPIC_MOBILITY_ZONES, 1, true);
  set_zones_srv_ = nh->advertiseService(SERVICE_MOBILITY_SET_ZONES,
    &Validator::SetZonesCallback, this);
  get_zones_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_ZONES,
    &Validator::GetZonesCallback, this);
  // Check if overwriting is allowed
  zone_file_ = cfg.Get<std::string>("zone_file");
  overwrite_ = cfg.Get<bool>("zone_overwrite");
  // Try and open the zone file. If this doesn't work, that's OK. It just means
  // that we need to wait for them to be updated.
  if (!ff_util::Serialization::ReadFile(zone_file_, zones_)) {
    ROS_WARN_STREAM("Cannot open zone file " << zone_file_);
  } else {
    PublishMarkers();
  }
  // Success
  return true;
}

// Publish the markers for the keepins and keepouts
void Validator::PublishMarkers() {
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
      marker.color.a = 0.1;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      break;
    case ff_msgs::Zone::KEEPIN:
      marker.color.a = 0.1;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      break;
    case ff_msgs::Zone::CLUTTER:
      marker.color.a = 0.1;
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
  pub_zones_.publish(markers);
  old_markers = markers;
}

// Callback to get the keep in/out zones
bool Validator::GetZonesCallback(
  ff_msgs::GetZones::Request &req, ff_msgs::GetZones::Response &res) {
  res.timestamp = zones_.timestamp;
  res.zones = zones_.zones;
  return true;
}

// Callback to set the keep in/out zones
bool Validator::SetZonesCallback(
  ff_msgs::SetZones::Request &req, ff_msgs::SetZones::Response &res) {
  // Update the zones
  zones_ = req;
  // If we should write the new zones to a file and use them by default
  if (overwrite_ && !ff_util::Serialization::WriteFile(zone_file_, zones_))
    ROS_WARN_STREAM("Cannot write zone file " << zone_file_);
  // Update visualization
  PublishMarkers();
  // Send result
  res.success = true;
  return true;
}

}  // namespace choreographer
