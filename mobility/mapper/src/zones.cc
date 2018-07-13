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

#include <mapper/mapper_nodelet.h>
#include <limits>
#include <vector>

namespace mapper {

// Check if a point is inside a cuboid
bool MapperNodelet::PointInsideCuboid(geometry_msgs::Point const& x,
                                      geometry_msgs::Vector3 const& cubemin,
                                      geometry_msgs::Vector3 const& cubemax) {
  if (x.x < cubemin.x || x.y < cubemin.y || x.z < cubemin.z ||
      x.x > cubemax.x || x.y > cubemax.y || x.z > cubemax.z) return false;
  return true;
}

// Are we inside all keepings and outside all keepouts
bool MapperNodelet::CheckZones(ff_util::Segment const& msg,
  bool check_keepins, bool check_keepouts, ff_msgs::Hazard &info) {
  // Calculate a resample rate that ensures we will never have a large enough
  // stride between setpoints that we could pass through an object. At half a
  // meter per second -- the max velocity -- a 10Hz check would cover 5cm,
  // which is smaller than the radius of the freeflyer.
  ff_util::Segment seg;
  if (ff_util::FlightUtil::Resample(msg, seg, 10.0) != ff_util::SUCCESS) {
    NODELET_DEBUG("Could not resample segment at 10Hz");
    return false;
  }
  // Now, check each setpoint in the segment
  ff_util::Segment::const_iterator it;
  for (it = seg.begin(); it != seg.end(); it++) {
    std::vector<ff_msgs::Zone>::iterator jt;
    // We must visit at least one keepin to be valid
    bool point_exists_within_keepin = false;
    for (jt = zones_.zones.begin(); jt != zones_.zones.end(); jt++) {
      if (check_keepins && jt->type == ff_msgs::Zone::KEEPIN) {
        if (PointInsideCuboid(it->pose.position, jt->min, jt->max))
          point_exists_within_keepin = true;
      }
      if (check_keepouts && jt->type == ff_msgs::Zone::KEEPOUT) {
        if (PointInsideCuboid(it->pose.position, jt->min, jt->max)) {
          NODELET_DEBUG_STREAM("KEEPOUT violation at time" << it->when.toSec());
          NODELET_DEBUG_STREAM(it->pose.position);
          NODELET_DEBUG_STREAM(zones_);
          info.header.stamp = ros::Time::now();
          info.header.frame_id = GetPlatform();
          info.type = ff_msgs::Hazard::TYPE_KEEP_OUT;
          info.hazard.header.stamp = it->when;
          info.hazard.point.x = it->pose.position.x;
          info.hazard.point.y = it->pose.position.y;
          info.hazard.point.z = it->pose.position.z;
          return false;
        }
      }
    }
    // Check that we are in a keepin
    if (check_keepins && !point_exists_within_keepin) {
      NODELET_DEBUG_STREAM("KEEPIN violation at time" << it->when.toSec());
      NODELET_DEBUG_STREAM(it->pose.position);
      NODELET_DEBUG_STREAM(zones_);
      info.header.stamp = ros::Time::now();
      info.header.frame_id = GetPlatform();
      info.type = ff_msgs::Hazard::TYPE_KEEP_IN;
      info.hazard.header.stamp = it->when;
      info.hazard.point.x = it->pose.position.x;
      info.hazard.point.y = it->pose.position.y;
      info.hazard.point.z = it->pose.position.z;
      return false;
    }
  }
  return true;
}

// Load the keep in and keepout zones into memory
void MapperNodelet::LoadKeepInOutZones() {
  // Grab the zone directory value from the LUA config
  config_reader::ConfigReader *handle = cfg_.GetConfigReader();
  if (handle == nullptr)
    NODELET_FATAL_STREAM("Cannot read LUA config");
  std::string zonefile;
  if (!handle->GetStr("zone_file", &zonefile))
    NODELET_FATAL_STREAM("Cannot read zone directory from LUA config");
  // Try and open the zone file
  if (!ff_util::Serialization::ReadFile(zonefile, zones_)) {
    NODELET_WARN_STREAM("Cannot open zone file " << zonefile);
  } else {
    UpdateKeepInOutMarkers();
  }
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
  map_keep_in_out_pub_.publish(markers);
  old_markers = markers;
}

}  // namespace mapper
