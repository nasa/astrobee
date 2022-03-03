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


// Process zone
  void Validator::ProcessZone(std::vector<signed char> &map, int type, char cell_value, bool surface) {
    Vec3f zmin, zmax;
    for (auto &zone : zones_.zones) {
      if (zone.type == type) {
        Vec3f tmp = Vec3f::Zero();
        zmin << std::min(zone.min.x, zone.max.x) + map_res_ * EPS,
                std::min(zone.min.y, zone.max.y) + map_res_ * EPS,
                std::min(zone.min.z, zone.max.z) + map_res_ * EPS;
        zmax << std::max(zone.min.x, zone.max.x) - map_res_ * EPS,
                std::max(zone.min.y, zone.max.y) - map_res_ * EPS,
                std::max(zone.min.z, zone.max.z) - map_res_ * EPS;

        if (surface) {
          // If it's a keepin zone, the border should be added outside
          // The second term is used to assure the selected voxel is the inner one when specifying boundaries
          double gap = (zone.type == ff_msgs::Zone::KEEPIN) ? map_res_  : 0;
          zmin = zmin.array() - gap;
          zmax = zmax.array() + gap;
          for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;
            for (auto zx = zmin(j); zx <= zmax(j); zx += map_res_) {
              for (auto zy = zmin(k); zy <= zmax(k); zy += map_res_) {
                // Attribute the desired value to the surface around zone
                tmp(j) = zx;
                tmp(k) = zy;
                tmp(i) = zmin(i);
                map[jps_map_util_->getIndex(jps_map_util_->floatToInt(tmp))] =
                    cell_value;
                tmp(i) = zmax(i);
                map[jps_map_util_->getIndex(jps_map_util_->floatToInt(tmp))] =
                    cell_value;
              }
            }
          }
        // Attribute the desired value to the volume of the zone
        } else {
          for (auto zx = zmin(0); zx <= zmax(0); zx += map_res_) {
            for (auto zy = zmin(1); zy <= zmax(1); zy += map_res_) {
              for (auto zz = zmin(2); zz <= zmax(2); zz += map_res_) {
                tmp(0) = zx;
                tmp(1) = zy;
                tmp(2) = zz;
                map[jps_map_util_->getIndex(jps_map_util_->floatToInt(tmp))] =
                    cell_value;
              }
            }
          }
        }
      }
    }
  }

// Get Zones Map
bool Validator::GetZonesMap() {
    ff_msgs::GetFloat srv;
    if (get_resolution_.call(srv)) {
      map_res_ = srv.response.data;
    }


    // Check min and max of keepin zones to use as map boundries
    Vec3f min, max, zmin, zmax;
    min << std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                              std::numeric_limits<float>::max();
    max << std::numeric_limits<float>::min(), std::numeric_limits<float>::min(),
                                              std::numeric_limits<float>::min();
    uint num_keepin = 0;
    for (auto &zone : zones_.zones) {
      if (zone.type == ff_msgs::Zone::KEEPIN) {
        zmin << zone.min.x, zone.min.y, zone.min.z;
        zmax << zone.max.x, zone.max.y, zone.max.z;
        for (int i = 0; i < 3; i++) {
          min(i) = std::min(min(i), zmin(i));
          min(i) = std::min(min(i), zmax(i));
          max(i) = std::max(max(i), zmin(i));
          max(i) = std::max(max(i), zmax(i));
        }
        num_keepin++;
      }
    }
    if (num_keepin == 0) {
      ROS_ERROR("Zero keepin zones!! Plan failed");
      return false;
    }

    // Based on zones boundries and obstacle map resolution, specify dimensions
    min = Vec3f(std::floor(min(0) / map_res_) * map_res_,
                std::floor(min(1) / map_res_) * map_res_,
                std::floor(min(2) / map_res_) * map_res_) - Vec3f::Ones() * map_res_ * 2.0;
    max = Vec3f(std::ceil(max(0) / map_res_) * map_res_,
                std::ceil(max(1) / map_res_) * map_res_,
                std::ceil(max(2) / map_res_) * map_res_) + Vec3f::Ones() * map_res_ * 2.0;


    Vec3f origin = min;
    Vec3f dimf = (max - min) / map_res_;
    Vec3i dim(std::ceil(dimf(0)), std::ceil(dimf(1)), std::ceil(dimf(2)));
    int num_cell = dim(0) * dim(1) * dim(2);

    // Keepin/Keepout zones:
    // To reduce computational load, only the contour of the keepin/keepout
    // zones is defined as occupied, to minimize the number of points that
    // are inflated
    // 0) The voxel map starts with all voxels set to unknown
    // Declare voxel map
    std::vector<signed char> map(num_cell, val_unknown_);
    jps_map_util_.reset(new JPS::VoxelMapUtil());
    jps_map_util_->setMap(origin, dim, map, map_res_);

    // 1) Keepin Zones add contour as occupied
    ProcessZone(map, ff_msgs::Zone::KEEPIN, val_occ_, true);

    // 2) We set the interior of the keepin zones to free, this corrects the
    // case where keepin zones are connected and a contourn was put between them
    ProcessZone(map, ff_msgs::Zone::KEEPIN, val_free_, false);

    // 4) Add keepout zones
    ProcessZone(map, ff_msgs::Zone::KEEPOUT, val_occ_, true);

    // set voxel map using keepin/keepout zones
    jps_map_util_->setMap(origin, dim, map, map_res_);

    // 6) Inflate using the robot radius and the collision distance
    double inflation = map_res_;
    if (get_map_inflation_.call(srv)) {
      inflation = srv.response.data;
    }
    jps_map_util_->dilate(inflation, inflation);     // sets dilating radius
    jps_map_util_->dilating();                       // this dilates the entire map
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
  ff_util::Segment::iterator it;
  Vec3f tmp = Vec3f::Zero();
  // Check if the robot is going outside a keepin zone
  tmp << seg.end()->pose.position.x, seg.end()->pose.position.y, seg.end()->pose.position.z;
  if (jps_map_util_->isOutSide(jps_map_util_->floatToInt(tmp)))
      return VIOLATES_KEEP_IN;

  for (it = seg.begin(); it != seg.end(); it++) {
    tmp << it->pose.position.x, it->pose.position.y, it->pose.position.z;
    if (jps_map_util_->isOccupied(jps_map_util_->floatToInt(tmp)))
      return VIOLATES_KEEP_OUT;
    else if (jps_map_util_->isUnKnown(jps_map_util_->floatToInt(tmp)) ||
              jps_map_util_->isOutSide(jps_map_util_->floatToInt(tmp)))
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
  get_zones_map_srv_ = nh->advertiseService(SERVICE_MOBILITY_GET_ZONES_MAP,
    &Validator::GetZonesMapCallback, this);
  get_resolution_ = nh->serviceClient<ff_msgs::GetFloat>(
    SERVICE_MOBILITY_GET_MAP_RESOLUTION);
  get_map_inflation_ = nh->serviceClient<ff_msgs::GetFloat>(
    SERVICE_MOBILITY_GET_MAP_INFLATION);

  // Wait for services to exist
  if (!get_resolution_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Mapper service to get map resolution not working");
  }
  if (!get_map_inflation_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Mapper service to get map inflation not working");
  }

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
  // Update map
  GetZonesMap();
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

// Callback to get the keep in/out zones
bool Validator::GetZonesMapCallback(
  ff_msgs::GetOccupancyMap::Request &req, ff_msgs::GetOccupancyMap::Response &res) {
  res.timestamp = zones_.timestamp;
  std::vector<signed char> map = jps_map_util_->getMap();
  res.map.resize(map.size());
  res.map = map;

  Vec3f origin = jps_map_util_->getOrigin();
  res.origin.x = origin[0];
  res.origin.y = origin[1];
  res.origin.z = origin[2];

  Vec3i dim = jps_map_util_->getDim();
  res.dim.x = dim[0];
  res.dim.y = dim[1];
  res.dim.z = dim[2];

  res.resolution = jps_map_util_->getRes();
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
  // Update map
  GetZonesMap();
  // Send result
  res.success = true;
  return true;
}

}  // namespace choreographer
