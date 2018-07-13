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

// Update resolution of the map
bool MapperNodelet::UpdateResolution(ff_msgs::SetFloat::Request &req,
                                     ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetResolution(req.data);
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}

// Update map memory time
bool MapperNodelet::UpdateMemoryTime(ff_msgs::SetFloat::Request &req,
                                     ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetMemory(req.data);
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}

bool MapperNodelet::MapInflation(ff_msgs::SetFloat::Request &req,
                                 ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetMapInflation(req.data);
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}

bool MapperNodelet::ResetMap(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.ResetMap();
  mutexes_.octomap.unlock();
  res.success = true;
  res.message = "Map has been reset!";
  return true;
}

// Callback to get the keep in/out zones
bool MapperNodelet::GetZonesCallback(ff_msgs::GetZones::Request &req,
                                     ff_msgs::GetZones::Response &res) {
  res.timestamp = zones_.timestamp;
  res.zones = zones_.zones;
  return true;
}

// Callback to set the keep in/out zones
bool MapperNodelet::SetZonesCallback(ff_msgs::SetZones::Request &req,
                                     ff_msgs::SetZones::Response &res) {
  // Update the zones
  zones_ = req;
  /*
  // Grab the zone directory value from the LUA config
  config_reader::ConfigReader *handle = cfg_.GetConfigReader();
  if (handle == nullptr)
    NODELET_FATAL_STREAM("Cannot read LUA config");
  std::string zonefile;
  if (!handle->GetStr("zone_file", &zonefile))
    NODELET_FATAL_STREAM("Cannot read zone directory from LUA config");
  // Try and open the zone file
  if (!ff_util::Serialization::WriteFile(zonefile, zones_))
    NODELET_WARN_STREAM("Cannot write zone file " << zonefile);
  */
  // Update visualization
  UpdateKeepInOutMarkers();
  // Send result
  res.success = false;
  return true;
}

}  // namespace mapper
