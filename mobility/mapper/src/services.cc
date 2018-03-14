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
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetResolution(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

// Update map memory time
bool MapperNodelet::UpdateMemoryTime(ff_msgs::SetFloat::Request &req,
                                     ff_msgs::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMemory(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperNodelet::MapInflation(ff_msgs::SetFloat::Request &req,
                                 ff_msgs::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMapInflation(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperNodelet::ResetMap(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.ResetMap();
    pthread_mutex_unlock(&mutexes_.octomap);

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
                                     ff_msgs::SetZones::Response &res) {  //NOLINT
if (req.timestamp >= zones_.timestamp) {
  zones_ = req;
  fs::path zone_file(std::to_string(zones_.timestamp.sec) + ".bin");
  if (!ff_util::Serialization::WriteFile(
    (zone_dir_ / zone_file).native(), zones_)) return false;
  UpdateKeepInOutMarkers();
  return true;
}
return false;
}

}  // namespace mapper
