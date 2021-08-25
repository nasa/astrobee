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
bool MapperNodelet::SetResolution(ff_msgs::SetFloat::Request &req,
                                     ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetResolution(req.data);
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}
// Update resolution of the map
bool MapperNodelet::GetResolution(ff_msgs::GetFloat::Request &req,
                                     ff_msgs::GetFloat::Response &res) {
  res.data = globals_.octomap.GetResolution();
  res.success = true;
  return true;
}

// Update map memory time
bool MapperNodelet::SetMemoryTime(ff_msgs::SetFloat::Request &req,
                                     ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetMemoryTime(req.data);
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}
// Update map memory time
bool MapperNodelet::GetMemoryTime(ff_msgs::GetFloat::Request &req,
                                     ff_msgs::GetFloat::Response &res) {
  res.data = globals_.octomap.GetMemoryTime();
  res.success = true;
  return true;
}

bool MapperNodelet::SetCollisionDistance(ff_msgs::SetFloat::Request &req,
                                 ff_msgs::SetFloat::Response &res) {
  mutexes_.octomap.lock();
  globals_.octomap.SetMapInflation(req.data + cfg_.Get<double>("robot_radius"));
  mutexes_.octomap.unlock();
  res.success = true;
  return true;
}
bool MapperNodelet::GetCollisionDistance(ff_msgs::GetFloat::Request &req,
                                 ff_msgs::GetFloat::Response &res) {
  res.data = globals_.octomap.GetMapInflation();
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

bool MapperNodelet::GetFreeMapCallback(ff_msgs::GetMap::Request &req,
                                       ff_msgs::GetMap::Response &res) {
  visualization_msgs::MarkerArray om, fm;
  sensor_msgs::PointCloud2 oc, fc;

  mutexes_.octomap.lock();
  globals_.octomap.InflatedVisMarkers(&om, &fm, &oc, &fc);
  mutexes_.octomap.unlock();

  res.points = fc;
  res.resolution = globals_.octomap.GetResolution();
  res.free = true;

  return true;
}
bool MapperNodelet::GetObstacleMapCallback(ff_msgs::GetMap::Request &req,
                                       ff_msgs::GetMap::Response &res) {
  visualization_msgs::MarkerArray om, fm;
  sensor_msgs::PointCloud2 oc, fc;

  mutexes_.octomap.lock();
  globals_.octomap.InflatedVisMarkers(&om, &fm, &oc, &fc);
  mutexes_.octomap.unlock();

  res.points = oc;
  res.resolution = globals_.octomap.GetResolution();
  res.free = false;

  return true;
}

}  // namespace mapper
