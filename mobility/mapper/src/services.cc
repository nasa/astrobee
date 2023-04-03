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

#include <mapper/mapper_component.h>
#include <limits>
#include <vector>

namespace mapper {

// Update resolution of the map
bool MapperComponent::SetResolution(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                                     std::shared_ptr<ff_msgs::SetFloat::Response> res) {
  globals_.octomap.SetResolution(req->data);
  res->success = true;
  return true;
}
// Update resolution of the map
bool MapperComponent::GetResolution(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                                     std::shared_ptr<ff_msgs::GetFloat::Response> res) {
  res->data = globals_.octomap.GetResolution();
  res->success = true;
  return true;
}

// Update map memory time
bool MapperComponent::SetMemoryTime(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                                     std::shared_ptr<ff_msgs::SetFloat::Response> res) {
  globals_.octomap.SetMemoryTime(req->data);
  res->success = true;
  return true;
}
// Update map memory time
bool MapperComponent::GetMemoryTime(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                                     std::shared_ptr<ff_msgs::GetFloat::Response> res) {
  res->data = globals_.octomap.GetMemoryTime();
  res->success = true;
  return true;
}

bool MapperComponent::SetCollisionDistance(const std::shared_ptr<ff_msgs::SetFloat::Request> req,
                                 std::shared_ptr<ff_msgs::SetFloat::Response> res) {
  globals_.octomap.SetMapInflation(req->data + cfg_.Get<double>("robot_radius"));
  res->success = true;
  return true;
}
bool MapperComponent::GetMapInflation(const std::shared_ptr<ff_msgs::GetFloat::Request> req,
                                 std::shared_ptr<ff_msgs::GetFloat::Response> res) {
  res->data = globals_.octomap.GetMapInflation();
  res->success = true;
  return true;
}

bool MapperComponent::ResetMap(const std::shared_ptr<std_srvs::Trigger::Request> req,
                             std::shared_ptr<std_srvs::Trigger::Response> res) {
  globals_.octomap.ResetMap();
  res->success = true;
  res->message = "Map has been reset!";
  return true;
}

bool MapperComponent::GetFreeMapCallback(const std::shared_ptr<ff_msgs::GetMap::Request> req,
                                       std::shared_ptr<ff_msgs::GetMap::Response> res) {
  visualization_msgs::MarkerArray om, fm;
  sensor_msgs::PointCloud2 oc, fc;

  globals_.octomap.InflatedVisMarkers(GetTimeNow(), &om, &fm, &oc, &fc);

  res->points = fc;
  res->resolution = globals_.octomap.GetResolution();
  res->free = true;

  return true;
}
bool MapperComponent::GetObstacleMapCallback(const std::shared_ptr<ff_msgs::GetMap::Request> req,
                                       std::shared_ptr<ff_msgs::GetMap::Response> res) {
  visualization_msgs::MarkerArray om, fm;
  sensor_msgs::PointCloud2 oc, fc;

  globals_.octomap.InflatedVisMarkers(GetTimeNow(), &om, &fm, &oc, &fc);

  res->points = oc;
  res->resolution = globals_.octomap.GetResolution();
  res->free = false;

  return true;
}

}  // namespace mapper
