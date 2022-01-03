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

#ifndef MAPPER_STRUCTS_H_
#define MAPPER_STRUCTS_H_

// PCL specific includes
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Locally defined libraries
#include <mapper/octoclass.h>
#include <mapper/sampled_trajectory.h>

// Astrobee message types
#include <ff_msgs/ControlGoal.h>

// c++ libraries
#include <queue>
#include <string>
#include <mutex>
#include <condition_variable>   // NOLINT

namespace mapper {

struct StampedPcl {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  geometry_msgs::TransformStamped tf_cam2world;
};

enum State {
  IDLE,
  VALIDATING
};

struct GlobalVariables {
  geometry_msgs::TransformStamped tf_cam2world;
  geometry_msgs::TransformStamped tf_perch2world;
  geometry_msgs::TransformStamped tf_body2world;
  octoclass::OctoClass octomap = octoclass::OctoClass(0.05);
  sampled_traj::SampledTrajectory3D sampled_traj;
  std::queue<StampedPcl> pcl_queue;
};

}  // namespace mapper

#endif  // MAPPER_STRUCTS_H_
