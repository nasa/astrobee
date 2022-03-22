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
#include <vector>
#include <string>
#include "mapper/pcl_conversions.h"

namespace mapper {

void MapperNodelet::PclCallback(ros::TimerEvent const& event) {
  // Get messages
  std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
  std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;

  if (use_haz_cam_) {
    std::string cam = TOPIC_HARDWARE_NAME_HAZ_CAM;
    // Get depth message
    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                      cam_prefix + cam + cam_suffix, ros::Duration(0.5));
    if (msg == NULL) {
      ROS_INFO("No point clound message received");
    } else {
      // Structure to include pcl and its frame
      StampedPcl new_pcl;

      // Convert message into pcl type
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      new_pcl.cloud = cloud;
      new_pcl.tf_cam2world = globals_.tf_cam2world;

      // save into global variables
      globals_.pcl_queue.push(new_pcl);
    }
  }
  if (use_perch_cam_) {
    std::string cam = TOPIC_HARDWARE_NAME_PERCH_CAM;
    // Get depth message
    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;

    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                      cam_prefix + cam + cam_suffix, ros::Duration(0.5));
    if (msg == NULL) {
      ROS_INFO("No point clound message received");
    } else {
    // Structure to include pcl and its frame
    StampedPcl new_pcl;

    // Convert message into pcl type
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    new_pcl.cloud = cloud;
    new_pcl.tf_cam2world = globals_.tf_perch2world;

    // save into global variables
    globals_.pcl_queue.push(new_pcl);
    }
  }

  OctomappingTask();
}


void MapperNodelet::SegmentCallback(const ff_msgs::Segment::ConstPtr &msg) {
  // Check for empty trajectory
  if (msg->segment.size() < 2)
    return;

  // If we shouldn't obstacle check, then don't don't add the segment
  if (!cfg_.Get<bool>("enable_obstacles"))
    return;

  // For timing this callback
  ros::Time t0 = ros::Time::now();

  // transform message into set of polynomials
  polynomials::Trajectory3D poly_trajectories(*msg);

  // sample trajectory at 10hz
  double ts = 0.1;
  sampled_traj::SampledTrajectory3D sampled_traj(ts, poly_trajectories);

  globals_.sampled_traj.pos_ = sampled_traj.pos_;
  globals_.sampled_traj.time_ = sampled_traj.time_;
  globals_.sampled_traj.n_points_ = sampled_traj.n_points_;

  // compress trajectory into points with max deviation of 1cm from original trajectory
  globals_.sampled_traj.CompressSamples();

  //  Transform compressed trajectory into a set of pixels in octomap
  //  Octomap insertion avoids repeated points
  globals_.sampled_traj.thick_traj_.clear();
  for (int i = 0; i < globals_.sampled_traj.n_compressed_points_-1; i++)
    globals_.sampled_traj.ThickBresenham(
      globals_.sampled_traj.compressed_pos_[i],
      globals_.sampled_traj.compressed_pos_[i+1]);

  // populate trajectory node centers in a point cloud
  globals_.sampled_traj.ThickTrajToPcl();

  // populate kdtree for finding nearest neighbor w.r.t. collisions
  globals_.sampled_traj.CreateKdTree();

  // Notify the collision checker to check for collision
  CollisionCheckTask();

  ros::Duration solver_time = ros::Time::now() - t0;
  ROS_DEBUG("Time to compute octotraj: %f", solver_time.toSec());
}

// Send diagnostics
void MapperNodelet::DiagnosticsCallback(const ros::TimerEvent &event) {
  SendDiagnostics(cfg_.Dump());
}

// Configure callback
bool MapperNodelet::ReconfigureCallback(dynamic_reconfigure::Config &config) {
  if (state_ != IDLE)
    return false;
  cfg_.Reconfigure(config);
  return true;
}

void MapperNodelet::ResetCallback(std_msgs::EmptyConstPtr const& msg) {
  globals_.octomap.ResetMap();
}


}  // namespace mapper
