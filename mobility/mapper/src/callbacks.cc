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
#include <vector>
#include <string>
#include "mapper/pcl_conversions.h"

namespace mapper {

void MapperComponent::PclCallback() {
  // Get messages
  std::string cam_prefix = TOPIC_HARDWARE_PICOFLEXX_PREFIX;
  std::string cam_suffix = TOPIC_HARDWARE_PICOFLEXX_SUFFIX;

  if (use_haz_cam_) {
    std::string cam = TOPIC_HARDWARE_NAME_HAZ_CAM;
    // Get depth message
    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    // rclcpp::wait_for_message<sensor_msgs::PointCloud2>(msg, nh_,
    //                   cam_prefix + cam + cam_suffix, std::chrono::duration<float>(0.5));
    if (msg == NULL) {
      FF_INFO("No point clound message received");
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

    // TODO(@mgouveia): New feature not available in current rolling
    // rclcpp::wait_for_message(msg, nh_
    //                   cam_prefix + cam + cam_suffix, std::chrono::duration<float>(0.5));
    if (msg == NULL) {
      FF_INFO("No point clound message received");
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


void MapperComponent::SegmentCallback(const std::shared_ptr<ff_msgs::Segment> msg) {
  // Check for empty trajectory
  if (msg->segment.size() < 2)
    return;

  // If we shouldn't obstacle check, then don't don't add the segment
  if (!cfg_.Get<bool>("enable_obstacles"))
    return;

  // For timing this callback
  ros::Time t0 = GetTimeNow();

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

  ros::Duration solver_time = GetTimeNow() - t0;
  FF_DEBUG("Time to compute octotraj: %f", solver_time.toSec());
}

// Send diagnostics
void MapperComponent::DiagnosticsCallback() {
  ReconfigureCallback();
}

// Configure callback
bool MapperComponent::ReconfigureCallback() {
  if (state_ != IDLE)
    return false;
  // Turn on mapper
  if (disable_mapper_ && !cfg_.Get<bool>("disable_mapper")) {
    // Timers
    timer_o_.start();
    timer_f_.start();
    // Subscribers
    segment_sub_ = FF_CREATE_SUBSCRIBER(nh_, ff_msgs::Segment, TOPIC_GNC_CTL_SEGMENT, 1,
      std::bind(&MapperComponent::SegmentCallback, this, std::placeholders::_1));
    reset_sub_ = FF_CREATE_SUBSCRIBER(nh_, std_msgs::Empty, TOPIC_GNC_EKF_RESET, 1,
      std::bind(&MapperComponent::ResetCallback, this, std::placeholders::_1));
  // Turn off mapper
  } else if (!disable_mapper_ && cfg_.Get<bool>("disable_mapper")) {
    // Timers
    timer_o_.stop();
    timer_f_.stop();
    // Subscribers
    segment_sub_.reset();
    reset_sub_.reset();
  }
  disable_mapper_ = cfg_.Get<bool>("disable_mapper");

  return true;
}

void MapperComponent::ResetCallback(const std::shared_ptr<std_msgs::Empty> msg) {
  globals_.octomap.ResetMap();
}


}  // namespace mapper
