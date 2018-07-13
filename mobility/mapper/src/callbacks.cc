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

void MapperNodelet::PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // Structure to include pcl and its frame
  StampedPcl new_pcl;
  uint max_queue_size = 4;

  // Convert message into pcl type
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl_conversions::FromROSMsg(*msg, &cloud);
  new_pcl.cloud = cloud;

  // Get transform from camera to world
  const std::string pcl_frame = cloud.header.frame_id;
  mutexes_.tf.lock();
  if (!pcl_frame.compare("haz_cam"))
    new_pcl.tf_cam2world = globals_.tf_cam2world;
  else if (!pcl_frame.compare("perch_cam"))
    new_pcl.tf_cam2world = globals_.tf_perch2world;
  mutexes_.tf.unlock();

  // save into global variables
  mutexes_.point_cloud.lock();
  globals_.pcl_queue.push(new_pcl);
  if (globals_.pcl_queue.size() > max_queue_size)
    globals_.pcl_queue.pop();
  mutexes_.point_cloud.unlock();

  // signal octomap thread to process new pcl data
  semaphores_.pcl.notify_one();
}


void MapperNodelet::SegmentCallback(const ff_msgs::Segment::ConstPtr &msg) {
  ros::Time t0 = ros::Time::now();
  while (t0.toSec() == 0)
    t0 = ros::Time::now();

  // Check for empty trajectory
  if (msg->segment.size() < 2)
    return;

  // If we shouldn't obstacle check, then don't don't add the segment
  if (!cfg_.Get<bool>("enable_obstacles"))
    return;

  // transform message into set of polynomials
  polynomials::Trajectory3D poly_trajectories(*msg);

  // sample trajectory at 10hz
  double ts = 0.1;
  sampled_traj::SampledTrajectory3D sampled_traj(ts, poly_trajectories);

  mutexes_.sampled_traj.lock();
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
  mutexes_.sampled_traj.unlock();

  // Notify the collision checker to check for collision
  semaphores_.collision_check.notify_one();

  ros::Duration solver_time = ros::Time::now() - t0;
  ROS_DEBUG("Time to compute octotraj: %f", solver_time.toSec());
}

// Assert a fault - katie's fault code handling will eventually go in here
void MapperNodelet::Complete(int32_t response) {
  NODELET_DEBUG_STREAM("Complete(" << response <<")");
  switch (state_) {
  case VALIDATING: {
    ff_msgs::ValidateResult validate_result;
    validate_result.response = response;
    validate_result.segment = segment_;
    if (response > 0)
      server_v_.SendResult(
        ff_util::FreeFlyerActionState::SUCCESS, validate_result);
    else if (response < 0)
      server_v_.SendResult(
        ff_util::FreeFlyerActionState::ABORTED, validate_result);
    else
      server_v_.SendResult(
        ff_util::FreeFlyerActionState::PREEMPTED, validate_result);
    break;
  }
  case IDLE:
  default:
    NODELET_WARN_STREAM("Complete() called in IDLE state");
    break;
  }
  state_ = IDLE;
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

// A new move  goal arrives asynchronously
void MapperNodelet::GoalCallback(ff_msgs::ValidateGoalConstPtr const& goal) {
  NODELET_DEBUG_STREAM("GoalCallback()");
  switch (state_) {
  case IDLE: {
    // Cover a special case where not enough info
    if (goal->segment.empty())
        return Complete(RESPONSE::UNEXPECTED_EMPTY_SEGMENT);

    // Start the validation
    segment_ = goal->segment;
    state_ = VALIDATING;

    // TODO(asymingt) Mike's QP planner seems to perioidcally supply a segment
    // with a non-stationary endpoint. This squashes any chance that we may
    // continue drifting and/or fail validation. It should be fixed in QP.
    segment_.rbegin()->twist = geometry_msgs::Twist();
    segment_.rbegin()->accel = geometry_msgs::Twist();

    // Do the basic set of checks first
    uint32_t mask = 0;
    if (cfg_.Get<bool>("validate_min_frequency"))
        mask |= ff_util::CHECK_MINIMUM_FREQUENCY;
    if (cfg_.Get<bool>("validate_stationary_endpoint"))
        mask |= ff_util::CHECK_STATIONARY_ENDPOINT;
    if (cfg_.Get<bool>("validate_first_timestamp"))
        mask |= ff_util::CHECK_FIRST_TIMESTAMP_IN_PAST;
    if (cfg_.Get<bool>("validate_minimum_num_setpoints"))
        mask |= ff_util::CHECK_MINIMUM_NUM_SETPOINTS;
    if (cfg_.Get<bool>("validate_limit_vel"))
        mask |= ff_util::CHECK_LIMITS_VEL;
    if (cfg_.Get<bool>("validate_limit_accel"))
        mask |= ff_util::CHECK_LIMITS_ACCEL;
    if (cfg_.Get<bool>("validate_limit_omega"))
        mask |= ff_util::CHECK_LIMITS_OMEGA;
    if (cfg_.Get<bool>("validate_limit_alpha"))
        mask |= ff_util::CHECK_LIMITS_ALPHA;

    // Do the checks based on the current state
    ff_util::SegmentResult r = ff_util::FlightUtil::Check(
      static_cast<ff_util::SegmentCheckMask>(mask),
      segment_, goal->flight_mode, goal->faceforward);
    if (r != ff_util::SUCCESS) {
      // Print the result to th elog
      NODELET_DEBUG_STREAM(*goal);
      NODELET_WARN_STREAM("Validate failed: "
        << ff_util::FlightUtil::GetDescription(r));
      // Do something based on the result
      switch (r) {
      case ff_util::ERROR_MINIMUM_FREQUENCY:
        return Complete(RESPONSE::MINIMUM_FREQUENCY_NOT_MET);
      case ff_util::ERROR_STATIONARY_ENDPOINT:
        return Complete(RESPONSE::ENDPOINT_NOT_STATIONARY);
      case ff_util::ERROR_MINIMUM_NUM_SETPOINTS:
        return Complete(RESPONSE::NOT_ENOUGH_SETPOINTS);
      case ff_util::ERROR_TIME_RUNS_BACKWARDS:
        return Complete(RESPONSE::TIME_RUNS_BACKWARDS);
      case ff_util::ERROR_LIMITS_VEL:
        return Complete(RESPONSE::EXCEEDS_LIMITS_VEL);
      case ff_util::ERROR_LIMITS_ACCEL:
        return Complete(RESPONSE::EXCEEDS_LIMITS_ACCEL);
      case ff_util::ERROR_LIMITS_OMEGA:
        return Complete(RESPONSE::EXCEEDS_LIMITS_OMEGA);
      case ff_util::ERROR_LIMITS_ALPHA:
        return Complete(RESPONSE::EXCEEDS_LIMITS_ALPHA);
      default:
        NODELET_WARN_STREAM("Validate failed for unknown reason");
        break;
      }
    }

    // Dummy data structure for checking collisions
    ff_msgs::Hazard info;
    if (!CheckZones(segment_, cfg_.Get<bool>("validate_keepins"),
      cfg_.Get<bool>("validate_keepouts"), info)) {
        return Complete(RESPONSE::FAILS_ZONE_CHECK);
    }

    // If we get here the trajectory is good
    return Complete(RESPONSE::SUCCESS);
  }
  case VALIDATING:
  default:
      NODELET_WARN_STREAM("GoalCallback() called in IDLE state");
      break;
  }
}

void MapperNodelet::PreemptCallback(void) {
  NODELET_DEBUG_STREAM("PreemptCallback()");
  return Complete(RESPONSE::PREEMPTED);
}

void MapperNodelet::CancelCallback(void) {
  NODELET_DEBUG_STREAM("CancelCallback()");
  return Complete(RESPONSE::CANCELLED);
}

void MapperNodelet::ResetCallback(std_msgs::EmptyConstPtr const& msg) {
  mutexes_.octomap.lock();
  globals_.octomap.ResetMap();
  mutexes_.octomap.unlock();
}


}  // namespace mapper
