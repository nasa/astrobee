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
#include <string>
#include <vector>
#include <algorithm>

namespace mapper {

// Thread for fading memory of the octomap
void MapperNodelet::FadeTask(ros::TimerEvent const& event) {
  if (globals_.octomap.memory_time_ > 0)
      globals_.octomap.FadeMemory(fading_memory_update_rate_);
}

// Sentinel
void MapperNodelet::CollisionCheckTask() {
  // visualization markers
  visualization_msgs::MarkerArray traj_markers, samples_markers;
  visualization_msgs::MarkerArray compressed_samples_markers, collision_markers;

  // pcl variables
  int cloudsize;

  // Get time for when this task started
  ros::Time time_now = ros::Time::now();

  // Copy trajectory into local point cloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud_traj;
  std::vector<octomap::point3d> colliding_nodes;
  traj_markers.markers.clear();
  collision_markers.markers.clear();
  samples_markers.markers.clear();
  compressed_samples_markers.markers.clear();

  point_cloud_traj = globals_.sampled_traj.point_cloud_traj_;
  std::vector<double> time = globals_.sampled_traj.time_;

  // Send visualization markers
  globals_.sampled_traj.TrajVisMarkers(&traj_markers);
  globals_.sampled_traj.SamplesVisMarkers(&samples_markers);
  globals_.sampled_traj.CompressedVisMarkers(&compressed_samples_markers);
  path_marker_pub_.publish(traj_markers);
  path_marker_pub_.publish(samples_markers);
  path_marker_pub_.publish(compressed_samples_markers);

  // Stop execution if there are no points in the trajectory structure
  cloudsize = point_cloud_traj.size();
  if (cloudsize <= 0) {
    visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 0.0, &collision_markers);
    path_marker_pub_.publish(traj_markers);
    path_marker_pub_.publish(collision_markers);
    return;
  }

  // Stop execution if the current time is beyond the final time of the trajectory
  if (time_now.toSec() > time.back()) {
    globals_.sampled_traj.ClearObject();
    globals_.sampled_traj.TrajVisMarkers(&traj_markers);
    visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 0.0, &collision_markers);
    path_marker_pub_.publish(traj_markers);
    path_marker_pub_.publish(collision_markers);
    return;
  }

  // Check if trajectory collides with points in the point-cloud
  double res = globals_.octomap.tree_inflated_.getResolution();
  globals_.octomap.FindCollidingNodesInflated(point_cloud_traj, &colliding_nodes);

  if (colliding_nodes.size() > 0) {
    // Sort collision time (use kdtree for nearest neighbor)
    std::vector<geometry_msgs::PointStamped> sorted_collisions;
    globals_.sampled_traj.SortCollisions(colliding_nodes, &sorted_collisions);

    double collision_time = (sorted_collisions[0].header.stamp - ros::Time::now()).toSec();
    // uint lastCollisionIdx = sorted_collisions.back().header.seq;
    if (collision_time > 0) {
      ROS_WARN("Imminent collision within %.3f seconds!", collision_time);
      // Publish the message
      ff_msgs::Hazard info;
      info.header.stamp = ros::Time::now();
      info.header.frame_id = GetPlatform();
      info.type = ff_msgs::Hazard::TYPE_OBSTACLE;
      info.hazard = sorted_collisions[0];
      hazard_pub_.publish(info);
      // Unlock resources
      globals_.sampled_traj.ClearObject();
    }
  }

  // Draw colliding markers (delete if none)
  visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 1.01*res, &collision_markers);
  path_marker_pub_.publish(traj_markers);
  path_marker_pub_.publish(collision_markers);
  ros::Duration solver_time = ros::Time::now() - time_now;
}

void MapperNodelet::OctomappingTask() {
  pcl::PointCloud< pcl::PointXYZ > pcl_world;

  // If there are no pcl point clounds
  if (globals_.pcl_queue.empty())
    return;

// Update TF values
  try {
    globals_.tf_cam2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_HAZ_CAM), ros::Time(0));
  } catch (tf2::TransformException &ex) {}
  try {
    globals_.tf_perch2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_PERCH_CAM), ros::Time(0));
  } catch (tf2::TransformException &ex) {}
  try {
    globals_.tf_body2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_BODY), ros::Time(0));
  } catch (tf2::TransformException &ex) {}

  // Get time for when this task started
  const ros::Time t0 = ros::Time::now();

  // Get Point Cloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud =
    globals_.pcl_queue.front().cloud;
  const geometry_msgs::TransformStamped tf_cam2world =
    globals_.pcl_queue.front().tf_cam2world;

  // Remove data from queue
  globals_.pcl_queue.pop();

  // Check if a tf message has been received already. If not, return
  if (tf_cam2world.header.stamp.toSec() == 0)
    return;

  // Transform pcl into world frame
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform = msg_conversions::ros_to_eigen_transform(tf_cam2world.transform);

  pcl::transformPointCloud(point_cloud, pcl_world, transform);

  // Save into octomap
  algebra_3d::FrustumPlanes world_frustum;
  globals_.octomap.cam_frustum_.TransformFrustum(transform, &world_frustum);
  globals_.octomap.PclToRayOctomap(pcl_world, tf_cam2world, world_frustum);
  globals_.octomap.tree_.prune();   // prune the tree before visualizing
  globals_.octomap.tree_inflated_.prune();
  // globals_.octomap.tree.writeBinary("simple_tree.bt");

  // Publish visualization markers iff at least one node is subscribed to it
  bool pub_obstacles, pub_free, pub_obstacles_inflated, pub_free_inflated;
  pub_obstacles = (obstacle_marker_pub_.getNumSubscribers() > 0) || (obstacle_cloud_pub_.getNumSubscribers() > 0);
  pub_free = (free_space_marker_pub_.getNumSubscribers() > 0) || (free_space_cloud_pub_.getNumSubscribers() > 0);
  pub_obstacles_inflated = (inflated_obstacle_marker_pub_.getNumSubscribers() > 0)
                        || (inflated_obstacle_cloud_pub_.getNumSubscribers() > 0);
  pub_free_inflated = (inflated_free_space_marker_pub_.getNumSubscribers() > 0)
                   || (inflated_free_space_cloud_pub_.getNumSubscribers() > 0);

  if (pub_obstacles || pub_free) {
    visualization_msgs::MarkerArray obstacle_markers, free_markers;
    sensor_msgs::PointCloud2 obstacle_cloud, free_cloud;
    globals_.octomap.TreeVisMarkers(&obstacle_markers, &free_markers,
                                    &obstacle_cloud,   &free_cloud);
    if (pub_obstacles) {
      obstacle_marker_pub_.publish(obstacle_markers);
      obstacle_cloud_pub_.publish(obstacle_cloud);
    }
    if (pub_free) {
      free_space_marker_pub_.publish(free_markers);
      free_space_cloud_pub_.publish(free_cloud);
    }
  }

  if (pub_obstacles_inflated || pub_free_inflated) {
    visualization_msgs::MarkerArray inflated_obstacle_markers, inflated_free_markers;
    sensor_msgs::PointCloud2 inflated_obstacle_cloud, inflated_free_cloud;
    globals_.octomap.InflatedVisMarkers(&inflated_obstacle_markers, &inflated_free_markers,
                                        &inflated_obstacle_cloud,   &inflated_free_cloud);
    if (pub_obstacles_inflated) {
      inflated_obstacle_marker_pub_.publish(inflated_obstacle_markers);
      inflated_obstacle_cloud_pub_.publish(inflated_obstacle_cloud);
    }
    if (pub_free_inflated) {
      inflated_free_space_marker_pub_.publish(inflated_free_markers);
      inflated_free_space_cloud_pub_.publish(inflated_free_cloud);
    }
  }

  if (cam_frustum_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker frustum_markers;
    globals_.octomap.cam_frustum_.VisualizeFrustum(point_cloud.header.frame_id, &frustum_markers);
    cam_frustum_pub_.publish(frustum_markers);
  }

  // Notify the collision checker to check for collision
  CollisionCheckTask();
  ros::Duration map_time = ros::Time::now() - t0;
}

}  // namespace mapper
