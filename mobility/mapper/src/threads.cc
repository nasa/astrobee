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
  mutexes_.octomap.lock();
  if (globals_.octomap.memory_time_ > 0)
      globals_.octomap.FadeMemory(fading_memory_update_rate_);
  mutexes_.octomap.unlock();
}

// Thread for constantly updating the tfTree values
void MapperNodelet::HazTfTask(ros::TimerEvent const& event) {
  mutexes_.tf.lock();
  try {
    globals_.tf_cam2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_HAZ_CAM), ros::Time(0));
  } catch (tf2::TransformException &ex) {}
  mutexes_.tf.unlock();
}

// Thread for constantly updating the tfTree values
void MapperNodelet::PerchTfTask(ros::TimerEvent const& event) {
  mutexes_.tf.lock();
  try {
    globals_.tf_perch2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_PERCH_CAM), ros::Time(0));
  } catch (tf2::TransformException &ex) {}
  mutexes_.tf.unlock();
}

// Thread for updating the tfTree values
void MapperNodelet::BodyTfTask(ros::TimerEvent const& event) {
  mutexes_.tf.lock();
  try {
    globals_.tf_body2world = buffer_.lookupTransform(FRAME_NAME_WORLD,
      GetTransform(FRAME_NAME_BODY), ros::Time(0));
  } catch (tf2::TransformException &ex) {}
  mutexes_.tf.unlock();
}

// Sentinel
void MapperNodelet::CollisionCheckTask() {
  ROS_DEBUG("collisionCheck Thread started!");

  // visualization markers
  visualization_msgs::MarkerArray traj_markers, samples_markers;
  visualization_msgs::MarkerArray compressed_samples_markers, collision_markers;

  // pcl variables
  int cloudsize;

  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  while (!killed_) {
    // Wait until there is a new trajectory or a new update on the map
    semaphores_.collision_check.wait(lck);
    if (killed_)
      return;

    // Get time for when this task started
    ros::Time time_now = ros::Time::now();

    // Copy trajectory into local point cloud
    pcl::PointCloud<pcl::PointXYZ> point_cloud_traj;
    std::vector<octomap::point3d> colliding_nodes;
    traj_markers.markers.clear();
    collision_markers.markers.clear();
    samples_markers.markers.clear();
    compressed_samples_markers.markers.clear();

    mutexes_.sampled_traj.lock();
    point_cloud_traj = globals_.sampled_traj.point_cloud_traj_;
    std::vector<double> time = globals_.sampled_traj.time_;

    // Send visualization markers
    globals_.sampled_traj.TrajVisMarkers(&traj_markers);
    globals_.sampled_traj.SamplesVisMarkers(&samples_markers);
    globals_.sampled_traj.CompressedVisMarkers(&compressed_samples_markers);
    path_marker_pub_.publish(traj_markers);
    path_marker_pub_.publish(samples_markers);
    path_marker_pub_.publish(compressed_samples_markers);
    mutexes_.sampled_traj.unlock();

    // Stop execution if there are no points in the trajectory structure
    cloudsize = point_cloud_traj.size();
    if (cloudsize <= 0) {
      visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 0.0, &collision_markers);
      path_marker_pub_.publish(traj_markers);
      path_marker_pub_.publish(collision_markers);
      continue;
    }

    // Stop execution if the current time is beyond the final time of the trajectory
    if (time_now.toSec() > time.back()) {
      mutexes_.sampled_traj.lock();
          globals_.sampled_traj.ClearObject();
          globals_.sampled_traj.TrajVisMarkers(&traj_markers);
      mutexes_.sampled_traj.unlock();
      visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 0.0, &collision_markers);
      path_marker_pub_.publish(traj_markers);
      path_marker_pub_.publish(collision_markers);
      continue;
    }

    // Check if trajectory collides with points in the point-cloud
    mutexes_.octomap.lock();
    double res = globals_.octomap.tree_inflated_.getResolution();
    globals_.octomap.FindCollidingNodesInflated(point_cloud_traj, &colliding_nodes);
    mutexes_.octomap.unlock();

    if (colliding_nodes.size() > 0) {
      // Sort collision time (use kdtree for nearest neighbor)
      std::vector<geometry_msgs::PointStamped> sorted_collisions;
      mutexes_.sampled_traj.lock();
      globals_.sampled_traj.SortCollisions(colliding_nodes, &sorted_collisions);
      mutexes_.sampled_traj.unlock();

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
        mutexes_.sampled_traj.lock();
        globals_.sampled_traj.ClearObject();
        mutexes_.sampled_traj.unlock();
      }
    }

    // Draw colliding markers (delete if none)
    visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 1.01*res, &collision_markers);
    path_marker_pub_.publish(traj_markers);
    path_marker_pub_.publish(collision_markers);
    ros::Duration solver_time = ros::Time::now() - time_now;
  }

  ROS_DEBUG("Exiting collisionCheck Thread...");
}

void MapperNodelet::OctomappingTask() {
  ROS_DEBUG("OctomappingTask Thread started!");
  geometry_msgs::TransformStamped tf_cam2world;
  pcl::PointCloud< pcl::PointXYZ > pcl_world;

  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  while (!killed_) {
    // Take care of the special case where the notify_one was sent during the
    // processing of the previous point cloud. The signal that this happened
    // is a non-empty point cloud queue. In this case we pass straight through.
    if (globals_.pcl_queue.empty())
      semaphores_.pcl.wait(lck);
    if (killed_)
      return;
    mutexes_.point_cloud.lock();

    // Get time for when this task started
    const ros::Time t0 = ros::Time::now();

    // Get Point Cloud
    pcl::PointCloud<pcl::PointXYZ> point_cloud =
      globals_.pcl_queue.front().cloud;
    const geometry_msgs::TransformStamped tf_cam2world =
      globals_.pcl_queue.front().tf_cam2world;

    // Remove data from queue
    globals_.pcl_queue.pop();
    mutexes_.point_cloud.unlock();

    // Check if a tf message has been received already. If not, return
    if (tf_cam2world.header.stamp.toSec() == 0)
      continue;

    // Transform pcl into world frame
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() <<  tf_cam2world.transform.translation.x,
                                tf_cam2world.transform.translation.y,
                                tf_cam2world.transform.translation.z;
    transform.rotate(Eigen::Quaterniond(
      tf_cam2world.transform.rotation.w,
      tf_cam2world.transform.rotation.x,
      tf_cam2world.transform.rotation.y,
      tf_cam2world.transform.rotation.z));
    pcl::transformPointCloud(point_cloud, pcl_world, transform);

    // Save into octomap
    algebra_3d::FrustumPlanes world_frustum;
    mutexes_.octomap.lock();
    globals_.octomap.cam_frustum_.TransformFrustum(transform, &world_frustum);
    globals_.octomap.PclToRayOctomap(pcl_world, tf_cam2world, world_frustum);
    globals_.octomap.tree_.prune();   // prune the tree before visualizing
    globals_.octomap.tree_inflated_.prune();
    // globals_.octomap.tree.writeBinary("simple_tree.bt");
    mutexes_.octomap.unlock();

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
      mutexes_.octomap.lock();
      globals_.octomap.TreeVisMarkers(&obstacle_markers, &free_markers,
                                      &obstacle_cloud,   &free_cloud);
      mutexes_.octomap.unlock();
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
      mutexes_.octomap.lock();
      globals_.octomap.InflatedVisMarkers(&inflated_obstacle_markers, &inflated_free_markers,
                                          &inflated_obstacle_cloud,   &inflated_free_cloud);
      mutexes_.octomap.unlock();
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
      mutexes_.octomap.lock();
      globals_.octomap.cam_frustum_.VisualizeFrustum(point_cloud.header.frame_id, &frustum_markers);
      mutexes_.octomap.unlock();
      cam_frustum_pub_.publish(frustum_markers);
    }

    // Notify the collision checker to check for collision
    semaphores_.collision_check.notify_one();
    ros::Duration map_time = ros::Time::now() - t0;
  }
  ROS_DEBUG("Exiting OctomappingTask Thread...");
}

}  // namespace mapper
