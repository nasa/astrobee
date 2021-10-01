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

#ifndef MAPPER_OCTOCLASS_H_
#define MAPPER_OCTOCLASS_H_

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>
#include "mapper/indexed_octree_key.h"
#include "mapper/linear_algebra.h"

namespace octoclass {

// 3D occupancy grid
class OctoClass{
 public:
  octomap::OcTree tree_ = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
  octomap::OcTree tree_inflated_ = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
  double memory_time_;  // Fading memory of the tree in seconds
  algebra_3d::FrustumPlanes cam_frustum_;

  // Constructor
  explicit OctoClass(const double resolution_in);  // Resolution in meters
  OctoClass();

  // Mapping methods
  void SetMemoryTime(const double memory);  // Fading memory time
  double GetMemoryTime();                   // Returns fading memory time
  void SetMaxRange(const double max_range);  // Max range for mapping
  void SetMinRange(const double min_range);  // Min range for mapping
  inline double GetResolution() const {return resolution_;}  // Returns resolution
  void SetResolution(const double resolution_in);     // Resolution of the octomap
  double GetResolution();                             // Returns resolution of the octomap
  void ResetMap();                                    // Reset the octomap structure
  void SetMapInflation(const double inflate_radius);  // Set the inflation radius
  double GetMapInflation();                           // Returns the inflation radius
  void SetCamFrustum(const double fov,
                     const double aspect_ratio);
  void SetOccupancyThreshold(const double occupancy_threshold);
  void SetHitMissProbabilities(const double probability_hit,
                               const double probability_miss);
  void SetClampingThresholds(const double clamping_threshold_min,
                             const double clamping_threshold_max);
  // DEPRECATED: turns leaf into voxel representation
  // octomap::point3d_list Voxelize(const octomap::OcTree::leaf_iterator &leaf);
  // DEPRECATED: Convert from octomap to pointcloud2
  // void PointsOctomapToPointCloud2(const octomap::point3d_list& points,
  //                                 sensor_msgs::PointCloud2* cloud);
  void PclToRayOctomap(const pcl::PointCloud< pcl::PointXYZ > &cloud,
                        const geometry_msgs::TransformStamped &tf_cam2world,
                        const algebra_3d::FrustumPlanes &frustum);    // Map obstacles and free area
  void ComputeUpdate(const octomap::KeySet &occ_inflated,  // Inflated endpoints
                     const octomap::KeySet &occ_slim,      // Non-inflated endpoints
                     const octomap::point3d& origin,
                     const double &maxrange,
                     octomap::KeySet *occ_slim_in_range,
                     octomap::KeySet *free_slim,
                     octomap::KeySet *free_inflated);  // Raycasting method for inflated maps
  void FadeMemory(const double &rate);  // Run fading memory method
  // DEPRECATED: it was used to inflate the whole map (too expensive)
  // void InflateObstacles(const double &thickness);
  // DEPRECATED: Returns all colliding nodes in the pcl
  // void FindCollidingNodesTree(const pcl::PointCloud< pcl::PointXYZ > &point_cloud,
  //                             std::vector<octomap::point3d> *colliding_nodes);
  // Returns points from the pcl that collides with inflated tree
  void FindCollidingNodesInflated(const pcl::PointCloud< pcl::PointXYZ > &point_cloud,
                                  std::vector<octomap::point3d> *colliding_nodes);

  // Visualization methods
  void TreeVisMarkers(visualization_msgs::MarkerArray *obstacles,
                      visualization_msgs::MarkerArray *free,
                      sensor_msgs::PointCloud2* obstacles_cloud,
                      sensor_msgs::PointCloud2* free_cloud);
  void InflatedVisMarkers(visualization_msgs::MarkerArray *obstacles,
                          visualization_msgs::MarkerArray *free,
                          sensor_msgs::PointCloud2* obstacles_cloud,
                          sensor_msgs::PointCloud2* free_cloud);
  // void freeVisMarkers(visualization_msgs::MarkerArray* marker_array);
  // void inflatedFreeVisMarkers(visualization_msgs::MarkerArray* marker_array);

  // Useful methods
  // checkOccupancy functions: Returns -1 if node is unknown, 0 if its free and 1 if its occupied
  int CheckOccupancy(const octomap::point3d &p);  // Point collision
  int CheckOccupancy(const pcl::PointXYZ &p);  // Point collision
  int CheckOccupancy(const Eigen::Vector3d &p);  // Point collision
  int CheckOccupancy(const octomap::point3d &p1,
                     const octomap::point3d &p2);  // line collision
  int CheckOccupancy(const Eigen::Vector3d &p1,
                     const Eigen::Vector3d &p2);  // line collision

  // checkCollision functions: Returns 0 if its free and 1 if its occupied or unknown
  bool CheckCollision(const octomap::point3d &p);  // Point collision
  bool CheckCollision(const Eigen::Vector3d &p);  // Point collision
  bool CheckCollision(const octomap::point3d &p1,
                      const octomap::point3d &p2);  // line collision
  bool CheckCollision(const Eigen::Vector3d &p1,
                      const Eigen::Vector3d &p2);  // line collision

  // Calculate the volume of free nodes in the bounding box
  void BBXFreeVolume(const Eigen::Vector3d &box_min,
                     const Eigen::Vector3d &box_max,
                     double *volume);
  // Calculate the volume of obstacles in the bounding box
  void BBXOccVolume(const Eigen::Vector3d &box_min,
                    const Eigen::Vector3d &box_max,
                    double *volume);

  // Return all free nodes within a bounding box
  void BBXFreeNodes(const Eigen::Vector3d &box_min,
                    const Eigen::Vector3d &box_max,
                    std::vector<octomap::OcTreeKey> *node_keys,
                    std::vector<double> *node_sizes);
  // Return all free nodes within a bounding box and assign them indexes
  void BBXFreeNodes(const Eigen::Vector3d &box_min,
                    const Eigen::Vector3d &box_max,
                    IndexedKeySet *indexed_node_keys,
                    std::vector<double> *node_sizes);

  // Find all immediate neighbors of a node
  void GetNodeNeighbors(const octomap::OcTreeKey &node_key,
                        const double &node_size,
                        std::vector<octomap::OcTreeKey> *neighbor_keys);  // Return all neighbors for a given node
  double GetNodeSize(const octomap::OcTreeKey &key);  // Returns size of node. Returns zero if node doesn't exist
  void PrintQueryInfo(octomap::point3d query,
                      octomap::OcTreeNode* node);

 private:
  int tree_depth_;
  double resolution_;
  double max_range_, min_range_;
  float inflate_radius_;
  std::vector<Eigen::Vector3d> sphere_;  // Discretized sphere used in map inflation
  std::vector<double> depth_volumes_;     // Volume per depth in the tree

  // Methods
  double VectorNormSquared(const double &x,
                           const double &y,
                           const double &z);
  std_msgs::ColorRGBA HeightMapColor(const double &height,
                                     const double &alpha);
};

}  // namespace octoclass

#endif  // MAPPER_OCTOCLASS_H_
