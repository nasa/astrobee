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

#ifndef MAPPER_SAMPLED_TRAJECTORY_H_
#define MAPPER_SAMPLED_TRAJECTORY_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "mapper/polynomials.h"
#include "mapper/linear_algebra.h"
#include "mapper/visualization_functions.h"

namespace sampled_traj {

//  Pos has the discretized points in the trajectory,
// and is compressed with the function compressSamples
//  Time has the corresponding times within Pos
//  nPoints has the number of points in the Pos vector
//  ThickTraj is of the octree type to avoid repeated nodes
// that occurs when concatenating trajectories between two
// waypoints
class SampledTrajectory3D{
 public:
  // Sampled trajectory variables
  pcl::PointCloud<pcl::PointXYZ> pos_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_pos_;
  std::vector<double> time_;
  int n_points_;

  // Compressed samples (std::vector can delete entries, pcl::PointCloud isn't that easy)
  std::vector<Eigen::Vector3d> compressed_pos_;
  std::vector<double> compressed_time_;
  int n_compressed_points_;   // Number of points after compression
  double max_dev_;          // Max deviation for compression

  // Thick trajectory variables
  octomap::OcTree thick_traj_ = octomap::OcTree(0.1);  // Create empty tree with resolution 0.1
  pcl::PointCloud< pcl::PointXYZ > point_cloud_traj_;
  double resolution_;
  double thickness_;


  // Constructor
  SampledTrajectory3D(const double &dt,
            const polynomials::Trajectory3D &poly_trajectories);
  SampledTrajectory3D(const std::vector<double> &time_vec,
            const pcl::PointCloud<pcl::PointXYZ> &pos_vec);
  SampledTrajectory3D();

  // Methods
  void PrintSamples();
  void SetMaxDev(const double &max_dev);
  void SetResolution(const double &resolution);
  void DeleteSample(const int &index);
  void CompressSamples();
  void Bresenham(const Eigen::Vector3d &p0,
           const Eigen::Vector3d &pf,
           std::vector<octomap::point3d> *points);  // Bresenham line algorithm por printing a line
  void ThickBresenham(const Eigen::Vector3d &p0,
            const Eigen::Vector3d &pf);          // Thick bresenham line algorithm por printing a line
  void ThickTrajToPcl();
  void CreateKdTree();
  void SortCollisions(const std::vector<octomap::point3d> &colliding_nodes,
            std::vector<geometry_msgs::PointStamped> *samples);
  void TrajVisMarkers(visualization_msgs::MarkerArray* marker_array);
  void SamplesVisMarkers(visualization_msgs::MarkerArray* marker_array);
  void CompressedVisMarkers(visualization_msgs::MarkerArray* marker_array);
  void ClearObject();  // Clear all the data within this object
};

// Comparison funcion used in sort algorithm
bool ComparePointStamped(const geometry_msgs::PointStamped &sample1,
             const geometry_msgs::PointStamped &sample2);


}  // namespace sampled_traj

#endif  // MAPPER_SAMPLED_TRAJECTORY_H_
