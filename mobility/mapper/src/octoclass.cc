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

#include <mapper/octoclass.h>
#include <mapper/pcl_conversions.h>

#include <utility>
#include <algorithm>
#include <vector>
#include <limits>

namespace octoclass {

OctoClass::OctoClass(const double resolution) {
    tree_.setResolution(resolution);
    tree_inflated_.setResolution(resolution);
    tree_depth_ = tree_.getTreeDepth();
    resolution_ = resolution;
}

OctoClass::OctoClass() {}

void OctoClass::SetMemoryTime(const double memory) {
  memory_time_ = memory;
  ROS_DEBUG("Fading memory time: %f seconds", memory_time_);
}

double OctoClass::GetMemoryTime() {
  return memory_time_;
}

void OctoClass::SetMaxRange(const double max_range) {
  max_range_ = max_range;
  ROS_DEBUG("Maximum range: %f meters", max_range_);
}

void OctoClass::SetMinRange(const double min_range) {
  min_range_ = min_range;
  ROS_DEBUG("Minimum range: %f meters", min_range_);
}

void OctoClass::SetResolution(const double resolution_in) {
  resolution_ = resolution_in;
  tree_.setResolution(resolution_);
  tree_inflated_.setResolution(resolution_);
  ResetMap();

  // Set the volumes for the node sizes
  depth_volumes_.resize(tree_depth_+1);
  for (unsigned i= 0; i < depth_volumes_.size(); ++i)
    depth_volumes_[i] = pow(tree_inflated_.getNodeSize(i), 3);

  sphere_.clear();
  static Eigen::Vector3d xyz;
  const int max_xyz = static_cast<int>(round(inflate_radius_/resolution_));
  const float max_dist = inflate_radius_*inflate_radius_;
  static float d_origin;
  for (int x = -max_xyz; x <= max_xyz; x++) {
    for (int y = -max_xyz; y <= max_xyz; y++) {
      for (int z = -max_xyz; z <= max_xyz; z++) {
        xyz << x*resolution_, y*resolution_, z*resolution_;
        d_origin = xyz.dot(xyz);  // distance from origin squared
        if (d_origin <= max_dist)
          sphere_.push_back(xyz);
      }
    }
  }
  ROS_DEBUG("Map resolution: %f meters", resolution_);
}

double OctoClass::GetResolution() {
  return resolution_;
}

void OctoClass::SetMapInflation(const double inflate_radius) {
  inflate_radius_ = inflate_radius;
  ResetMap();
  sphere_.clear();
  static Eigen::Vector3d xyz;
  // if (inflateMap) {
  ROS_DEBUG("The map is being inflated by a radius of %f!", inflate_radius_);
  const int max_xyz = static_cast<int>(round(inflate_radius_/resolution_));
  const float max_dist = inflate_radius_*inflate_radius_;
  static float d_origin;
  for (int x = -max_xyz; x <= max_xyz; x++) {
    for (int y = -max_xyz; y <= max_xyz; y++) {
      for (int z = -max_xyz; z <= max_xyz; z++) {
        xyz << x*resolution_, y*resolution_, z*resolution_;
        d_origin = xyz.dot(xyz);  // distance from origin squared
        if (d_origin <= max_dist) {
            sphere_.push_back(xyz);
        }
      }
    }
  }
}

double OctoClass::GetMapInflation() {
  return inflate_radius_;
}

void OctoClass::SetCamFrustum(const double fov,
                              const double aspect_ratio) {
  cam_frustum_ = algebra_3d::FrustumPlanes(fov, aspect_ratio);
  ROS_DEBUG("Cam frustum was set!");
}

void OctoClass::ResetMap() {
  tree_.clear();
  tree_inflated_.clear();
  ROS_DEBUG("Map was reset!");
}

void OctoClass::SetOccupancyThreshold(const double occupancy_threshold) {
  tree_.setOccupancyThres(occupancy_threshold);
  tree_inflated_.setOccupancyThres(occupancy_threshold);
  ROS_DEBUG("Occupancy probability threshold: %f", occupancy_threshold);
}

void OctoClass::SetHitMissProbabilities(const double probability_hit,
                             const double probability_miss) {
  tree_.setProbHit(probability_hit);
  tree_.setProbMiss(probability_miss);
  tree_inflated_.setProbHit(probability_hit);
  tree_inflated_.setProbMiss(probability_miss);
  ROS_DEBUG("Probability hit: %f", probability_hit);
  ROS_DEBUG("Probability miss: %f", probability_miss);
}

void OctoClass::SetClampingThresholds(const double clamping_threshold_min,
                           const double clamping_threshold_max) {
  tree_.setClampingThresMin(clamping_threshold_min);
  tree_.setClampingThresMax(clamping_threshold_max);
  tree_inflated_.setClampingThresMin(clamping_threshold_min);
  tree_inflated_.setClampingThresMax(clamping_threshold_max);
  ROS_DEBUG("Clamping threshold minimum: %f", clamping_threshold_min);
  ROS_DEBUG("Clamping threshold maximum: %f", clamping_threshold_max);
}

// // Function obtained from https://github.com/OctoMap/octomap_ros
// void OctoClass::PointsOctomapToPointCloud2(const octomap::point3d_list& points,
//                                            sensor_msgs::PointCloud2* cloud) {
//   // create pcl version
//   pcl::PointCloud<pcl::PointXYZ> out_cloud;
//  out_cloud.reserve(points.size());

//   // copy points
//   for (auto &p : points)
//     out_cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));

//   // convert msg
//   pcl::toROSMsg(out_cloud, *cloud);

//   // add stamps
//   cloud->header.stamp = ros::Time::now();
//   cloud->header.frame_id = "world";
// }

void OctoClass::PclToRayOctomap(const pcl::PointCloud< pcl::PointXYZ > &cloud,
                                const geometry_msgs::TransformStamped &tf_cam2world,
                                const algebra_3d::FrustumPlanes &frustum) {
  // set camera origin
  const octomap::point3d cam_origin = octomap::point3d(
    tf_cam2world.transform.translation.x,
    tf_cam2world.transform.translation.y,
    tf_cam2world.transform.translation.z);
  const double min_threshold_sqr = min_range_*min_range_;
  const uint32_t width = cloud.width;
  const uint32_t height = cloud.height;
  static double range_sqr;

  // discretize point cloud
  // octomap::Pointcloud octoCloud, inflatedSafeCloud;
  // octoCloud.reserve(cloud.height*cloud.width);
  octomap::KeySet endpoints, endpoints_inflated;
  static octomap::point3d central_point, cur_point;
  const double max_range_sqr = max_range_*max_range_;
  pcl::PointXYZ point;
  for (uint32_t i = 0; i < height; i++) {
    for (uint32_t j = 0; j < width; j++) {
      point = cloud.at(j, i);

      // Check if the point is invalid
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
        continue;

      // Check if point is within camera frustum
      if (!frustum.IsPointWithinFrustum(Eigen::Vector3d(point.x, point.y, point.z)))
        continue;

      // points too close to origin of camera are not added
      range_sqr = this->VectorNormSquared(
        tf_cam2world.transform.translation.x - point.x,
        tf_cam2world.transform.translation.y - point.y,
        tf_cam2world.transform.translation.z - point.z);
      if ((range_sqr < min_threshold_sqr))
        continue;

      // create discretized octocloud
      octomap::OcTreeKey k = tree_.coordToKey(
        octomap::point3d(point.x, point.y, point.z));
      // Add non-repeated nodes to keysets (inflated and non-inflated)
      std::pair<octomap::KeySet::iterator, bool> ret = endpoints.insert(k);
      octomap::OcTreeKey key;
      if (ret.second) {  // insertion took place => k was not in set
        // insert points in inflated octomap
        if (range_sqr < max_range_sqr) {
          central_point = tree_.keyToCoord(k);
          for (uint j = 0; j < sphere_.size(); j++) {
            cur_point = central_point + octomap::point3d(sphere_[j][0], sphere_[j][1], sphere_[j][2]);
            if (!frustum.IsPointWithinFrustum(Eigen::Vector3d(cur_point.x(),
                                                              cur_point.y(),
                                                              cur_point.z())))
                continue;
            if (tree_.coordToKeyChecked(cur_point, key))
               endpoints_inflated.insert(key);
          }
        }
      }
    }
  }

  // Calculate free nodes
  octomap::KeySet occ_cells_in_range, free_cells, inflated_free_cells;
  ComputeUpdate(endpoints_inflated, endpoints, cam_origin, max_range_,
                &occ_cells_in_range, &free_cells, &inflated_free_cells);

  for (octomap::KeySet::iterator it = endpoints_inflated.begin(); it != endpoints_inflated.end(); ++it) {
    // Only add nodes that are being added to the slim tree as well
    if (free_cells.find(*it) != free_cells.end()) {
        tree_inflated_.updateNode(*it, true);
    } else if (endpoints.find(*it) != endpoints.end()) {
        tree_inflated_.updateNode(*it, true);
    }
  }
  for (octomap::KeySet::iterator it = occ_cells_in_range.begin(); it != occ_cells_in_range.end(); ++it) {
    tree_.updateNode(*it, true);
  }
  for (octomap::KeySet::iterator it = inflated_free_cells.begin(); it != inflated_free_cells.end(); ++it) {
    tree_inflated_.updateNode(*it, false);
    tree_.updateNode(*it, false);
  }
}

void OctoClass::ComputeUpdate(const octomap::KeySet &occ_inflated,  // Inflated endpoints
                              const octomap::KeySet &occ_slim,      // Non-inflated endpoints
                              const octomap::point3d& origin,
                              const double &max_range,
                              octomap::KeySet *occ_slim_in_range,
                              octomap::KeySet *free_slim,
                              octomap::KeySet *free_inflated) {
  octomap::KeyRay keyray;
  for (octomap::KeySet::const_iterator occIt = occ_slim.begin(); occIt != occ_slim.end(); ++occIt) {
    const octomap::point3d& p = tree_inflated_.keyToCoord(*occIt);
    // If in line of sight, add free cells
    if ((max_range < 0.0) || ((p - origin).norm() <= max_range)) {  // is not max_range_ meas.
        octomap::OcTreeKey key;
        if (tree_.coordToKeyChecked(p, key))
            occ_slim_in_range->insert(key);
        // Ray keys are already checked using coordToKeyChecked
        tree_inflated_.computeRayKeys(origin, p, keyray);
        free_slim->insert(keyray.begin(), keyray.end());
        for (octomap::KeyRay::iterator it = keyray.begin(); it != keyray.end(); ++it) {
          if (occ_inflated.find(*it) == occ_inflated.end())   // If not occupied
              free_inflated->insert(*it);
          else  // If occupied
            break;
        }
    } else {  // user set a max_range_ and length is above
      octomap::point3d direction = (p - origin).normalized();
      octomap::point3d new_end = origin + direction * static_cast<float>(max_range);
      tree_inflated_.computeRayKeys(origin, new_end, keyray);
      free_slim->insert(keyray.begin(), keyray.end());
      for (octomap::KeyRay::iterator it = keyray.begin(); it != keyray.end(); ++it) {
        if (occ_inflated.find(*it) == occ_inflated.end())  // If not occupied
          free_inflated->insert(*it);
        else
          break;
      }
    }
  }
}

void OctoClass::FadeMemory(const double &rate) {  // rate at which this function is being called
  const double clamp_log_max = tree_.getClampingThresMaxLog();
  const double clamp_log_min = tree_.getClampingThresMinLog();
  const double occ_thres = tree_.getOccupancyThres();
  // const double probLogHit = tree.getProbHitLog();
  const double prob_log_range_obs = clamp_log_max - occ_thres;
  const double prob_log_range_free = clamp_log_min - occ_thres;
  const double fading_obs_log_prob_per_run = -prob_log_range_obs/(memory_time_*rate);
  const double fading_free_log_prob_per_run = -prob_log_range_free/(memory_time_*rate);

  static bool is_occ;
  static octomap::OcTreeKey key;

  // Fade tree_
  for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
                                     end= tree_.end_leafs();
                                     it!= end; ++it) {
    // fade obstacles and free areas
    key = it.getKey();
    octomap::OcTreeNode* n = tree_.search(key);
    is_occ = tree_.isNodeOccupied(n);
    if (is_occ)
        tree_.updateNodeLogOdds(n, fading_obs_log_prob_per_run);
    else
        tree_.updateNodeLogOdds(n, fading_free_log_prob_per_run);

    // tree nodes that are unknown
    // if it was occupied then disoccupied, delete node
    if (is_occ != tree_.isNodeOccupied(n))
      tree_.deleteNode(key, it.getDepth());
  }
  // Fade tree_inflated_
  for (octomap::OcTree::leaf_iterator it = tree_inflated_.begin_leafs(),
                                     end= tree_inflated_.end_leafs();
                                     it!= end; ++it) {
    // fade obstacles and free areas
    key = it.getKey();
    octomap::OcTreeNode* n = tree_inflated_.search(key);
    is_occ = tree_inflated_.isNodeOccupied(n);
    if (is_occ)
        tree_inflated_.updateNodeLogOdds(n, fading_obs_log_prob_per_run);
    else
        tree_inflated_.updateNodeLogOdds(n, fading_free_log_prob_per_run);

    // tree nodes that are unknown
    // if it was occupied then disoccupied, delete node
    if (is_occ != tree_inflated_.isNodeOccupied(n))
      tree_inflated_.deleteNode(key, it.getDepth());
  }
}

// void OctoClass::InflateObstacles(const double &thickness) {
//   // set all pixels in a sphere around the origin
//   std::vector<Eigen::Vector3d> sphere;
//   static Eigen::Vector3d xyz;
//   const int max_xyz = static_cast<int>(ceil(thickness/resolution_));
//   const float max_dist = thickness*thickness;
//   static float d_origin;
//   for (int x = -max_xyz; x <= max_xyz; x++) {
//     for (int y = -max_xyz; y <= max_xyz; y++) {
//       for (int z = -max_xyz; z <= max_xyz; z++) {
//         xyz << x*resolution_, y*resolution_, z*resolution_;
//         d_origin = xyz.dot(xyz);  // distance from origin squared
//         if (d_origin <= max_dist)
//           sphere.push_back(xyz);
//       }
//     }
//   }

//   // create inflated tree
//   tree_inflated_.clear();
//   const int n_sphere_nodes = sphere.size();
//   static bool is_central_occ;
//   static octomap::point3d central_point, cur_point;
//   octomap::OcTreeNode *node, *node_inflated;
//   for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
//                                      end= tree_.end_leafs();
//                                    it!= end; ++it) {
//     // check occupancy of the current point
//     central_point = it.getCoordinate();
//     node = tree_.search(central_point);
//     is_central_occ = tree_.isNodeOccupied(node);

//     // populate the inflated map accordingly
//     if (is_central_occ) {  // populate the inflated with a sphere around this node
//       for (int j = 0; j < n_sphere_nodes; j++) {
//           cur_point = central_point + octomap::point3d(sphere[j][0], sphere[j][1], sphere[j][2]);
//           tree_inflated_.updateNode(cur_point, true);
//       }
//     } else {  // set as free if not uccupied already
//       node_inflated = tree_inflated_.search(central_point);
//       if (node_inflated == NULL)
//         tree_inflated_.updateNode(central_point, false);
//       else if (!tree_inflated_.isNodeOccupied(node_inflated))
//         tree_inflated_.updateNode(central_point, false);
//     }
//   }
// }

// void OctoClass::FindCollidingNodesTree(const pcl::PointCloud< pcl::PointXYZ > &point_cloud,
//                                        std::vector<octomap::point3d> *colliding_nodes) {
//   static octomap::point3d query, node_center;
//   static octomap::OcTreeNode* node;
//   static octomap::OcTreeKey key;
//   octomap::KeySet endpoints;

//   const int cloudsize = point_cloud.size();

//   for (int j = 0; j < cloudsize; j++) {
//     query = octomap::point3d(point_cloud.points[j].x,
//                              point_cloud.points[j].y,
//                              point_cloud.points[j].z);
//     key = tree_.coordToKey(query);
//     std::pair<octomap::KeySet::iterator, bool> ret = endpoints.insert(key);

//     // check if current node has not been evaluated yet
//     if (ret.second) {  // insertion took place => new node being evaluated
//       node = tree_.search(query);
//       if (node == NULL) {
//         continue;
//       } else if (tree_.isNodeOccupied(node)) {
//         node_center = tree_.keyToCoord(key);
//         colliding_nodes->push_back(node_center);
//       }
//     }
//   }
// }

void OctoClass::FindCollidingNodesInflated(const pcl::PointCloud< pcl::PointXYZ > &point_cloud,
                                           std::vector<octomap::point3d> *colliding_nodes) {
  static octomap::point3d query, node_center;
  static octomap::OcTreeNode* node;
  static octomap::OcTreeKey key;
  octomap::KeySet endpoints;

  const int cloudsize = point_cloud.size();

  for (int j = 0; j < cloudsize; j++) {
    query = octomap::point3d(point_cloud.points[j].x,
                             point_cloud.points[j].y,
                             point_cloud.points[j].z);
    key = tree_inflated_.coordToKey(query);
    std::pair<octomap::KeySet::iterator, bool> ret = endpoints.insert(key);

    // check if current node has not been evaluated yet
    if (ret.second) {  // insertion took place => new node being evaluated
      node = tree_inflated_.search(query);
      if (node == NULL) {
        continue;
      } else if (tree_inflated_.isNodeOccupied(node)) {
        node_center = tree_inflated_.keyToCoord(key);
        colliding_nodes->push_back(node_center);
      }
    }
  }
}

// turns into voxelized representation
// octomap::point3d_list OctoClass::Voxelize(const octomap::OcTree::leaf_iterator &leaf) {
//   double res = tree_.getResolution();
//   double size = leaf.getSize();

//   double low = -size*0.5 +  res/2.0;
//   double high = -low;

//   octomap::point3d_list list;

//   for (double dx = low; dx <= high; dx+=res) {
//     for (double dy = low; dy <= high; dy+=res) {
//       for (double dz = low; dz <= high; dz+=res) {
//         octomap::point3d d(dx, dy, dz);
//         list.push_back(d+leaf.getCoordinate());
//       }
//     }
//   }

//   return list;
// }

// adapted from https:// ithub.com/OctoMap/octomap_mapping
void OctoClass::TreeVisMarkers(visualization_msgs::MarkerArray* obstacles,
                               visualization_msgs::MarkerArray* free,
                               sensor_msgs::PointCloud2* obstacles_cloud,
                               sensor_msgs::PointCloud2* free_cloud) {
  // Markers: each marker array stores a set of nodes with similar size
  obstacles->markers.resize(tree_depth_+1);
  free->markers.resize(tree_depth_+1);
  const ros::Time rostime = ros::Time::now();

  // get pointcloud holder
  pcl::PointCloud<pcl::PointXYZ>  free_points, obstacles_points;

  // set tree min and max
  static double min_x, min_y, min_z, max_x, max_y, max_z;
  tree_.getMetricMin(min_x, min_y, min_z);
  tree_.getMetricMax(max_x, max_y, max_z);
  const double colorFactor = 1.0;   // define the gradient of colors

  // publish all leafs from the tree
  static geometry_msgs::Point point_center;
  for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
                                     end= tree_.end_leafs();
                                     it!= end; ++it) {
    // set depth in the tree
    const unsigned idx = it.getDepth();
    point_center.x = it.getX();
    point_center.y = it.getY();
    point_center.z = it.getZ();
    const double h = (1.0 - std::min(std::max((point_center.z-min_z)/ (max_z - min_z), 0.0), 1.0))*colorFactor;

    if (tree_.isNodeOccupied(*it)) {
      // Add point and set color based on height
      obstacles->markers[idx].points.push_back(point_center);
      obstacles->markers[idx].colors.push_back(HeightMapColor(h, 1.0));
      obstacles_points.push_back(pcl::PointXYZ(point_center.x, point_center.y, point_center.z));
    } else {
      free->markers[idx].points.push_back(point_center);
      free->markers[idx].colors.push_back(HeightMapColor(h, 0.05));
      free_points.push_back(pcl::PointXYZ(point_center.x, point_center.y, point_center.z));
    }
  }

  // set marker properties
  for (unsigned i= 0; i < obstacles->markers.size(); ++i) {
    const double size = tree_.getNodeSize(i);
    obstacles->markers[i].header.frame_id = "world";
    obstacles->markers[i].header.stamp = rostime;
    obstacles->markers[i].ns = "obstacleMap";
    obstacles->markers[i].id = i;
    obstacles->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    obstacles->markers[i].scale.x = size;
    obstacles->markers[i].scale.y = size;
    obstacles->markers[i].scale.z = size;
    obstacles->markers[i].pose.orientation.w = 1.0;

    free->markers[i].header = obstacles->markers[i].header;
    free->markers[i].ns = obstacles->markers[i].ns;
    free->markers[i].id = obstacles->markers[i].id;
    free->markers[i].type = obstacles->markers[i].type;
    free->markers[i].scale = obstacles->markers[i].scale;
    free->markers[i].pose.orientation.w = 1.0;

    if (obstacles->markers[i].points.size() > 0)
        obstacles->markers[i].action = visualization_msgs::Marker::ADD;
    else
        obstacles->markers[i].action = visualization_msgs::Marker::DELETE;

    if (free->markers[i].points.size() > 0)
        free->markers[i].action = visualization_msgs::Marker::ADD;
    else
        free->markers[i].action = visualization_msgs::Marker::DELETE;
  }

  pcl::toROSMsg(free_points, *free_cloud);
  free_cloud->header.stamp = ros::Time::now();
  free_cloud->header.frame_id = "world";
  pcl::toROSMsg(obstacles_points, *obstacles_cloud);
  obstacles_cloud->header.stamp = ros::Time::now();
  obstacles_cloud->header.frame_id = "world";
}

// adapted from https:// github.com/OctoMap/octomap_mapping
void OctoClass::InflatedVisMarkers(visualization_msgs::MarkerArray* obstacles,
                                   visualization_msgs::MarkerArray* free,
                                   sensor_msgs::PointCloud2* obstacles_cloud,
                                   sensor_msgs::PointCloud2* free_cloud) {
  // Markers: each marker array stores a set of nodes with similar size
  obstacles->markers.resize(tree_depth_+1);
  free->markers.resize(tree_depth_+1);
  const ros::Time rostime = ros::Time::now();

  // get pointcloud holder
  pcl::PointCloud<pcl::PointXYZ> free_points, obstacles_points;

  // set tree_inflated_ min and max
  static double min_x, min_y, min_z, max_x, max_y, max_z;
  tree_inflated_.getMetricMin(min_x, min_y, min_z);
  tree_inflated_.getMetricMax(max_x, max_y, max_z);
  const double colorFactor = 1.0;   // define the gradient of colors

  // publish all leafs from the tree_inflated_
  static geometry_msgs::Point point_center;
  for (octomap::OcTree::leaf_iterator it = tree_inflated_.begin_leafs(),
                                     end= tree_inflated_.end_leafs();
                                     it!= end; ++it) {
      // set depth in the tree_inflated_
      const unsigned idx = it.getDepth();
      point_center.x = it.getX();
      point_center.y = it.getY();
      point_center.z = it.getZ();
      const double h = (1.0 - std::min(std::max((point_center.z-min_z)/ (max_z - min_z), 0.0), 1.0))*colorFactor;
      if (tree_inflated_.isNodeOccupied(*it)) {
          // Add point and set color based on height
          obstacles->markers[idx].points.push_back(point_center);
          obstacles->markers[idx].colors.push_back(HeightMapColor(h, 1.0));
          obstacles_points.push_back(pcl::PointXYZ(point_center.x, point_center.y, point_center.z));
      } else {
          free->markers[idx].points.push_back(point_center);
          free->markers[idx].colors.push_back(HeightMapColor(h, 0.05));
          free_points.push_back(pcl::PointXYZ(point_center.x, point_center.y, point_center.z));
      }
  }

  // set marker properties
  for (unsigned i= 0; i < obstacles->markers.size(); ++i) {
      const double size = tree_inflated_.getNodeSize(i);
    obstacles->markers[i].header.frame_id = "world";
    obstacles->markers[i].header.stamp = rostime;
    obstacles->markers[i].ns = "obstacleMap";
    obstacles->markers[i].id = i;
    obstacles->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    obstacles->markers[i].scale.x = size;
    obstacles->markers[i].scale.y = size;
    obstacles->markers[i].scale.z = size;
    obstacles->markers[i].pose.orientation.w = 1.0;

    free->markers[i].header = obstacles->markers[i].header;
    free->markers[i].ns = obstacles->markers[i].ns;
    free->markers[i].id = obstacles->markers[i].id;
    free->markers[i].type = obstacles->markers[i].type;
    free->markers[i].scale = obstacles->markers[i].scale;
    free->markers[i].pose.orientation.w = 1.0;

    if (obstacles->markers[i].points.size() > 0)
      obstacles->markers[i].action = visualization_msgs::Marker::ADD;
    else
      obstacles->markers[i].action = visualization_msgs::Marker::DELETE;

    if (free->markers[i].points.size() > 0)
      free->markers[i].action = visualization_msgs::Marker::ADD;
    else
      free->markers[i].action = visualization_msgs::Marker::DELETE;
  }
  pcl::toROSMsg(free_points, *free_cloud);
  free_cloud->header.stamp = ros::Time::now();
  free_cloud->header.frame_id = "world";
  pcl::toROSMsg(obstacles_points, *obstacles_cloud);
  obstacles_cloud->header.stamp = ros::Time::now();
  obstacles_cloud->header.frame_id = "world";
}

// Returns -1 if node is unknown, 0 if its free and 1 if its occupied
int OctoClass::CheckOccupancy(const octomap::point3d &p) {
  static octomap::OcTreeKey key;
  key = tree_inflated_.coordToKey(p);
  const octomap::OcTreeNode* n = tree_inflated_.search(key);
  if (n == NULL)
    return -1;
  else if (tree_inflated_.isNodeOccupied(n))
    return 1;
  return 0;
}

// Returns -1 if node is unknown, 0 if its free and 1 if its occupied
int OctoClass::CheckOccupancy(const pcl::PointXYZ &p) {
  return CheckOccupancy(octomap::point3d(p.x, p.y, p.z));
}

// Returns -1 if node is unknown, 0 if its free and 1 if its occupied
int OctoClass::CheckOccupancy(const Eigen::Vector3d &p) {
  return CheckOccupancy(octomap::point3d(p[0], p[1], p[2]));
}

// Returns 1 if there is at least one colliding node, otherwise
// returns -1 if there is at least one unknown node, otherwise
// returns 0 (only free nodes between points)
int OctoClass::CheckOccupancy(const octomap::point3d &p1,
                              const octomap::point3d &p2) {
  octomap::KeyRay ray;
  tree_inflated_.computeRayKeys(p1, p2, ray);
  int retVal = 0;
  const octomap::OcTreeNode* n;
  for (octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); ++it) {
    n = tree_inflated_.search(*it);
    if (n == NULL)
      retVal = -1;
    else if (tree_inflated_.isNodeOccupied(n))
      return 1;
  }

  // computeRayKeys does not compute the final point, so we check manually
  static octomap::OcTreeKey key;
  key = tree_inflated_.coordToKey(p2);
  n = tree_inflated_.search(key);
  if (n == NULL)
    retVal = -1;
  else if (tree_inflated_.isNodeOccupied(n))
    return 1;
  return retVal;
}

int OctoClass::CheckOccupancy(const Eigen::Vector3d &p1,
                              const Eigen::Vector3d &p2) {
  return CheckOccupancy(octomap::point3d(p1[0], p1[1], p1[2]),
                        octomap::point3d(p2[0], p2[1], p2[2]));
}

bool OctoClass::CheckCollision(const octomap::point3d &p) {
  static octomap::OcTreeKey key;
  key = tree_inflated_.coordToKey(p);
  const octomap::OcTreeNode* n = tree_inflated_.search(key);
  if (n == NULL)
      return true;
  else if (tree_inflated_.isNodeOccupied(n))
      return true;
  return false;
}

bool OctoClass::CheckCollision(const Eigen::Vector3d &p) {
    return CheckCollision(octomap::point3d(p[0], p[1], p[2]));
}

bool OctoClass::CheckCollision(const octomap::point3d &p1,
                               const octomap::point3d &p2) {
  octomap::KeyRay ray;
  tree_inflated_.computeRayKeys(p1, p2, ray);
  const octomap::OcTreeNode* n;
  for (octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); ++it) {
    n = tree_inflated_.search(*it);
    if (n == NULL)
      return true;
    else if (tree_inflated_.isNodeOccupied(n))
      return true;
  }

  // computeRayKeys does not compute the final point, so we check manually
  static octomap::OcTreeKey key;
  key = tree_inflated_.coordToKey(p2);
  n = tree_inflated_.search(key);
  if (n == NULL)
      return true;
  else if (tree_inflated_.isNodeOccupied(n))
      return true;
  return false;
}

bool OctoClass::CheckCollision(const Eigen::Vector3d &p1,
                               const Eigen::Vector3d &p2) {
  octomap::point3d p1_, p2_;
  p1_ = octomap::point3d(p1[0], p1[1], p1[2]);
  p2_ = octomap::point3d(p2[0], p2[1], p2[2]);
  return CheckCollision(p1_, p2_);
}

void OctoClass::BBXFreeVolume(const Eigen::Vector3d &box_min,
                              const Eigen::Vector3d &box_max,
                              double *volume) {
  // Initialize vector with zeros
  std::vector<uint> n_nodes_per_depth(tree_depth_ + 1);
  for (uint i = 0; i < n_nodes_per_depth.size(); i++) {
      n_nodes_per_depth[i] = 0;
  }

  // Count number of nodes per depth in the tree
  octomap::OcTree::leaf_bbx_iterator it;
  uint depth;
  const octomap::OcTreeNode* n;
  for (it = tree_inflated_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                           octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                           it != tree_inflated_.end_leafs_bbx(); ++it) {
    n = tree_inflated_.search(it.getKey());
    if (n == NULL)
      continue;

    // Count free nodes per depth
    if (!tree_inflated_.isNodeOccupied(n)) {
        depth = it.getDepth();
        n_nodes_per_depth[depth] = n_nodes_per_depth[depth] + 1;
    }
  }

  // Calculate final volume
  *volume = 0;
  for (uint i = 0; i < n_nodes_per_depth.size(); i++)
      *volume = *volume + static_cast<double>(n_nodes_per_depth[i]) * depth_volumes_[i];
}

// Calculate the volume of obstacles in the bounding box
void OctoClass::BBXOccVolume(const Eigen::Vector3d &box_min,
                             const Eigen::Vector3d &box_max,
                             double *volume) {
  // Initialize vector with zeros
  std::vector<uint> n_nodes_per_depth(tree_depth_ + 1);
  for (uint i = 0; i < n_nodes_per_depth.size(); i++)
    n_nodes_per_depth[i] = 0;

  // Count number of nodes per depth in the bounding box
  octomap::OcTree::leaf_bbx_iterator it;
  uint depth;
  const octomap::OcTreeNode* n;
  for (it = tree_inflated_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                         octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                         it != tree_inflated_.end_leafs_bbx(); ++it) {
    n = tree_inflated_.search(it.getKey());
    if (n == NULL)
      continue;
    // Count free nodes per depth
    if (tree_inflated_.isNodeOccupied(n)) {
        depth = it.getDepth();
        n_nodes_per_depth[depth] = n_nodes_per_depth[depth] + 1;
    }
  }

  // Calculate final volume
  *volume = 0;
  for (uint i = 0; i < n_nodes_per_depth.size(); i++) {
      // ROS_INFO("Nodes in depth %d: %d", int(i), int(n_nodes_per_depth[i]));
      *volume = *volume + static_cast<double>(n_nodes_per_depth[i]) * depth_volumes_[i];
  }
  // ROS_INFO("Volume calculated: %f", volume);
}

// Return all free nodes within a bounding box
void OctoClass::BBXFreeNodes(const Eigen::Vector3d &box_min,
                             const Eigen::Vector3d &box_max,
                             std::vector<octomap::OcTreeKey> *node_keys,
                             std::vector<double> *node_sizes) {
  octomap::OcTree::leaf_bbx_iterator it;
  // uint depth;
  const octomap::OcTreeNode* n;
  // octomap::point3d nodeCenter;
  octomap::OcTreeKey key;
  for (it = tree_inflated_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                         octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                         it != tree_inflated_.end_leafs_bbx(); ++it) {
    key = it.getKey();
    n = tree_inflated_.search(key);
    if (n == NULL)
      continue;
    if (!tree_inflated_.isNodeOccupied(n)) {
      node_keys->push_back(key);
      node_sizes->push_back(tree_inflated_.getNodeSize(it.getDepth()));
    }
  }
}

// Return all free nodes within a bounding box
void OctoClass::BBXFreeNodes(const Eigen::Vector3d &box_min,
                             const Eigen::Vector3d &box_max,
                             IndexedKeySet *indexed_node_keys,
                             std::vector<double> *node_sizes) {
  octomap::OcTree::leaf_bbx_iterator it;
  const octomap::OcTreeNode* n;
  octomap::OcTreeKey key;
  uint index = 0;
  for (it = tree_inflated_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                         octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                         it != tree_inflated_.end_leafs_bbx(); ++it) {
    key = it.getKey();
    n = tree_inflated_.search(key);
    if (n == NULL)
      continue;
    if (!tree_inflated_.isNodeOccupied(n)) {
      indexed_node_keys->Insert(key, index);
      node_sizes->push_back(tree_inflated_.getNodeSize(it.getDepth()));
      index++;
    }
  }
}

void OctoClass::GetNodeNeighbors(const octomap::OcTreeKey &node_key,
                                 const double &node_size,
                                 std::vector<octomap::OcTreeKey> *neighbor_keys) {
  // const double size = getNodeSize(node_key);
  const Eigen::Vector3d Bounds = Eigen::Vector3d((node_size + resolution_)/2.0,
                                                 (node_size + resolution_)/2.0,
                                                 (node_size + resolution_)/2.0);
  octomap::point3d node_pos = tree_inflated_.keyToCoord(node_key);
  Eigen::Vector3d pos = Eigen::Vector3d(node_pos.x(), node_pos.y(), node_pos.z());

  // Get free nodes within a bounding box
  std::vector<octomap::OcTreeKey> bbx_keys;
  std::vector<double> bbx_sizes;
  BBXFreeNodes(pos-Bounds, pos+Bounds, &bbx_keys, &bbx_sizes);

  // Populate the nodeKeys vector with all bbxKeys, except node_key
  uint n_bbx_nodes = bbx_keys.size();
  if (n_bbx_nodes > 0) {
    neighbor_keys->reserve(n_bbx_nodes - 1);
    for (uint i = 0; i < bbx_keys.size(); i++) {
      if (bbx_keys[i] != node_key)
          neighbor_keys->push_back(bbx_keys[i]);
    }
  }
}

// Returns size of node. Returns zero if node doesn't exist
double OctoClass::GetNodeSize(const octomap::OcTreeKey &key) {
  const octomap::OcTreeNode* n;
  n = tree_inflated_.search(key);
  if (n == NULL) {
      return 0.0;
  } else {
    const octomap::point3d bounds = octomap::point3d(resolution_/4.0,
                                                     resolution_/4.0,
                                                     resolution_/4.0);
    const octomap::point3d pos = tree_inflated_.keyToCoord(key);
    octomap::OcTree::leaf_bbx_iterator it;
    it = tree_inflated_.begin_leafs_bbx(pos-bounds, pos+bounds);
    return tree_inflated_.getNodeSize(it.getDepth());
  }
}

void OctoClass::PrintQueryInfo(octomap::point3d query,
                               octomap::OcTreeNode* node) {
  if (node != NULL)
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  else
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

double OctoClass::VectorNormSquared(const double &x,
                                    const double &y,
                                    const double &z) {
    return x*x + y*y + z*z;
}

// extracted from https://github.com/OctoMap/octomap_mapping
std_msgs::ColorRGBA OctoClass::HeightMapColor(const double &height,
                                              const double &alpha) {
  std_msgs::ColorRGBA color;
  color.a = alpha;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  double height_mod = height;
  height_mod -= floor(height_mod);
  height_mod *= 6;
  int i;
  double m, n, f;

  i = floor(height_mod);
  f = height_mod - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 6:
  case 0:
    color.r = v; color.g = n; color.b = m;
    break;
  case 1:
    color.r = n; color.g = v; color.b = m;
    break;
  case 2:
    color.r = m; color.g = v; color.b = n;
    break;
  case 3:
    color.r = m; color.g = n; color.b = v;
    break;
  case 4:
    color.r = n; color.g = m; color.b = v;
    break;
  case 5:
    color.r = v; color.g = m; color.b = n;
    break;
  default:
    color.r = 1; color.g = 0.5; color.b = 0.5;
    break;
  }

  return color;
}

}  // namespace octoclass
