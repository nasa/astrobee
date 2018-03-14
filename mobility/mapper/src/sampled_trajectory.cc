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

#include <limits>
#include <algorithm>
#include <vector>
#include "mapper/sampled_trajectory.h"

namespace sampled_traj {

SampledTrajectory3D::SampledTrajectory3D(const double &dt,
                                         const polynomials::Trajectory3D &poly_trajectories) {
    static double t0, tf, delta_t;
    t0 = poly_trajectories.t0_;
    tf = poly_trajectories.tf_;
    delta_t = tf - t0;

    // define number of points for trajectory
    if (floor(delta_t/dt) == delta_t/dt) {
        n_points_ = delta_t/dt + 1;
    } else {
        n_points_ = delta_t/dt + 2;
    }

    // get vector values
    // Eigen::Vector3d Point;
    pcl::PointXYZ point;
    double time;
    for (int i = 0; i < n_points_; i++) {
        time = std::min(static_cast<float>(i)*dt + t0, tf);
        time_.push_back(time);
        poly_trajectories.TrajectoryAtTime(time, &point);
        pos_.push_back(point);
    }
}

SampledTrajectory3D::SampledTrajectory3D(const std::vector<double> &time_vec,
                                         const pcl::PointCloud<pcl::PointXYZ> &pos_vec) {
    time_ = time_vec;
    pos_ = pos_vec;
    n_points_ = time_.size();
}

SampledTrajectory3D::SampledTrajectory3D() {
    n_points_ = 0;
    // Pos = std::vector<Eigen::Vector3d>;
}

void SampledTrajectory3D::PrintSamples() {
    for (int i = 0; i < n_points_; i++) {
        std::cout << "Sample: " << i+1 << "\t";
        std::cout << "Time: " << time_[i] << "\t";
        std::cout << "Point: " << pos_[i].x << "\t" << pos_[i].y << "\t" << pos_[i].z << std::endl;
    }
}

void SampledTrajectory3D::SetMaxDev(const double &max_dev) {
    max_dev_ = max_dev;
    ROS_DEBUG("Trajectory compression max deviation is set to: %f", max_dev_);
}

void SampledTrajectory3D::SetResolution(const double &resolution) {
    resolution_ = resolution;
    thickness_ = 1.4*resolution;
    thick_traj_.setResolution(resolution_);
    thick_traj_.clear();
    ROS_DEBUG("Trajectory resolution is set to: %f", resolution_);
}

void SampledTrajectory3D::DeleteSample(const int &index) {
    compressed_pos_.erase(compressed_pos_.begin() + index);
    compressed_time_.erase(compressed_time_.begin() + index);
    n_compressed_points_ = n_compressed_points_ - 1;
}

void SampledTrajectory3D::CompressSamples() {
    // the minimum number of points for final vector
    static int min_points = 2;

    // initialize compressed points as all samples
    compressed_pos_.resize(n_points_);
    for (int i = 0; i < n_points_; i++) {
        compressed_pos_[i] << pos_[i].x, pos_[i].y, pos_[i].z;
    }
    compressed_time_ = time_;
    n_compressed_points_ = n_points_;

    // first delete colinear points
    static double epsilon = 0.0001, dist;
    static int delete_index;
    static Eigen::Vector3d p1, p2, p;
    // static algebra_3d::Line3d line;
    while (true) {
        int delete_index = -1;
        for (int i = 1; i < n_compressed_points_-1; i++) {
            p1 << compressed_pos_[i-1][0],
                  compressed_pos_[i-1][1],
                  compressed_pos_[i-1][2];
            p2 << compressed_pos_[i+1][0],
                  compressed_pos_[i+1][1],
                  compressed_pos_[i+1][2];
            p  << compressed_pos_[i][0],
                  compressed_pos_[i][1],
                  compressed_pos_[i][2];
            algebra_3d::Line3d line(p1, p2);
            line.DistancePoint2Line(p, &dist);
            if (dist < epsilon) {
                delete_index = i;
                break;
            }
        }
        if (delete_index > 0) {
            this->DeleteSample(delete_index);
        } else {
            break;
        }
    }
    // ROS_INFO("Number of non-colinear points: %d", n_compressed_points_);

    // now compress the remaining points
    double min_dist;
    while (n_compressed_points_ > min_points) {
        // first find the point that deviates the least in the whole set
        min_dist = std::numeric_limits<float>::infinity();
        for (int i = 1; i < n_compressed_points_-1; i++) {
            p1 << compressed_pos_[i-1][0],
                  compressed_pos_[i-1][1],
                  compressed_pos_[i-1][2];
            p2 << compressed_pos_[i+1][0],
                  compressed_pos_[i+1][1],
                  compressed_pos_[i+1][2];
            p  << compressed_pos_[i][0],
                  compressed_pos_[i][1],
                  compressed_pos_[i][2];
            algebra_3d::Line3d line(p1, p2);
            line.DistancePoint2Line(p, &dist);
            if (dist < min_dist) {
                min_dist = dist;
                delete_index = i;
            }
        }

        // if point does not deviate too much from original set, delete it
        if (min_dist > max_dev_) {
            break;
        } else {
            this->DeleteSample(delete_index);
        }
    }
    // ROS_INFO("Compressed points: %d", n_compressed_points_);
}


// It is highly likely that the original Author was Bob Pendelton
// I translated this algorithm from Matlab into C++ based on:
// http://www.mathworks.com/matlabcentral/fileexchange/21057-3d-bresenham-s-line-generation?focused=5102923&tab=function
void SampledTrajectory3D::Bresenham(const Eigen::Vector3d &p0,
                                    const Eigen::Vector3d &pf,
                                    std::vector<octomap::point3d> *points) {
    // Get initial and final pixel positions
    static int x1, x2, y1, y2, z1, z2;
    x1 = round(p0[0]/resolution_);
    x2 = round(pf[0]/resolution_);
    y1 = round(p0[1]/resolution_);
    y2 = round(pf[1]/resolution_);
    z1 = round(p0[2]/resolution_);
    z2 = round(pf[2]/resolution_);

    // Get the output vector length
    static int dx, dy, dz;
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;
    const int d = std::max({abs(dx), abs(dy), abs(dz)}) + 1;
    // Eigen::MatrixXi Pixels(d,3);
    points->reserve(d);

    // Extra variables
    static int ax, ay, az, sx, sy, sz, x, y, z, xd, yd, zd;
    ax = abs(dx)*2;
    ay = abs(dy)*2;
    az = abs(dz)*2;
    sx = (dx > 0) - (dx < 0);  // ign of dx
    sy = (dy > 0) - (dy < 0);  // ign of dy
    sz = (dz > 0) - (dz < 0);  // ign of dz
    x = x1;
    y = y1;
    z = z1;
    // idx = 0;

    octomap::point3d point;
    if (ax >= std::max(ay, az)) {  //  dominant
        yd = ay - ax/2;
        zd = az - ax/2;

        while (true) {
            // Pixel << x, y, z;
            // Pixels.row(idx) = Pixel;
            // idx = idx + 1;
            point = octomap::point3d(static_cast<double>(x)*resolution_,
                                     static_cast<double>(y)*resolution_,
                                     static_cast<double>(z)*resolution_);
            points->push_back(point);

            if (x == x2) {
                break;
            }

            if (yd >=0) {  // Move along y
                y = y + sy;
                yd = yd - ax;
            }

            if (zd >= 0) {  // Move along z
                z = z + sz;
                zd = zd - ax;
            }

            x = x + sx;  // Move along x
            yd = yd + ay;
            zd = zd + az;
        }
    } else if (ay >= std::max(ax, az)) {  //  dominant
        xd = ax - ay/2;
        zd = az - ay/2;

        while (true) {
            // Pixel << x, y, z;
            // Pixels.row(idx) = Pixel;
            // idx = idx + 1;
            point = octomap::point3d(static_cast<double>(x)*resolution_,
                                     static_cast<double>(y)*resolution_,
                                     static_cast<double>(z)*resolution_);
            points->push_back(point);

            if (y == y2) {
                break;
            }

            if (xd >= 0) {
                x = x + sx;
                xd = xd - ay;
            }

            if (zd >= 0) {
                z = z + sz;
                zd = zd - ay;
            }

            y = y + sy;
            xd = xd + ax;
            zd = zd + az;
        }
    } else if (az >= std::max(ax, ay)) {  //  dominant
        xd = ax - az/2;
        yd = ay - az/2;

        while (true) {
            // Pixel << x, y, z;
            // Pixels.row(idx) = Pixel;
            // idx = idx + 1;
            point = octomap::point3d(static_cast<double>(x)*resolution_,
                                     static_cast<double>(y)*resolution_,
                                     static_cast<double>(z)*resolution_);
            points->push_back(point);

            if (z == z2) {
                break;
            }

            if (xd >= 0) {
                x = x + sx;
                xd = xd - az;
            }

            if (yd >= 0) {
                y = y + sy;
                yd = yd - az;
            }

            z = z + sz;
            xd = xd + ax;
            yd = yd + ay;
        }
    }
}

// Algorithm for getting a "thick" line based on bresenham
// TODO(marcelino): Some voxels are painted twice, which can be improved.
void SampledTrajectory3D::ThickBresenham(const Eigen::Vector3d &p0,
                                         const Eigen::Vector3d &pf) {
    // Get vector in the direction of the trajectory (if length is greater than 0)
    static Eigen::Vector3d v1 = pf - p0;
    if (v1.norm() > 0) {
        v1 = v1/v1.norm();
    } else {
        v1 << 1, 0, 0;
    }


    // Set all pixels in a sphere around the origin
    // The pixels are assigned as initialEdge, finalEdge or trajectory padding
    std::vector<octomap::point3d> padding, init_edge_padding, final_edge_padding;
    static octomap::point3d xyz, v1_point3d;
    const int max_xyz = static_cast<int>(round(thickness_/resolution_));
    const float max_dist = thickness_*thickness_;
    static float d_origin, d_plane;
    v1_point3d = octomap::point3d(v1[0], v1[1], v1[2]);
    for (int x = -max_xyz; x <= max_xyz; x++) {
        for (int y = -max_xyz; y <= max_xyz; y++) {
            for (int z = -max_xyz; z <= max_xyz; z++) {
                xyz = octomap::point3d(x*resolution_, y*resolution_, z*resolution_);

                d_origin = xyz.dot(xyz);  // Distance from origin squared
                if (d_origin > max_dist) {
                    continue;
                }

                // Assign to proper vector
                d_plane = v1_point3d.dot(xyz);
                if (d_plane > 0.85*resolution_) {
                    final_edge_padding.push_back(xyz);
                } else if (d_plane < -0.5*resolution_) {
                    init_edge_padding.push_back(xyz);
                } else {
                    padding.push_back(xyz);
                }
            }
        }
    }

    // Get the standard bresenham
    std::vector<octomap::point3d> thin_bresenham;
    this->Bresenham(p0, pf, &thin_bresenham);

    // Vector that is used to displace pixels based on origin at 0
    // to pixels in octree based origin (there is no pixel at the)
    // origin in octrees
    static octomap::point3d ds;
    ds = octomap::point3d(resolution_/2.0, resolution_/2.0, resolution_/2.0);

    // Add padding around thin_bresenham to get thick_bresenham
    const bool lazy_eval = true;
    for (uint i = 0; i < thin_bresenham.size(); i++) {
        for (uint j = 0; j < padding.size(); j++) {
            // Pixels.push_back(thin_bresenham[i] + padding[j] + ds);
            thick_traj_.updateNode(thin_bresenham[i] + padding[j] + ds, true, lazy_eval);
        }
    }

    // Add edge padding
    for (uint j = 0; j < init_edge_padding.size(); j++) {
        // Pixels.push_back(thin_bresenham[0] + init_edge_padding[j] + ds);
        thick_traj_.updateNode(thin_bresenham[0] + init_edge_padding[j] + ds, true, lazy_eval);
    }
    for (uint j = 0; j < final_edge_padding.size(); j++) {
        // Pixels.push_back(thin_bresenham[thin_bresenhamthin_bresenham.size()-1] + final_edge_padding[j] + ds);
        thick_traj_.updateNode(thin_bresenham[thin_bresenham.size()-1] + final_edge_padding[j] + ds, true, lazy_eval);
    }
    // std::cout << init_edge_padding.size() << std::endl;
    // Pixels = padding;
}

void SampledTrajectory3D::ThickTrajToPcl() {
    pcl::PointXYZ point;
    point_cloud_traj_.clear();

    // Iterate through trajectory points
    for (octomap::OcTree::leaf_iterator it = thick_traj_.begin_leafs(),
                                       end = thick_traj_.end_leafs();
                                       it != end; ++it) {
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();
        point_cloud_traj_.push_back(point);
    }
}

void SampledTrajectory3D::CreateKdTree() {
    *cloud_ptr_ = pos_;
    kdtree_pos_.setInputCloud(cloud_ptr_);
}

void SampledTrajectory3D::SortCollisions(const std::vector<octomap::point3d> &colliding_nodes,
                                         std::vector<geometry_msgs::PointStamped> *samples) {
    pcl::PointXYZ search_point;
    int K = 1;  // Search for 1 nearest neighbor
    std::vector<int> point_idx(K);
    std::vector<float> point_sqr_dist(K);
    geometry_msgs::PointStamped sample;
    samples->resize(colliding_nodes.size());

    for (uint i = 0; i < colliding_nodes.size(); i++) {
        search_point.x = colliding_nodes[i].x();
        search_point.y = colliding_nodes[i].y();
        search_point.z = colliding_nodes[i].z();
        kdtree_pos_.nearestKSearch(search_point, K, point_idx, point_sqr_dist);
        sample.point.x = cloud_ptr_->points[point_idx[0]].x;
        sample.point.y = cloud_ptr_->points[point_idx[0]].y;
        sample.point.z = cloud_ptr_->points[point_idx[0]].z;
        sample.header.stamp = ros::Time(time_[point_idx[0]]);
        sample.header.seq = point_idx[0];
        (*samples)[i] = sample;
    }
    std::sort(samples->begin(), samples->end(), ComparePointStamped);
}


void SampledTrajectory3D::TrajVisMarkers(visualization_msgs::MarkerArray* marker_array) {    // Publish occupied nodes
    // Markers: each marker array stores a set of nodes with similar size
    // visualization_msgs::MarkerArray occupiedNodesVis;
    const int tree_depth = thick_traj_.getTreeDepth();
    marker_array->markers.resize(tree_depth+1);
    const ros::Time rostime = ros::Time::now();

    // Set color parameters
    std_msgs::ColorRGBA color;
    color = visualization_functions::Color::Orange();
    color.a = 0.15;

    // Publish all leafs from the tree
    for (octomap::OcTree::leaf_iterator it = thick_traj_.begin_leafs(),
                                       end = thick_traj_.end_leafs();
                                       it != end; ++it) {
        if (thick_traj_.isNodeOccupied(*it)) {
            // Get depth in the tree
            unsigned idx = it.getDepth();
            geometry_msgs::Point point_center;
            point_center.x = it.getX();
            point_center.y = it.getY();
            point_center.z = it.getZ();
            marker_array->markers[idx].points.push_back(point_center);

            // Get color based on height
            // double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
            marker_array->markers[idx].colors.push_back(color);
        }
    }

    // Set marker properties
    for (unsigned i= 0; i < marker_array->markers.size(); ++i) {
        double size = thick_traj_.getNodeSize(i);

        marker_array->markers[i].header.frame_id = "world";
        marker_array->markers[i].header.stamp = rostime;
        marker_array->markers[i].ns = "thick_traj";
        marker_array->markers[i].id = i;
        marker_array->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        marker_array->markers[i].scale.x = size;
        marker_array->markers[i].scale.y = size;
        marker_array->markers[i].scale.z = size;
        marker_array->markers[i].pose.orientation.w = 1.0;

        if (marker_array->markers[i].points.size() > 0)
            marker_array->markers[i].action = visualization_msgs::Marker::ADD;
        else
            marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void SampledTrajectory3D::SamplesVisMarkers(visualization_msgs::MarkerArray* marker_array) {
    // marker_array->markers.resize(1);
    const ros::Time rostime = ros::Time::now();
    visualization_msgs::Marker marker;

    // Set color parameters
    std_msgs::ColorRGBA color;
    color = visualization_functions::Color::Blue();
    color.a = 0.5;

    // Publish all leafs from the tree
    marker.points.resize(pos_.size());
    marker.colors.resize(pos_.size());
    for (uint i = 0; i < pos_.size(); i++) {
        geometry_msgs::Point point;
        point.x = pos_[i].x;
        point.y = pos_[i].y;
        point.z = pos_[i].z;
        marker.points.push_back(point);
        marker.colors.push_back(color);
    }

    // Set marker properties
    marker.header.frame_id = "world";
    marker.header.stamp = rostime;
    marker.ns = "SampledTraj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.pose.orientation.w = 1.0;

    if (marker.points.size() > 0)
        marker.action = visualization_msgs::Marker::ADD;
    else
        marker.action = visualization_msgs::Marker::DELETE;

    marker_array->markers.push_back(marker);
}

void SampledTrajectory3D::CompressedVisMarkers(visualization_msgs::MarkerArray* marker_array) {
    // marker_array->markers.resize(1);
    const ros::Time rostime = ros::Time::now();
    visualization_msgs::Marker marker;

    // Set color parameters
    std_msgs::ColorRGBA color;
    color = visualization_functions::Color::Green();
    color.a = 0.9;

    // Publish all leafs from the tree
    marker.points.resize(pos_.size());
    marker.colors.resize(pos_.size());
    for (uint i = 0; i < compressed_pos_.size(); i++) {
        geometry_msgs::Point Point;
        Point.x = compressed_pos_[i][0];
        Point.y = compressed_pos_[i][1];
        Point.z = compressed_pos_[i][2];
        marker.points.push_back(Point);
        marker.colors.push_back(color);
    }

    // Set marker properties
    marker.header.frame_id = "world";
    marker.header.stamp = rostime;
    marker.ns = "compressedTraj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.pose.orientation.w = 1.0;

    if (marker.points.size() > 0)
        marker.action = visualization_msgs::Marker::ADD;
    else
        marker.action = visualization_msgs::Marker::DELETE;

    marker_array->markers.push_back(marker);
}

// Clear all the data within this object
void SampledTrajectory3D::ClearObject() {
    SampledTrajectory3D newObj();
    this->pos_.clear();
    this->time_.clear();
    this->n_points_ = 0;
    this->compressed_pos_.clear();
    this->compressed_time_.clear();
    this->n_compressed_points_ = 0;
    this->thick_traj_.clear();
    this->point_cloud_traj_.clear();
}

// Return the sample with lowest time
bool ComparePointStamped(const geometry_msgs::PointStamped &sample1,
                         const geometry_msgs::PointStamped &sample2) {
    return sample1.header.stamp.toSec() < sample2.header.stamp.toSec();
}

}  // namespace sampled_traj
