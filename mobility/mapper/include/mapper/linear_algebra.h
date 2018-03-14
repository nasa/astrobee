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

#ifndef MAPPER_LINEAR_ALGEBRA_H_
#define MAPPER_LINEAR_ALGEBRA_H_

#include <msg_conversions/msg_conversions.h>
#include <vector>
#include <string>
#include "mapper/visualization_functions.h"

namespace algebra_3d {

// Line parameterized as l = p0 + t.vec, t belongs to (-inf,inf)
// This class is used in point compression algorithms
class Line3d{
 public:
    Eigen::Vector3d p0_;
    Eigen::Vector3d vec_;

    // Constructor: returns line that goes through two points p1, p2
    Line3d(const Eigen::Vector3d &p1,
            const Eigen::Vector3d &p2) {
        p0_ = p1;
        vec_ = p1 - p2;
    }

    // Methods
    // Calculate the distance between one point and the line
    void DistancePoint2Line(const Eigen::Vector3d &point,
                            double *dist) {
        // Two points on the line
        const Eigen::Vector3d x1 = p0_;
        const Eigen::Vector3d x2 = p0_ + vec_;

        // Optimal t is the t at which the point is closest to the line
        // t_opt = (x1-x2)'*(x1-p)/norm(x1-x2)^2;
        static double t_opt;
        if (x1 == x2) {
            t_opt = 0;
        } else {
            const double gain = 1.0/(pow((x1-x2).norm(), 2.0));
            t_opt = gain*(x1-x2).transpose()*(x1-point);
        }

        // p_opt is the closest point between the point and the line
        const Eigen::Vector3d p_opt = x1 + t_opt*vec_;

        *dist = (point - p_opt).norm();
    }
};

class Plane3d{
 public:
    Eigen::Vector3d origin_;
    Eigen::Vector3d normal_;

    // Constructor: plane given by an origin and a normal
    Plane3d(const Eigen::Vector3d &origin,
            const Eigen::Vector3d &normal) {
        origin_ = origin;
        normal_ = normal;
        if (normal_.norm() == 0) {
            std::cout << "Warning: plane ill-defined: "
                      << "zero-norm normal vector!"
                      << std::endl;
        }
    }
    // Plane given by three points.
    // Normal is in the direction of (p2-p1)x(p3-p1)
    Plane3d(const Eigen::Vector3d &p1,
            const Eigen::Vector3d &p2,
            const Eigen::Vector3d &p3) {
        const Eigen::Vector3d v1 = p2 - p1;
        const Eigen::Vector3d v2 = p3 - p1;
        // const Eigen::Vector3d v3 = (p1 + p2 + p3)/3.0;
        if ((v1.norm() == 0) || (v2.norm() == 0) || ((p2-p3).norm() == 0)) {
            std::cout << "Warning: plane ill-defined: "
                      << "three distinct points are needed!"
                      << std::endl;
            origin_ = p1;
            normal_ << 0.0, 0.0, 0.0;
        } else {
            origin_ = p1;
            normal_ = v1.cross(v2).normalized();
        }
    }

    Plane3d() {}

    void transformPlane(const Eigen::Affine3d &transform,
                        Plane3d *transformedPlane) {
        transformedPlane->origin_ = transform*origin_;
        transformedPlane->normal_ = (transform.linear()*normal_).normalized();
    }
};

class FrustumPlanes{
 public:
    Plane3d left_plane_;
    Plane3d right_plane_;
    Plane3d up_plane_;
    Plane3d down_plane_;
    Eigen::Vector3d UL_, UR_, DL_, DR_, origin_;

    FrustumPlanes(const double fov,
                  const double aspectRatio) {
        double znear = 1.0;
        double hh = tan(fov/2.0);
        double hw = -hh*aspectRatio;
        double normalize = 1.0/Eigen::Vector3d(hw, hh, znear).norm();
        UL_ = normalize*Eigen::Vector3d(-hw, hh, znear);
        UR_ = normalize*Eigen::Vector3d(hw, hh, znear);
        DL_ = normalize*Eigen::Vector3d(-hw, -hh, znear);
        DR_ = normalize*Eigen::Vector3d(hw, -hh, znear);
        origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        left_plane_ = Plane3d(origin_, DL_, UL_);
        right_plane_ = Plane3d(origin_, UR_, DR_);
        up_plane_ = Plane3d(origin_, UL_, UR_);
        down_plane_ = Plane3d(origin_, DR_, DL_);
    }

    FrustumPlanes(const double fx,
                  const double fy,
                  const double cx,
                  const double cy,
                  const uint32_t width,
                  const uint32_t height) {
        double znear = 1.0;
        double x0 = 0.0, y0 = 0.0;
        double xf = static_cast<float>(width) - 1;
        double yf = static_cast<float>(height) - 1;
        double XL = -znear*(x0-cx)/fx;  // X left
        double XR = -znear*(xf-cx)/fx;  // X right
        double YU = -znear*(y0-cy)/fy;  // Y up
        double YD = -znear*(yf-cy)/fy;  // Y down
        UL_ = Eigen::Vector3d(XL, YU, znear).normalized();
        UR_ = Eigen::Vector3d(XR, YU, znear).normalized();
        DL_ = Eigen::Vector3d(XL, YD, znear).normalized();
        DR_ = Eigen::Vector3d(XR, YD, znear).normalized();
        origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        left_plane_ = Plane3d(origin_, DL_, UL_);
        right_plane_ = Plane3d(origin_, UR_, DR_);
        up_plane_ = Plane3d(origin_, UL_, UR_);
        down_plane_ = Plane3d(origin_, DR_, DL_);
    }

    FrustumPlanes() {}

    void TransformFrustum(const Eigen::Affine3d &transform,
                          FrustumPlanes *transformed_frustum) {
        left_plane_.transformPlane(transform, &transformed_frustum->left_plane_);
        right_plane_.transformPlane(transform, &transformed_frustum->right_plane_);
        up_plane_.transformPlane(transform, &transformed_frustum->up_plane_);
        down_plane_.transformPlane(transform, &transformed_frustum->down_plane_);
        transformed_frustum->origin_ = transform*origin_;
        transformed_frustum->UL_ = transform*UL_;
        transformed_frustum->UR_ = transform*UR_;
        transformed_frustum->DL_ = transform*DL_;
        transformed_frustum->DR_ = transform*DR_;
    }

    bool IsPointWithinFrustum(const Eigen::Vector3d &pt) const {
        if ((pt - left_plane_.origin_).dot(left_plane_.normal_) < 0) {
            return false;
        } else if ((pt - right_plane_.origin_).dot(right_plane_.normal_) < 0) {
            return false;
        } else if ((pt - up_plane_.origin_).dot(up_plane_.normal_) < 0) {
            return false;
        } else if ((pt - down_plane_.origin_).dot(down_plane_.normal_) < 0) {
            return false;
        } else {
            return true;
        }
    }

    // Return visualization markers frustum visualization
    void VisualizeFrustum(const std::string &frame_id,
                          visualization_msgs::Marker *line_list) {
        // Initialize array
        line_list->header.frame_id = frame_id;
        line_list->header.stamp = ros::Time::now();
        line_list->ns = "fustrum/" + frame_id;
        line_list->action = visualization_msgs::Marker::ADD;
        line_list->pose.orientation.w = 1.0;
        line_list->type = visualization_msgs::Marker::LINE_LIST;
        line_list->id = 0;
        line_list->scale.x = 0.01;  // Line width
        line_list->color = visualization_functions::Color::Red();
        line_list->lifetime = ros::Duration(1);  // Disappears in 1 second

        std::vector<Eigen::Vector3d> points;
        points.push_back(origin_);
        points.push_back(UL_);
        points.push_back(UR_);
        points.push_back(DL_);
        points.push_back(DR_);

        geometry_msgs::Point node1, node2;
        for (uint i = 0; i < points.size(); i++) {
            for (uint j = 0; j < points.size(); j++) {
                if (i == j) {
                    continue;
                }
                node1 = msg_conversions::eigen_to_ros_point(points[i]);
                node2 = msg_conversions::eigen_to_ros_point(points[j]);
                line_list->points.push_back(node1);
                line_list->points.push_back(node2);
            }
        }
    }
};

}  // namespace algebra_3d

#endif  // MAPPER_LINEAR_ALGEBRA_H_
