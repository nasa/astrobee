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

#ifndef LOCALIZATION_MEASUREMENTS_PLANE_H_
#define LOCALIZATION_MEASUREMENTS_PLANE_H_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

namespace localization_measurements {
class PointNormalPlane {
 public:
  PointNormalPlane(const gtsam::Point3& point, const gtsam::Vector3& normal) : point_(point), normal_(normal) {}
  const gtsam::Point3& point() const { return point_; }
  const gtsam::Vector3& normal() const { return normal_; }

 private:
  gtsam::Point3 point_;
  gtsam::Vector3 normal_;
};

// Plane parameterized by ax+by+cz = d
class GeneralPlane {
 public:
  GeneralPlane(const PointNormalPlane& point_normal_plane) {
    normal_ = point_normal_plane.normal();
    // Since the point (p) of the point normal parameterization is on the plane, the vector between another point on the
    // plane u = (x,y,z) and p is perpendicular to the normal. Thus (u-p) dot normal = 0 and a(x - p_x) + b(y - p_y) +
    // c(z - p_z) = 0.  Thus ax + by + cz = ap_x + bp_y + cp_z = -d. So d = -(ap_x + bp_y + cp_z) = -(normal dot p)
    d_ = -1.0 * normal_.dot(point_normal_plane.point());
  }

  const gtsam::Vector3& normal() const { return normal_; }
  const double& d() const { return d_; }

 private:
  // Contains plane parameters a,b,c
  gtsam::Vector3 normal_;
  double d_;
};

// Normalized version of General Plane.
// Useful for calculating distances from points to plane
class HessianNormalPlane {
 public:
  HessianNormalPlane() = default;
  HessianNormalPlane(const GeneralPlane& general_plane) {
    const double norm = general_plane.normal().norm();
    unit_normal_ = general_plane.normal() / norm;
    constant_ = general_plane.d() / norm;
  }
  const gtsam::Vector3& unit_normal() const { return unit_normal_; }
  const double& constant() const { return constant_; }

 private:
  gtsam::Vector3 unit_normal_;
  double constant_;
};

class Plane : public HessianNormalPlane {
 public:
  Plane() = default;
  Plane(const PointNormalPlane& point_normal_plane) : HessianNormalPlane(point_normal_plane) {}
  Plane(const gtsam::Point3& point, const gtsam::Vector3& normal) : Plane(PointNormalPlane(point, normal)) {}
  double Distance(const gtsam::Point3& point, gtsam::OptionalJacobian<1, 3> d_distance_d_point = boost::none) const {
    const double distance = unit_normal().dot(point) + constant();
    if (d_distance_d_point) {
      *d_distance_d_point = unit_normal();
    }
    return distance;
  }
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_PLANE_H_
