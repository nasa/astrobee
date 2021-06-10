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

#ifndef GRAPH_LOCALIZER_POINT_TO_PLANE_FACTOR_H_
#define GRAPH_LOCALIZER_POINT_TO_PLANE_FACTOR_H_

#include <localization_measurements/plane.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PointToPlaneFactor : public NoiseModelFactor1<Pose3> {
  typedef NoiseModelFactor1<Pose3> Base;
  typedef PointToPlaneFactor This;

 public:
  PointToPlaneFactor() {}

  PointToPlaneFactor(const Point3& sensor_t_point, const localization_measurements::Plane& world_T_plane,
                     const Pose3& body_T_sensor, const SharedNoiseModel& model, Key pose_key)
      : Base(model, pose_key),
        sensor_t_point_(sensor_t_point),
        world_T_plane_(world_T_plane),
        body_T_sensor_(body_T_sensor) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToPlaneFactor, z = ";
    traits<Point3>::Print(sensor_t_point_);
    // TODO(rsoussan): add print for plane!
    // traits<Pose3>::Print(world_T_plane_);
    traits<Pose3>::Print(body_T_sensor_);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point3>::Equals(this->sensor_t_point(), e->sensor_t_point(), tol) &&
           // TODO(rsoussan): add this!
           // traits<Pose3>::Equals(this->world_T_plane(), e->world_T_plane(), tol) &&
           traits<Pose3>::Equals(this->body_T_sensor(), e->body_T_sensor(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const override {
    if (H) {
      Matrix66 d_world_T_sensor_d_world_T_body;
      Matrix36 d_world_t_point_d_world_T_sensor;
      Matrix13 d_distance_d_world_t_point;
      const auto error = getError(world_T_body, d_world_T_sensor_d_world_T_body, d_world_t_point_d_world_T_sensor,
                                  d_distance_d_world_t_point);
      *H = d_distance_d_world_t_point * d_world_t_point_d_world_T_sensor * d_world_T_sensor_d_world_T_body;
      return error;
    }
    return getError(world_T_body);
  }

  Vector getError(const Pose3& world_T_body, OptionalJacobian<6, 6> d_world_T_sensor_d_world_T_body = boost::none,
                  OptionalJacobian<3, 6> d_world_t_point_d_world_T_sensor = boost::none,
                  OptionalJacobian<1, 3> d_distance_d_world_t_point = boost::none) const {
    const Pose3 world_T_sensor = world_T_body.transformPoseFrom(body_T_sensor_, d_world_T_sensor_d_world_T_body);
    const Point3 world_t_point = world_T_sensor.transformFrom(sensor_t_point_, d_world_t_point_d_world_T_sensor);
    const double distance = world_T_plane_.Distance(world_t_point, d_distance_d_world_t_point);
    Vector error(1);
    error << distance;
    return error;
  }

  const Point3& sensor_t_point() const { return sensor_t_point_; }
  const localization_measurements::Plane& world_T_plane() const { return world_T_plane_; }
  const Pose3& body_T_sensor() const { return body_T_sensor_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(sensor_t_point_);
    // TODO(rsoussan): add this!
    // ar& BOOST_SERIALIZATION_NVP(world_T_plane_);
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }

  Point3 sensor_t_point_;
  localization_measurements::Plane world_T_plane_;
  Pose3 body_T_sensor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POINT_TO_PLANE_FACTOR_H_
