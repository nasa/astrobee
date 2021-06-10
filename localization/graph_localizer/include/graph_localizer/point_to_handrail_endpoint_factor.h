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

#ifndef GRAPH_LOCALIZER_POINT_TO_HANDRAIL_ENDPOINT_FACTOR_H_
#define GRAPH_LOCALIZER_POINT_TO_HANDRAIL_ENDPOINT_FACTOR_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PointToHandrailEndpointFactor : public NoiseModelFactor1<Pose3> {
  typedef NoiseModelFactor1<Pose3> Base;
  typedef PointToHandrailEndpointFactor This;

 public:
  PointToHandrailEndpointFactor() {}

  PointToHandrailEndpointFactor(const Point3& sensor_t_point, const Point3& world_t_endpoint_a,
                                const Point3& world_t_endpoint_b, const Pose3& body_T_sensor,
                                const SharedNoiseModel& model, Key pose_key)
      : Base(model, pose_key),
        sensor_t_point_(sensor_t_point),
        world_t_endpoint_a_(world_t_endpoint_a),
        world_t_endpoint_b_(world_t_endpoint_b),
        body_T_sensor_(body_T_sensor) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToHandrailEndpointFactor, z = ";
    traits<Point3>::Print(sensor_t_point_);
    traits<Point3>::Print(world_t_endpoint_a_);
    traits<Point3>::Print(world_t_endpoint_b_);
    traits<Pose3>::Print(body_T_sensor_);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point3>::Equals(this->sensor_t_point(), e->sensor_t_point(), tol) &&
           traits<Point3>::Equals(this->world_t_endpoint_a(), e->world_t_endpoint_a(), tol) &&
           traits<Point3>::Equals(this->world_t_endpoint_b(), e->world_t_endpoint_b(), tol) &&
           traits<Pose3>::Equals(this->body_T_sensor(), e->body_T_sensor(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const override {
    if (H) {
      Matrix66 d_world_T_sensor_d_world_T_body;
      Matrix36 d_world_t_point_d_world_T_sensor;
      Matrix33 d_local_d_world_t_point;
      const auto error = getError(world_T_body, d_world_T_sensor_d_world_T_body, d_world_t_point_d_world_T_sensor,
                                  d_local_d_world_t_point);
      *H = d_local_d_world_t_point * d_world_t_point_d_world_T_sensor * d_world_T_sensor_d_world_T_body;
      return error;
    }
    return getError(world_T_body);
  }

  Vector getError(const Pose3& world_T_body, OptionalJacobian<6, 6> d_world_T_sensor_d_world_T_body = boost::none,
                  OptionalJacobian<3, 6> d_world_t_point_d_world_T_sensor = boost::none,
                  OptionalJacobian<3, 3> d_local_d_world_t_point = boost::none) const {
    const Pose3 world_T_sensor = world_T_body.transformPoseFrom(body_T_sensor_, d_world_T_sensor_d_world_T_body);
    const Point3 world_t_point = world_T_sensor.transformFrom(sensor_t_point_, d_world_t_point_d_world_T_sensor);
    const distance_to_endpoint_a = (world_t_point - world_t_endpoint_a_).norm();
    const distance_to_endpoint_b = (world_t_point - world_t_endpoint_b_).norm();
    // TODO(rsoussan): Find closest endpoint before creating factor and stick to it? Add option to redo correspondence?
    const Point3& world_t_closest_endpoint =
      distance_to_endpoint_a < distance_to_endpoint_b ? world_t_endpoint_a_ : world_t_endpoint_b_;
    const Point3 world_F_point_t_closest_endpoint =
      local(world_t_point, world_t_closest_endpoint, d_local_d_world_t_point);
    return world_F_point_t_closest_endpoint;
  }

  // In gtsam terminology, local is the difference in the tangent space between two Lie group elements.
  // In this case this is simply the difference between the two Point3 elements.
  static Point3 local(const Point3& point1, const Point3& point2,
                      OptionalJacobian<3, 3> d_local_d_point1 = boost::none) const {
    if (d_local_d_point1) {
      *d_local_d_point1 = I_3x3;
    }
    return point1 - point2;
  }

  const Point3& sensor_t_point() const { return sensor_t_point_; }
  const Point3& world_t_endpoint_a() const { return world_t_endpoint_a_; }
  const Point3& world_t_endpoint_b() const { return world_t_endpoint_b_; }
  const Pose3& body_T_sensor() const { return body_T_sensor_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(sensor_t_point_);
    ar& BOOST_SERIALIZATION_NVP(world_t_endpoint_a_);
    ar& BOOST_SERIALIZATION_NVP(world_t_endpoint_b_);
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }

  Point3 sensor_t_point_;
  Point3 world_t_endpoint_a_;
  Point3 world_t_endpoint_b_;
  Pose3 body_T_sensor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POINT_TO_HANDRAIL_ENDPOINT_FACTOR_H_
