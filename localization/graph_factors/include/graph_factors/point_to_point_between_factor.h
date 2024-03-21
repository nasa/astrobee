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

#ifndef GRAPH_FACTORS_POINT_TO_POINT_BETWEEN_FACTOR_H_
#define GRAPH_FACTORS_POINT_TO_POINT_BETWEEN_FACTOR_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PointToPointBetweenFactor : public NoiseModelFactor2<Pose3, Pose3> {
  typedef NoiseModelFactor2<Pose3, Pose3> Base;
  typedef PointToPointBetweenFactor This;

 public:
  PointToPointBetweenFactor() {}

  PointToPointBetweenFactor(const Point3& sensor_t_point_source, const Point3& sensor_t_point_target,
                            const Pose3& body_T_sensor, const SharedNoiseModel& model, Key source_pose_key,
                            Key target_pose_key)
      : Base(model, source_pose_key, target_pose_key),
        sensor_t_point_source_(sensor_t_point_source),
        sensor_t_point_target_(sensor_t_point_target),
        body_T_sensor_(body_T_sensor) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToPointBetweenFactor, z = ";
    traits<Point3>::Print(sensor_t_point_source_);
    traits<Point3>::Print(sensor_t_point_target_);
    traits<Pose3>::Print(body_T_sensor_);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point3>::Equals(this->sensor_t_point_source(), e->sensor_t_point_source(), tol) &&
           traits<Point3>::Equals(this->sensor_t_point_target(), e->sensor_t_point_target(), tol) &&
           traits<Pose3>::Equals(this->body_T_sensor(), e->body_T_sensor(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body_source, const Pose3& world_T_body_target,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const override {
    const auto world_t_point_source = world_T_body_source.transformFrom(body_T_sensor_ * sensor_t_point_source_, H1);
    const auto world_t_point_target = world_T_body_target.transformFrom(body_T_sensor_ * sensor_t_point_target_, H2);
    if (H2) *H2 = -1.0 * (*H2);
    return world_t_point_source - world_t_point_target;
  }

  const Point3& sensor_t_point_source() const { return sensor_t_point_source_; }
  const Point3& sensor_t_point_target() const { return sensor_t_point_target_; }
  const Pose3& body_T_sensor() const { return body_T_sensor_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(sensor_t_point_source_);
    ar& BOOST_SERIALIZATION_NVP(sensor_t_point_target_);
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }

  Point3 sensor_t_point_source_;
  Point3 sensor_t_point_target_;
  Pose3 body_T_sensor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_FACTORS_POINT_TO_POINT_BETWEEN_FACTOR_H_
