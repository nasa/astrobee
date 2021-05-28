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

#ifndef GRAPH_LOCALIZER_POINT_TO_LINE_FACTOR_H_
#define GRAPH_LOCALIZER_POINT_TO_LINE_FACTOR_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PointToLineFactor : public NoiseModelFactor1<gtsam::Pose3> {
  typedef NoiseModelFactor1<Pose3> Base;
  typedef PointToLineFactor This;

 public:
  PointToLineFactor() {}

  // The line in world_T_line is oriented such that the z axis is along the line and is initialized with an arbitrary
  // orientation about the z axis.  body_T_sensor is the sensor extrinsics and assumed to be static.
  PointToLineFactor(const Point3& sensor_t_point, const Pose3& world_T_line, const Pose3& body_T_sensor,
                    const SharedNoiseModel& model, Key pose_key)
      : Base(model, pose_key),
        sensor_t_point_(sensor_t_point),
        world_T_line_(world_T_line),
        body_T_sensor_(body_T_sensor) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToLineFactor, z = ";
    traits<Point3>::Print(sensor_t_point_);
    traits<Pose3>::Print(world_T_line_);
    traits<Pose3>::Print(body_T_sensor_);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point3>::Equals(this->sensor_t_point(), e->sensor_t_point(), tol) &&
           traits<Pose3>::Equals(this->world_T_line(), e->world_T_line(), tol) &&
           traits<Pose3>::Equals(this->body_T_sensor(), e->body_T_sensor(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const override {
    // TODO: store line_T_world in cache
    const Point3 line_t_point = world_T_line_.inverse() * world_T_body * body_T_sensor_ * sensor_t_point_;
    if (H) {
      // TODO: store this as well
      const Pose3 sensor_T_body = body_T_sensor_.inverse();
      const Pose3 line_T_sensor = world_T_line_.inverse() * world_T_body * body_T_sensor_;
      const Matrix3 line_R_sensor = line_T_sensor.rotation().matrix();
      const Matrix3 sensor_R_body = sensor_T_body.rotation().matrix();
      const Point3& body_t_sensor = body_T_sensor_.translation();
      const Matrix A = -1.0 * line_R_sensor * skewSymmetric(sensor_t_point_) * sensor_R_body;
      const Matrix B = line_R_sensor * (I_3x3 + -1.0 * skewSymmetric(sensor_R_body * body_t_sensor) * sensor_R_body);
      // Remove last row as error does not account for z value in line_t_point
      // TODO(rsoussan): avoid zero initialization
      *H = Eigen::Matrix<double, 2, 6>::Zero();
      *H << A.block<2, 3>(0, 0), B.block<2, 3>(0, 0);
    }
    return line_t_point.head<2>();
  }

  const Point3& sensor_t_point() const { return sensor_t_point_; }
  const Pose3& world_T_line() const { return world_T_line_; }
  const Pose3& body_T_sensor() const { return body_T_sensor_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(sensor_t_point_);
    ar& BOOST_SERIALIZATION_NVP(world_T_line_);
    ar& BOOST_SERIALIZATION_NVP(body_T_sensor_);
  }

  Point3 sensor_t_point_;
  Pose3 world_T_line_;
  Pose3 body_T_sensor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POINT_TO_LINE_FACTOR_H_
