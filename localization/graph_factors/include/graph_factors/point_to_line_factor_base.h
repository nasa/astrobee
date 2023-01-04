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

#ifndef GRAPH_FACTORS_POINT_TO_LINE_FACTOR_BASE_H_
#define GRAPH_FACTORS_POINT_TO_LINE_FACTOR_BASE_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PointToLineFactorBase : public NoiseModelFactor1<gtsam::Pose3> {
  typedef NoiseModelFactor1<Pose3> Base;
  typedef PointToLineFactorBase This;

 public:
  PointToLineFactorBase() {}

  // The line in world_T_line is oriented such that the z axis is along the line and is initialized with an arbitrary
  // orientation about the z axis.  body_T_sensor is the sensor extrinsics and assumed to be static.
  PointToLineFactorBase(const Point3& sensor_t_point, const Pose3& world_T_line, const Pose3& body_T_sensor,
                        const SharedNoiseModel& model, Key pose_key)
      : Base(model, pose_key),
        sensor_t_point_(sensor_t_point),
        world_T_line_(world_T_line),
        body_T_sensor_(body_T_sensor),
        sensor_R_body_(body_T_sensor.inverse().rotation().matrix()),
        body_t_sensor_(body_T_sensor_.translation()) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToLineFactorBase, z = ";
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

  Vector error(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const {
    const Pose3 world_T_sensor = world_T_body * body_T_sensor_;
    const Pose3 line_T_sensor = world_T_line_.inverse() * world_T_sensor;
    if (H) {
      // GTSAM equivalent:
      /* gtsam::Matrix H_a;
       const auto world_T_sensor = world_T_body.transformPoseFrom(body_T_sensor_, H_a);
       gtsam::Matrix H_b;
       const auto line_T_sensor = world_T_line_.inverse().transformPoseFrom(world_T_sensor, boost::none, H_b);
         gtsam::Matrix H_c;
         const auto line_t_point = line_T_sensor.transformFrom(sensor_t_point_, H_c);
         H = H_c *H_b* H_a;
         */
      const Matrix3 line_R_sensor = line_T_sensor.rotation().matrix();
      const Matrix A = -1.0 * line_R_sensor *
                       (skewSymmetric(sensor_t_point_) + skewSymmetric(sensor_R_body_ * body_t_sensor_)) *
                       sensor_R_body_;
      const Matrix B = line_R_sensor * sensor_R_body_;
      // TODO(rsoussan): avoid zero initialization
      *H = Eigen::Matrix<double, 3, 6>::Zero();
      *H << A, B;
    }
    return line_T_sensor * sensor_t_point_;
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
    ar& BOOST_SERIALIZATION_NVP(sensor_R_body_);
    ar& BOOST_SERIALIZATION_NVP(body_t_sensor_);
  }

  Point3 sensor_t_point_;
  Pose3 world_T_line_;
  Pose3 body_T_sensor_;
  // Cached for faster Jacobian calculations
  Matrix sensor_R_body_;
  Point3 body_t_sensor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_FACTORS_POINT_TO_LINE_FACTOR_BASE_H_
