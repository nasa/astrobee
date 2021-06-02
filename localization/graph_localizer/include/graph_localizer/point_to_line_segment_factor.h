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

#ifndef GRAPH_LOCALIZER_POINT_TO_LINE_SEGMENT_FACTOR_H_
#define GRAPH_LOCALIZER_POINT_TO_LINE_SEGMENT_FACTOR_H_

#include <graph_localizer/point_to_line_factor_base.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <string>

namespace gtsam {
class PointToLineSegmentFactor : public PointToLineFactorBase {
  typedef PointToLineFactorBase Base;
  typedef PointToLineSegmentFactor This;

 public:
  PointToLineSegmentFactor() {}

  PointToLineSegmentFactor(const Point3& sensor_t_point, const Pose3& world_T_line, const Pose3& body_T_sensor,
                           const double line_length, const SharedNoiseModel& model, Key pose_key)
      : Base(sensor_t_point, world_T_line, body_T_sensor, model, pose_key), line_length_(line_length) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToLineSegmentFactor, z = ";
    std::cout << "line length: " << line_length_;
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return Base::equals(p, tol) && traits<double>::Equals(this->line_length(), e->line_length(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const override {
    const auto line_t_point = Base::error(world_T_body, H);
    // TODO: get line_t_endpoint_a, line_t_endpoint_b using length, see which is closer, use this for z error and zero
    // out jacobian if necessary
  }

  double line_length() const { return line_length_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(line_length_);
  }

  double line_length_;
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POINT_TO_LINE_SEGMENT_FACTOR_H_
