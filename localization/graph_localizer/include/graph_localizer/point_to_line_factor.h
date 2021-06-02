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

#include <graph_localizer/point_to_line_factor_base.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {
class PointToLineFactor : public PointToLineFactorBase {
  typedef PointToLineFactorBase Base;
  typedef PointToLineFactor This;

 public:
  PointToLineFactor() {}

  PointToLineFactor(const Point3& sensor_t_point, const Pose3& world_T_line, const Pose3& body_T_sensor,
                    const SharedNoiseModel& model, Key pose_key)
      : Base(sensor_t_point, world_T_line, body_T_sensor, model, pose_key) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PointToLineFactor, z = ";
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override { return Base::equals(p, tol); }

  Vector evaluateError(const Pose3& world_T_body, boost::optional<Matrix&> H = boost::none) const override {
    const auto line_t_point = Base::error(world_T_body, H);
    if (H) {
      // Remove last row as error does not account for z value in line_t_point
      *H = H->topRows(2).eval();
    }
    return line_t_point.head<2>();
  }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POINT_TO_LINE_FACTOR_H_
