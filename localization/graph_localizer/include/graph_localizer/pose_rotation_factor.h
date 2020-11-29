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

#ifndef GRAPH_LOCALIZER_POSE_ROTATION_FACTOR_H_
#define GRAPH_LOCALIZER_POSE_ROTATION_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class PoseRotationFactor : public NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  PoseRotationFactor() {}

  PoseRotationFactor(const Rot3& rotation, const SharedNoiseModel& model, Key pose_key1, Key pose_key2)
      : Base(model, pose_key1, pose_key2), rotation_(rotation) {}

  // TODO(rsoussan): Is this necessary?
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "PoseRotationFactor, z = ";
    traits<Point2>::Print(rotation_);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Rot3>::Equals(this->rotation_, e->rotation(), tol);
  }

  Vector evaluateError(const Pose3& pose1, const Pose3& pose2, boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const override {
    const auto& rot1 = pose1.rotation(H1);
    const auto& rot2 = pose2.rotation(H2);
    if (H1) LOG(ERROR) << "starting h1: " << H1->matrix();
    if (H2) LOG(ERROR) << "starting h2: " << H2->matrix();
    // TODO(rsoussan): are these references or copys?????
    boost::optional<Rot3&> H1_rot = H1 ? H1.block<3, 3>(0, 0) : boost::none;
    boost::optional<Rot3&> H2_rot = H2 ? H2.block<3, 3>(0, 0) : boost::none;
    // Adapted from BetweenFactor.h
    const auto relative_rotation = traits<Rot3>::Between(rot1, rot2, H1_rot, H2_rot);
    // manifold equivalent of h(x)-z -> log(z,h(x))
    traits<Rot3>::ChartJacobian::Jacobian Hlocal;
    Vector error = traits<Rot3>::Local(rotation_, relative_rotation, boost::none, (H1_rot || H2_rot) ? &Hlocal : 0);
    if (H1_rot) *H1_rot = Hlocal * (*H1_rot);
    if (H2_rot) *H2_rot = Hlocal * (*H2_rot);
    if (H1) LOG(ERROR) << "ending h1: " << H1->matrix();
    if (H2) LOG(ERROR) << "ending h2: " << H2->matrix();
    return error;
  }

  const Rot3& rotation() const { return rotation_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(rotation_);
  }

  Rot3 rotation_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_POSE_ROTATION_FACTOR_H_
