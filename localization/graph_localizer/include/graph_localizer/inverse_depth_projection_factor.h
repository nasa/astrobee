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

#ifndef GRAPH_LOCALIZER_INVERSE_DEPTH_PROJECTION_FACTOR_H_
#define GRAPH_LOCALIZER_INVERSE_DEPTH_PROJECTION_FACTOR_H_

#include <graph_localizer/inverse_depth_measurement.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/optional.hpp>

#include <string>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement. The
 * calibration is known and the landmark point is represented using the inverse depth parameterization.
 * Adapted from gtsam GenericProjectionFactor.
 */
class InverseDepthProjectionFactor : public NoiseModelFactor3<Pose3, Pose3, InverseDepthMeasurement> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;  ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;    ///< If true, rethrows Cheirality exceptions (default:
                            ///< false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions
                            ///< (default: false)

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor3<Pose3, Pose3, InverseDepthMeasurement> Base;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<InverseDepthFactor> shared_ptr;

  /// Default constructor
  InverseDepthProjectionFactor() : measured_(0, 0), throwCheirality_(false), verboseCheirality_(false) {}

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity)
   */
  InverseDepthProjectionFactor(const Point2& measured, const SharedNoiseModel& model, Key poseKey,
                               Key inverseDepthCameraPoseKey, Key inverseDepthMeasurementKey)
      : Base(model, poseKey, inverseDepthCameraPoseKey, inverseDepthMeasurementKey),
        measured_(measured),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor with exception-handling flags
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are
   * rethrown
   * @param verboseCheirality determines whether exceptions are printed for
   * Cheirality
   * @param body_P_sensor is the transform from body to sensor frame  (default
   * identity)
   */
  InverseDepthProjectionFactor(const Point2& measured, const SharedNoiseModel& model, Key poseKey,
                               Key inverseDepthCameraPoseKey, Key inverseDepthMeasurementKey, bool throwCheirality,
                               bool verboseCheirality)
      : Base(model, poseKey, inverseDepthCameraPoseKey, inverseDepthMeasurementKey),
        measured_(measured),
        throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  virtual ~InverseDepthProjectionFactor() {}

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "InverseDepthProjectionFactor, z = ";
    traits<Point2>::Print(measured_);
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose, const Pose3& inverseDepthCameraPose,
                       const InverseDepthMeasurement& inverseDepthMeasurement,
                       boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none) const {
    try {
      return inverseDepthMeasurement.Project(inverseDepthCameraPose, pose, H1, H2) - measured_;
    } catch (CheiralityException& e) {
      if (H0) *H0 = Matrix::Zero(2, 6);
      if (H1) *H1 = Matrix::Zero(2, 1);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark moved behind camera " << DefaultKeyFormatter(this->key()) << std::endl;
      if (throwCheirality_) throw CheiralityException(this->key());
    }
    return Vector2::Constant(0.0);
  }

  /** return the measurement */
  const Point2& measured() const { return measured_; }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measured_);
    ar& BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar& BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// traits
struct traits<InverseDepthProjectionFactor> : public Testable<InverseDepthProjectionFactor> {};

}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_INVERSE_DEPTH_PROJECTION_FACTOR_H_
