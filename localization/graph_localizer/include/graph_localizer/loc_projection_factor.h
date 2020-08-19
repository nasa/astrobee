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

#ifndef GRAPH_LOCALIZER_LOC_PROJECTION_FACTOR_H_
#define GRAPH_LOCALIZER_LOC_PROJECTION_FACTOR_H_

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/optional.hpp>

#include <string>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement. The
 * calibration and landmark point are known here. Adapted from gtsam
 * GenericProjectionFactor.
 */
template <class POSE = Pose3, class LANDMARK = Point3, class CALIBRATION = Cal3_S2>
class LocProjectionFactor : public NoiseModelFactor1<POSE> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;                      ///< 2D measurement
  LANDMARK landmark_point_;              ///< Landmark point
  boost::shared_ptr<CALIBRATION> K_;     ///< shared pointer to calibration object
  boost::optional<POSE> body_P_sensor_;  ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;    ///< If true, rethrows Cheirality exceptions (default:
                            ///< false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions
                            ///< (default: false)

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor1<POSE> Base;

  /// shorthand for this class
  typedef LocProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  LocProjectionFactor()
      : measured_(0, 0), landmark_point_(0, 0, 0), throwCheirality_(false), verboseCheirality_(false) {}

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
  LocProjectionFactor(const Point2 &measured, const LANDMARK &landmark_point, const SharedNoiseModel &model,
                      Key poseKey, const boost::shared_ptr<CALIBRATION> &K,
                      boost::optional<POSE> body_P_sensor = boost::none)
      : Base(model, poseKey),
        measured_(measured),
        landmark_point_(landmark_point),
        K_(K),
        body_P_sensor_(body_P_sensor),
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
  LocProjectionFactor(const Point2 &measured, const LANDMARK &landmark_point, const SharedNoiseModel &model,
                      Key poseKey, const boost::shared_ptr<CALIBRATION> &K, bool throwCheirality,
                      bool verboseCheirality, boost::optional<POSE> body_P_sensor = boost::none)
      : Base(model, poseKey),
        measured_(measured),
        landmark_point_(landmark_point),
        K_(K),
        body_P_sensor_(body_P_sensor),
        throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  virtual ~LocProjectionFactor() {}

  /// @return a deep copy of this factor
  virtual NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string &s = "", const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "LocProjectionFactor, z = ";
    traits<Point2>::Print(measured_);
    std::cout << " landmark_point = ";
    traits<LANDMARK>::Print(landmark_point_);
    if (this->body_P_sensor_) this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor &p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol) &&
           traits<LANDMARK>::Equals(this->landmark_point_, e->landmark_point_, tol) && this->K_->equals(*e->K_, tol) &&
           ((!body_P_sensor_ && !e->body_P_sensor_) ||
            (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H1 = boost::none) const {
    try {
      if (body_P_sensor_) {
        if (H1) {
          Matrix H0;
          PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
          Point2 reprojectionError(camera.project(landmark_point_, H1, boost::none, boost::none) - measured_);
          *H1 = *H1 * H0;
          return reprojectionError;
        } else {
          PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
          return camera.project(landmark_point_, H1, boost::none, boost::none) - measured_;
        }
      } else {
        PinholeCamera<CALIBRATION> camera(pose, *K_);
        return camera.project(landmark_point_, H1, boost::none, boost::none) - measured_;
      }
    } catch (CheiralityException &e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark moved behind camera " << DefaultKeyFormatter(this->key()) << std::endl;
      if (throwCheirality_) throw CheiralityException(this->key());
    }
    return Vector2::Constant(2.0 * K_->fx());
  }

  /** return the measurement */
  const Point2 &measured() const { return measured_; }

  const LANDMARK &landmark_point() const { return landmark_point_; }

  /** return the calibration object */
  inline const boost::shared_ptr<CALIBRATION> calibration() const { return K_; }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(measured_);
    ar &BOOST_SERIALIZATION_NVP(landmark_point_);
    ar &BOOST_SERIALIZATION_NVP(K_);
    ar &BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar &BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar &BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// traits
template <class POSE, class LANDMARK, class CALIBRATION>
struct traits<LocProjectionFactor<POSE, LANDMARK, CALIBRATION>>
    : public Testable<LocProjectionFactor<POSE, LANDMARK, CALIBRATION>> {};

}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_LOC_PROJECTION_FACTOR_H_
