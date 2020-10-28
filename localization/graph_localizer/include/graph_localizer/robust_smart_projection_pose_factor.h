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

#ifndef GRAPH_LOCALIZER_ROBUST_SMART_PROJECTION_POSE_FACTOR_H_
#define GRAPH_LOCALIZER_ROBUST_SMART_PROJECTION_POSE_FACTOR_H_

#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <utility>
#include <vector>

namespace gtsam {
template <class CALIBRATION>
class RobustSmartProjectionPoseFactor : public SmartProjectionPoseFactor<CALIBRATION> {
  typedef PinholePose<CALIBRATION> Camera;
  typedef SmartFactorBase<Camera> Base;
  typedef typename Camera::Measurement Z;
  static const int Dim = traits<Camera>::dimension;  ///< Camera dimension
  static const int ZDim = traits<Z>::dimension;      ///< Measurement dimension

 public:
  // For serialization
  RobustSmartProjectionPoseFactor() {}

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param K (fixed) calibration, assumed to be the same for all cameras
   * @param params parameters for the smart projection factors
   */
  RobustSmartProjectionPoseFactor(const SharedNoiseModel& sharedNoiseModel, const boost::shared_ptr<CALIBRATION> K,
                                  const boost::optional<Pose3> body_P_sensor,
                                  const SmartProjectionParams& params = SmartProjectionParams())
      : SmartProjectionPoseFactor<CALIBRATION>(sharedNoiseModel, K, body_P_sensor, params) {
    // From SmartFactorBase
    if (!sharedNoiseModel) throw std::runtime_error("RobustSmartProjectionPoseFactor: sharedNoiseModel is required");
    SharedIsotropic sharedIsotropic = boost::dynamic_pointer_cast<noiseModel::Isotropic>(sharedNoiseModel);
    if (!sharedIsotropic) throw std::runtime_error("RobustSmartProjectionPoseFactor: needs isotropic");
    noiseModel_ = sharedIsotropic;
    // robust_model_ = graph_localizer::Robust(sharedNoiseModel);
    robust_model_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345 /*Taken from gtsam*/), sharedNoiseModel);
  }

  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const override {
    typename Base::Cameras cameras = this->cameras(values);
    if (!this->triangulateForLinearize(cameras)) return boost::make_shared<JacobianFactorSVD<Dim, 2>>(this->keys());
    // Adapted from SmartFactorBase::CreateJacobianSVDFactor
    size_t m = this->keys().size();
    typename Base::FBlocks F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);
    this->computeJacobiansSVD(F, E0, b, cameras, *(this->point()));
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M - 3, noiseModel_->sigma());
    return createRegularJacobianFactorSVD<Dim, ZDim>(this->keys(), F, E0, b, n);
  }

  double error(const Values& values) const override {
    if (this->active(values)) {
      try {
        // Multiply by 2 since totalReporjectionError divides mahal distance by 2, and robust_model_->loss
        // expects mahal distance
        return robust_model_->loss(2.0 * this->totalReprojectionError(this->cameras(values)));
      } catch (...) {
        // Catch cheirality and other errors, zero on errors
        // TODO(rsoussan): Make as inactive instead of zero?
        return 0.0;
      }
    } else {  // else of active flag
      return 0.0;
    }
  }

 private:
  template <size_t D, size_t ZDim>
  boost::shared_ptr<RegularJacobianFactor<D>> createRegularJacobianFactorSVD(
      const KeyVector& keys,
      const std::vector<Eigen::Matrix<double, ZDim, D>, Eigen::aligned_allocator<Eigen::Matrix<double, ZDim, D>>>&
          Fblocks,
      const Matrix& Enull, const Vector& b, const SharedDiagonal& model = SharedDiagonal()) const {
    typedef std::pair<Key, Matrix> KeyMatrix;

    size_t numKeys = Enull.rows() / ZDim;
    size_t m2 = ZDim * numKeys - 3;  // TODO(gtsam): is this not just Enull.rows()?
    std::vector<Matrix> reduced_matrices;
    reduced_matrices.reserve(numKeys);
    for (size_t k = 0; k < Fblocks.size(); ++k) {
      reduced_matrices.push_back((Enull.transpose()).block(0, ZDim * k, m2, ZDim) * Fblocks[k]);
    }

    Vector robust_reduced_error = Enull.transpose() * b;
    robust_model_->WhitenSystem(reduced_matrices, robust_reduced_error);

    std::vector<KeyMatrix> QF;
    QF.reserve(numKeys);
    for (size_t k = 0; k < Fblocks.size(); ++k) {
      Key key = keys[k];
      QF.push_back(KeyMatrix(key, reduced_matrices[k]));
    }
    return boost::make_shared<RegularJacobianFactor<D>>(QF, robust_reduced_error, model);
  }

  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SmartProjectionPoseFactor<CALIBRATION>);
    ar& BOOST_SERIALIZATION_NVP(noiseModel_);
  }

  SharedIsotropic noiseModel_;
  gtsam::noiseModel::Robust::shared_ptr robust_model_;
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_ROBUST_SMART_PROJECTION_POSE_FACTOR_H_
