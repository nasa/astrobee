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
                                  const SmartProjectionParams& params = SmartProjectionParams(),
                                  const bool rotation_only_fallback = false, const bool robust = true,
                                  const double huber_k = 1.0)
      : SmartProjectionPoseFactor<CALIBRATION>(sharedNoiseModel, K, body_P_sensor, params),
        rotation_only_fallback_(rotation_only_fallback),
        robust_(robust),
        huber_k_(huber_k) {
    // From SmartFactorBase
    if (!sharedNoiseModel) throw std::runtime_error("RobustSmartProjectionPoseFactor: sharedNoiseModel is required");
    SharedIsotropic sharedIsotropic = boost::dynamic_pointer_cast<noiseModel::Isotropic>(sharedNoiseModel);
    if (!sharedIsotropic) throw std::runtime_error("RobustSmartProjectionPoseFactor: needs isotropic");
    noise_inv_sigma_ = 1.0 / sharedIsotropic->sigma();
    triangulation_params_ = params.triangulation;
  }

  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const override {
    typename Base::Cameras cameras = this->cameras(values);
    // if (!this->triangulateForLinearize(cameras)) return boost::make_shared<JacobianFactorSVD<Dim, 2>>(this->keys());
    const auto result = this->triangulateSafe(cameras);
    // Adapted from SmartFactorBase::CreateJacobianSVDFactor
    size_t m = this->keys().size();
    typename Base::FBlocks F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);

    // Handle behind camera result with rotation only factors (see paper)
    // Degenerate result tends to lead to solve failures, so return empty factor in this case
    if (result.valid()) {
      this->computeJacobiansSVD(F, E0, b, cameras, *(this->point()));
    } else if (useForRotationOnly(result)) {  // Rotation only factor
      Unit3 backProjected = cameras[0].backprojectPointAtInfinity(this->measured().at(0));
      this->computeJacobiansSVD(F, E0, b, cameras, backProjected);
    } else {  // Empty factor  // NOLINT
      return boost::make_shared<JacobianFactorSVD<Dim, 2>>(this->keys());
    }
    return createRegularJacobianFactorSVD<Dim, ZDim>(this->keys(), F, E0, b);
  }

  bool useForRotationOnly(const gtsam::TriangulationResult& result) const {
    // Use rotation only for all failure cases
    return true;
  }

  double error(const Values& values) const override {
    if (this->active(values)) {
      try {
        const double total_reprojection_loss = this->totalReprojectionError(this->cameras(values));
        const auto result = this->point();
        if (!result.valid() && !useForRotationOnly(result)) return 0.0;
        // Multiply by 2 since totalReporjectionError divides mahal distance by 2, and robust_model_->loss
        // expects mahal distance
        const double loss = robust_ ? robustLoss(2.0 * total_reprojection_loss) : total_reprojection_loss;
        return loss;
      } catch (...) {
        // Catch cheirality and other errors, zero on errors
        return 0.0;
      }
    } else {  // Inactive
      return 0.0;
    }
  }

  // These "serialized" functions are only needed due to an error in gtsam for serializing result_.
  // Call these instead of error() and point() for a smart factor that has been serialized.
  // TODO(rsoussan): Remove these when gtsam bug fixed
  double serialized_error(const Values& values) const {
    if (this->active(values)) {
      try {
        const auto point = serialized_point(values);
        const double total_reprojection_loss = this->totalReprojectionError(this->cameras(values), point);
        if (!point.valid() && !useForRotationOnly(point)) return 0.0;
        // Multiply by 2 since totalReporjectionError divides mahal distance by 2, and robust_model_->loss
        // expects mahal distance
        const double loss = robust_ ? robustLoss(2.0 * total_reprojection_loss) : total_reprojection_loss;
        return loss;
      } catch (...) {
        // Catch cheirality and other errors, zero on errors
        return 0.0;
      }
    } else {  // Inactive
      return 0.0;
    }
  }

  TriangulationResult serialized_point(const Values& values) const {
    return gtsam::triangulateSafe(this->cameras(values), this->measured(), triangulation_params_);
  }

  bool robust() const { return robust_; }

  double noise_inv_sigma() const { return noise_inv_sigma_; }

  // More efficient implementation of robust loss (also avoids inheritance calls)
  double robustLoss(const double mahal_distance) const {
    const double sqrt_mahal_distance = std::sqrt(mahal_distance);
    const double absError = std::abs(sqrt_mahal_distance);
    if (absError <= huber_k_) {  // |x| <= k
      return mahal_distance / 2;
    } else {  // |x| > k
      return huber_k_ * (absError - (huber_k_ / 2));
    }
  }

 private:
  template <size_t D, size_t ZDim>
  boost::shared_ptr<RegularJacobianFactor<D>> createRegularJacobianFactorSVD(
    const KeyVector& keys,
    const std::vector<Eigen::Matrix<double, ZDim, D>, Eigen::aligned_allocator<Eigen::Matrix<double, ZDim, D>>>&
      Fblocks,
    const Matrix& Enull, const Vector& b) const {
    typedef std::pair<Key, Matrix> KeyMatrix;

    Vector reduced_error = Enull.transpose() * b;
    // Apply noise whitening and robust weighting manually to more efficiently robustify
    // error vector and jacobians.  Equivalent to calling whitenSystem with a robust noise model.
    // Assumes noise is diagonal (required for this factor anyway).
    reduced_error *= noise_inv_sigma_;
    double reduced_matrix_weight = noise_inv_sigma_;
    if (robust_) {
      const double robust_weight = robustWeight(reduced_error.norm());
      reduced_error *= robust_weight;
      reduced_matrix_weight *= robust_weight;
    }

    size_t numKeys = Enull.rows() / ZDim;
    size_t m2 = Enull.cols();
    std::vector<KeyMatrix> reduced_matrices;
    reduced_matrices.reserve(numKeys);
    for (size_t k = 0; k < Fblocks.size(); ++k) {
      Key key = keys[k];
      reduced_matrices.emplace_back(
        KeyMatrix(key, (Enull.transpose()).block(0, ZDim * k, m2, ZDim) * Fblocks[k] * reduced_matrix_weight));
    }

    return boost::make_shared<RegularJacobianFactor<D>>(reduced_matrices, reduced_error);
  }

  // TODO(rsoussan): profile these calls vs. gtsam better, is there a significant improvement?
  // More efficient implementation of robust weight (also avoids inheritance calls)
  double robustWeight(const double error_norm) const {
    const double squared_weight = (error_norm <= huber_k_) ? (1.0) : (huber_k_ / error_norm);
    return std::sqrt(squared_weight);
  }

  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SmartProjectionPoseFactor<CALIBRATION>);
    ar& BOOST_SERIALIZATION_NVP(noise_inv_sigma_);
    ar& BOOST_SERIALIZATION_NVP(huber_k_);
    ar& BOOST_SERIALIZATION_NVP(robust_);
    ar& BOOST_SERIALIZATION_NVP(rotation_only_fallback_);
    ar& BOOST_SERIALIZATION_NVP(triangulation_params_);
  }

  bool rotation_only_fallback_;
  double robust_;
  double huber_k_;
  double noise_inv_sigma_;
  // TODO(rsoussan): Remove once result_ serialization bug in gtsam fixed
  TriangulationParameters triangulation_params_;
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_ROBUST_SMART_PROJECTION_POSE_FACTOR_H_
