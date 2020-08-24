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

#include <imu_integration/imu_integrator.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>

#include <glog/logging.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
ImuIntegrator::ImuIntegrator(const Eigen::Isometry3d& body_T_imu, const Eigen::Vector3d& gravity)
    : body_T_imu_(body_T_imu) {
  DLOG(INFO) << "ImuIntegrator: Gravity vector: " << std::endl << gravity.matrix();
  pim_params_.reset(new gtsam::PreintegratedCombinedMeasurements::Params(gravity));
  // Set sensor covariances
  pim_params_->gyroscopeCovariance = kGyroSigma_ * kGyroSigma_ * gtsam::I_3x3;
  pim_params_->accelerometerCovariance = kAccelSigma_ * kAccelSigma_ * gtsam::I_3x3;
  pim_params_->integrationCovariance = 0.0001 * gtsam::I_3x3;
  // Set bias random walk covariances
  pim_params_->biasAccCovariance = kAccelBiasSigma_ * kAccelBiasSigma_ * gtsam::I_3x3;
  pim_params_->biasOmegaCovariance = kGyroBiasSigma_ * kGyroBiasSigma_ * gtsam::I_3x3;
  // Set bias covariance used for pim integration
  pim_params_->biasAccOmegaInt = 0.0001 * gtsam::I_6x6;
  // Set imu calibration relative pose
  pim_params_->setBodyPSensor(lc::GtPose(body_T_imu_));
}

void ImuIntegrator::BufferImuMeasurement(const lm::ImuMeasurement& imu_measurement) {
  measurements_.emplace(imu_measurement.timestamp, imu_measurement);
}

lc::Time ImuIntegrator::IntegrateImuMeasurements(const lc::Time start_time, const lc::Time end_time,
                                                 gtsam::PreintegratedCombinedMeasurements& pim) const {
  if (measurements_.size() < 2) {
    LOG(FATAL) << "IntegrateImuMeasurements: Less than 2 measurements available.";
  }
  if (end_time > measurements_.crbegin()->first) {
    LOG(FATAL) << "IntegrateImuMeasurements: End time occurs after last measurement.";
  }
  if (start_time > end_time) {
    LOG(FATAL) << "IntegrateImuMeasurements: Start time occurs after end time.";
  }

  // Start with least upper bound or equal measurement
  auto measurement_it = measurements_.lower_bound(start_time);
  lc::Time last_added_imu_measurement_time = start_time;
  int num_measurements_added = 0;
  for (; measurement_it != measurements_.cend() && measurement_it->first <= end_time; ++measurement_it) {
    AddMeasurement(measurement_it->second, last_added_imu_measurement_time, pim);
    ++num_measurements_added;
  }

  // Add final interpolated measurement if necessary
  if (last_added_imu_measurement_time != end_time) {
    const auto interpolated_measurement =
        Interpolate(std::prev(measurement_it)->second, measurement_it->second, end_time);
    AddMeasurement(interpolated_measurement, last_added_imu_measurement_time, pim);
    ++num_measurements_added;
  }

  DLOG(INFO) << "IntegrateImuMeasurements: Num imu measurements integrated: " << num_measurements_added;
  DLOG(INFO) << "IntegrateImuMeasurements: Total Num Imu Measurements after "
                "integrating: "
             << measurements_.size();
  return last_added_imu_measurement_time;
}

void ImuIntegrator::RemoveOldMeasurements(const lc::Time new_start_time) {
  for (auto measurement_it = measurements_.cbegin();
       measurement_it != measurements_.cend() && measurement_it->first < new_start_time;) {
    measurement_it = measurements_.erase(measurement_it);
  }
}

gtsam::PreintegratedCombinedMeasurements ImuIntegrator::IntegratedPim(
    const gtsam::imuBias::ConstantBias& bias, const lc::Time start_time, const lc::Time end_time,
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params) const {
  auto pim = Pim(bias, params);
  IntegrateImuMeasurements(start_time, end_time, pim);
  return pim;
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> ImuIntegrator::pim_params() const {
  // Make a copy so internal params aren't exposed.  Gtsam expects a point to a
  // non const type, so can't pass a pointer to const.
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> copied_params(
      new gtsam::PreintegratedCombinedMeasurements::Params(*pim_params_));
  return copied_params;
}

lc::Time ImuIntegrator::OldestTime() const {
  if (Empty()) {
    LOG(FATAL) << "OldestTime: No measurements available.";
  }
  return measurements_.cbegin()->first;
}

lc::Time ImuIntegrator::LatestTime() const {
  if (Empty()) {
    LOG(FATAL) << "LatestTime: No measurements available.";
  }
  return measurements_.crbegin()->first;
}

lm::ImuMeasurement LatestMeasurement() const {
  if (Empty()) {
    LOG(FATAL) << "LatestTime: No measurements available.";
  }
  return measurements_.crbegin()->second;
}

bool ImuIntegrator::Empty() const { return measurements_.empty(); }

int ImuIntegrator::Size() const { return measurements_.size(); }

}  // namespace imu_integration
