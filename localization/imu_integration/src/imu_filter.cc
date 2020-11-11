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
#include <imu_integration/butterworth_lowpass_filter.h>
#include <imu_integration/butterworth_lowpass_filter_3rd_order.h>
#include <imu_integration/butterworth_lowpass_filter_5th_order.h>
#include <imu_integration/butterworth_lowpass_filter_5th_order_05.h>
#include <imu_integration/butterworth_lowpass_filter_5th_order_1.h>
#include <imu_integration/identity_filter.h>
#include <imu_integration/imu_filter.h>

#include <glog/logging.h>

namespace imu_integration {
namespace lm = localization_measurements;
ImuFilter::ImuFilter(const ImuFilterParams& params) {
  // TODO(rsoussan): Do this more efficiently
  if (params.type == "butter") {
    VLOG(2) << "ImuFilter: Using Butterworth lowpass filter.";
    acceleration_x_filter_.reset(new ButterworthLowpassFilter());
    acceleration_y_filter_.reset(new ButterworthLowpassFilter());
    acceleration_z_filter_.reset(new ButterworthLowpassFilter());
    angular_velocity_x_filter_.reset(new ButterworthLowpassFilter());
    angular_velocity_y_filter_.reset(new ButterworthLowpassFilter());
    angular_velocity_z_filter_.reset(new ButterworthLowpassFilter());
  } else if (params.type == "butter3") {
    VLOG(2) << "ImuFilter: Using Butterworth lowpass 3rd order filter.";
    acceleration_x_filter_.reset(new ButterworthLowpassFilter3rdOrder());
    acceleration_y_filter_.reset(new ButterworthLowpassFilter3rdOrder());
    acceleration_z_filter_.reset(new ButterworthLowpassFilter3rdOrder());
    angular_velocity_x_filter_.reset(new ButterworthLowpassFilter3rdOrder());
    angular_velocity_y_filter_.reset(new ButterworthLowpassFilter3rdOrder());
    angular_velocity_z_filter_.reset(new ButterworthLowpassFilter3rdOrder());
  } else if (params.type == "butter5") {
    VLOG(2) << "ImuFilter: Using Butterworth lowpass 5th order filter.";
    acceleration_x_filter_.reset(new ButterworthLowpassFilter5thOrder());
    acceleration_y_filter_.reset(new ButterworthLowpassFilter5thOrder());
    acceleration_z_filter_.reset(new ButterworthLowpassFilter5thOrder());
    angular_velocity_x_filter_.reset(new ButterworthLowpassFilter5thOrder());
    angular_velocity_y_filter_.reset(new ButterworthLowpassFilter5thOrder());
    angular_velocity_z_filter_.reset(new ButterworthLowpassFilter5thOrder());
  } else if (params.type == "butter5_1") {
    VLOG(2) << "ImuFilter: Using Butterworth lowpass 5th order 1Hz cutoff filter.";
    acceleration_x_filter_.reset(new ButterworthLowpassFilter5thOrder1());
    acceleration_y_filter_.reset(new ButterworthLowpassFilter5thOrder1());
    acceleration_z_filter_.reset(new ButterworthLowpassFilter5thOrder1());
    angular_velocity_x_filter_.reset(new ButterworthLowpassFilter5thOrder1());
    angular_velocity_y_filter_.reset(new ButterworthLowpassFilter5thOrder1());
    angular_velocity_z_filter_.reset(new ButterworthLowpassFilter5thOrder1());
  } else if (params.type == "butter5_05") {
    VLOG(2) << "ImuFilter: Using Butterworth lowpass 5th order 0.5Hz cutoff filter.";
    acceleration_x_filter_.reset(new ButterworthLowpassFilter5thOrder05());
    acceleration_y_filter_.reset(new ButterworthLowpassFilter5thOrder05());
    acceleration_z_filter_.reset(new ButterworthLowpassFilter5thOrder05());
    angular_velocity_x_filter_.reset(new ButterworthLowpassFilter5thOrder05());
    angular_velocity_y_filter_.reset(new ButterworthLowpassFilter5thOrder05());
    angular_velocity_z_filter_.reset(new ButterworthLowpassFilter5thOrder05());
  } else if (params.type == "none") {
    VLOG(2) << "ImuFilter: No filter.";
    acceleration_x_filter_.reset(new IdentityFilter());
    acceleration_y_filter_.reset(new IdentityFilter());
    acceleration_z_filter_.reset(new IdentityFilter());
    angular_velocity_x_filter_.reset(new IdentityFilter());
    angular_velocity_y_filter_.reset(new IdentityFilter());
    angular_velocity_z_filter_.reset(new IdentityFilter());
  } else {
    LOG(FATAL) << "ImuFilter: Invalid filter selection.";
  }
}

boost::optional<lm::ImuMeasurement> ImuFilter::AddMeasurement(const lm::ImuMeasurement& imu_measurement) {
  const double filtered_acceleration_x = acceleration_x_filter_->AddValue(imu_measurement.acceleration.x());
  const double filtered_acceleration_y = acceleration_y_filter_->AddValue(imu_measurement.acceleration.y());
  const double filtered_acceleration_z = acceleration_z_filter_->AddValue(imu_measurement.acceleration.z());
  const double filtered_angular_velocity_x = angular_velocity_x_filter_->AddValue(imu_measurement.angular_velocity.x());
  const double filtered_angular_velocity_y = angular_velocity_y_filter_->AddValue(imu_measurement.angular_velocity.y());
  const double filtered_angular_velocity_z = angular_velocity_z_filter_->AddValue(imu_measurement.angular_velocity.z());

  // Use original timestamp
  // TODO(rsoussan): incorporate phase delay into timestamp?
  auto filtered_imu_measurement = imu_measurement;
  filtered_imu_measurement.acceleration << filtered_acceleration_x, filtered_acceleration_y, filtered_acceleration_z;
  filtered_imu_measurement.angular_velocity << filtered_angular_velocity_x, filtered_angular_velocity_y,
    filtered_angular_velocity_z;
  return filtered_imu_measurement;
}
}  // namespace imu_integration
