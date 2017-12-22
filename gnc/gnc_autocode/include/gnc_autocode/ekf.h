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

#ifndef GNC_AUTOCODE_EKF_H_
#define GNC_AUTOCODE_EKF_H_

extern "C" {
#include <est_estimator.h>
}

namespace config_reader {
  class ConfigReader;
}

namespace gnc_autocode {

class GncEkfAutocode {
 public:
  GncEkfAutocode(void);
  ~GncEkfAutocode(void);

  virtual void Initialize();
  virtual void Step();
  virtual void ReadParams(config_reader::ConfigReader* config);

  RT_MODEL_est_estimator_T* est_;
  cvs_landmark_msg vis_;
  cvs_registration_pulse reg_;
  cvs_optical_flow_msg of_;
  cvs_handrail_msg hand_;
  imu_msg imu_;
  real32_T quat_[4];
  // TODO(bcoltin): this needs to be removed by GNC
  cmc_msg cmc_;
  kfl_msg kfl_;
  float* P_;
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_EKF_H_
