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

#ifndef GNC_AUTOCODE_FAM_H_
#define GNC_AUTOCODE_FAM_H_

extern "C" {
#include <fam_force_allocation_module.h>
}

namespace config_reader {
  class ConfigReader;
}

namespace gnc_autocode {

class GncFamAutocode {
 public:
  GncFamAutocode(void);
  ~GncFamAutocode(void);

  virtual void Initialize();
  virtual void Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl);
  virtual void ReadParams(config_reader::ConfigReader* config);

  RT_MODEL_fam_force_allocation_T* fam_;

  ex_time_msg time_;
  cmd_msg cmd_;
  ctl_msg ctl_;
  cmc_msg cmc_;
  act_msg act_;
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_FAM_H_
