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

#ifndef GNC_AUTOCODE_EKF_CSV_H_
#define GNC_AUTOCODE_EKF_CSV_H_

#include <gnc_autocode/ekf.h>

#include <stdio.h>
#include <string>

namespace gnc_autocode {

class GncEkfCSV : public GncEkfAutocode {
 public:
  explicit GncEkfCSV(cmc_msg * cmc);
  ~GncEkfCSV(void);

  virtual void Initialize(std::string directory);
  virtual void Step();

  FILE* kfl_file_;
 protected:
  int ReadStepState(void);
  void SkipFirstLine(FILE* f);
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_EKF_CSV_H_
