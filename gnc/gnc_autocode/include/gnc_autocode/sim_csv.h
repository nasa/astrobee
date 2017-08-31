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

#ifndef GNC_AUTOCODE_SIM_CSV_H_
#define GNC_AUTOCODE_SIM_CSV_H_

#include <gnc_autocode/sim.h>

#include <stdio.h>
#include <string>

typedef struct {
  FILE* act;
  FILE* reg;
  FILE* landmark;
  FILE* optical;
  FILE* cmd;
  FILE* imu;
  FILE* env;
} GncFiles;

namespace gnc_autocode {
class GncSimCSV : public GncSimAutocode {
 public:
  GncSimCSV();
  ~GncSimCSV();
  virtual void Initialize(std::string directory);
  virtual void Step();
 protected:
  void SkipFirstLine(FILE* f);
  void LoadGncFiles(std::string directory);
  int ReadStepState(void);

  GncFiles f_;
  int seconds_, nsec_;
  float last_landmark_sec_, last_landmark_nsec_;
  float last_of_sec_, last_of_nsec_;
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_SIM_CSV_H_
