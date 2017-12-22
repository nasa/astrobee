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

#include <gnc_autocode/ekf_csv.h>

namespace gnc_autocode {

GncEkfCSV::GncEkfCSV(void) : GncEkfAutocode() {}

void GncEkfCSV::Initialize(std::string directory) {
  kfl_file_ = fopen((directory + "/out_kfl_msg.csv").c_str(), "r");
  if (kfl_file_ == NULL) {
    fprintf(stderr, "Failed to open out_kfl_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(kfl_file_);
}

void GncEkfCSV::SkipFirstLine(FILE* f) {
  char c;
  do {
      c = fgetc(f);
  } while (c != '\n');
}

GncEkfCSV::~GncEkfCSV() {
  fclose(kfl_file_);
}

int GncEkfCSV::ReadStepState(void) {
  int ret = fscanf(kfl_file_, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,",
      &kfl_.quat_ISS2B[0], &kfl_.quat_ISS2B[1], &kfl_.quat_ISS2B[2], &kfl_.quat_ISS2B[3],
      &kfl_.omega_B_ISS_B[0], &kfl_.omega_B_ISS_B[1], &kfl_.omega_B_ISS_B[2],
      &kfl_.gyro_bias[0], &kfl_.gyro_bias[1], &kfl_.gyro_bias[2]);
  if (ret != 10) {
    fprintf(stderr, "Failed to read line part 1 in KFL file.\n");
    return 1;
  }
  ret = fscanf(kfl_file_, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%hhu,%hhu,",
      &kfl_.V_B_ISS_ISS[0], &kfl_.V_B_ISS_ISS[1], &kfl_.V_B_ISS_ISS[2],
      &kfl_.A_B_ISS_ISS[0], &kfl_.A_B_ISS_ISS[1], &kfl_.A_B_ISS_ISS[2],
      &kfl_.accel_bias[0], &kfl_.accel_bias[1], &kfl_.accel_bias[2],
      &kfl_.P_B_ISS_ISS[0], &kfl_.P_B_ISS_ISS[1], &kfl_.P_B_ISS_ISS[2],
      &kfl_.confidence, &kfl_.aug_state_enum);
  if (ret != 14) {
    fprintf(stderr, "Failed to read line part 2 in KFL file.\n");
    return 1;
  }
  ret = fscanf(kfl_file_, "%g,%g,%g,%g,%g,%g,%g,",
      &kfl_.ml_quat_ISS2cam[0], &kfl_.ml_quat_ISS2cam[1],
      &kfl_.ml_quat_ISS2cam[2], &kfl_.ml_quat_ISS2cam[3],
      &kfl_.ml_P_cam_ISS_ISS[0], &kfl_.ml_P_cam_ISS_ISS[1], &kfl_.ml_P_cam_ISS_ISS[2]);
  if (ret != 7) {
    fprintf(stderr, "Failed to read line part 3 in KFL file.\n");
    return 1;
  }
  for (int i = 0; i < 5; i++) {
    ret = fscanf(kfl_file_, "%g,%g,%g,%g,",
        &kfl_.of_quat_ISS2cam[i], &kfl_.of_quat_ISS2cam[5 + i],
        &kfl_.of_quat_ISS2cam[10 + i], &kfl_.of_quat_ISS2cam[15 + i]);
    if (ret != 4) {
      fprintf(stderr, "Failed to read line part 4 in KFL file.\n");
      return 1;
    }
  }
  for (int i = 0; i < 5; i++) {
    ret = fscanf(kfl_file_, "%g,%g,%g,", &kfl_.of_P_cam_ISS_ISS[i],
        &kfl_.of_P_cam_ISS_ISS[5 + i], &kfl_.of_P_cam_ISS_ISS[10 + i]);
    if (ret != 3) {
      fprintf(stderr, "Failed to read line part 5 in KFL file.\n");
      return 1;
    }
  }
  for (int i = 0; i < 51; i++) {
    ret = fscanf(kfl_file_, "%lf,", &kfl_.cov_diag[i]);
    if (ret != 1) {
      fprintf(stderr, "Failed to read line part 6 in KFL file.\n");
      return 1;
    }
  }
  ret = fscanf(kfl_file_, "%hu\n", &kfl_.kfl_status);
  if (ret != 1) {
    fprintf(stderr, "Failed to read line part 7 in KFL file.\n");
    return 1;
  }
  return 0;
}


void GncEkfCSV::Step(void) {
  if (ReadStepState())
    exit(0);
}

}  // namespace gnc_autocode

