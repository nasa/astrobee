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

#include <gnc_autocode/sim_csv.h>

namespace gnc_autocode {

GncSimCSV::GncSimCSV(void) : GncSimAutocode() {
  seconds_ = 0;
  nsec_ = 0;
  last_landmark_sec_ = 0;
  last_landmark_nsec_ = 0;
  last_of_sec_ = 0;
  last_of_nsec_ = 0;
}

void GncSimCSV::Initialize(std::string directory) {
  LoadGncFiles(directory);
}

GncSimCSV::~GncSimCSV() {
  fclose(f_.act);
  fclose(f_.reg);
  fclose(f_.landmark);
  fclose(f_.optical);
  fclose(f_.cmd);
  fclose(f_.imu);
  fclose(f_.env);
}

void GncSimCSV::SkipFirstLine(FILE* f) {
  char c;
  do {
      c = fgetc(f);
  } while (c != '\n');
}

void GncSimCSV::LoadGncFiles(std::string directory) {
  f_.act = fopen((directory + "/out_act_msg.csv").c_str(), "r");
  if (f_.act == NULL) {
    fprintf(stderr, "Failed to open out_act_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.act);

  f_.reg = fopen((directory + "/out_cvs_reg_pulse.csv").c_str(), "r");
  if (f_.reg == NULL) {
    fprintf(stderr, "Failed to open out_cvs_reg_pulse.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.reg);

  f_.landmark = fopen((directory + "/out_cvs_landmark_msg.csv").c_str(), "r");
  if (f_.landmark == NULL) {
    fprintf(stderr, "Failed to open out_cvs_landmark_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.landmark);

  f_.optical = fopen((directory + "/out_cvs_optflow_msg.csv").c_str(), "r");
  if (f_.optical == NULL) {
    fprintf(stderr, "Failed to open out_cvs_optflow_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.optical);

  f_.cmd = fopen((directory + "/out_cmd_msg.csv").c_str(), "r");
  if (f_.cmd == NULL) {
    fprintf(stderr, "Failed to open out_cmd_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.cmd);

  f_.imu = fopen((directory + "/out_imu_msg.csv").c_str(), "r");
  if (f_.imu == NULL) {
    fprintf(stderr, "Failed to open out_imu_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.imu);

  f_.env = fopen((directory + "/out_env_msg.csv").c_str(), "r");
  if (f_.env == NULL) {
    fprintf(stderr, "Failed to open out_env_msg.csv\n");
    exit(1);
  }
  SkipFirstLine(f_.env);
}

int GncSimCSV::ReadStepState(void) {
  float ignore1 = 0.0, ignore2 = 0.0;
  act_msg_.act_timestamp_sec = seconds_;
  act_msg_.act_timestamp_nsec = nsec_;
  /*
  int ret = fscanf(f_.act, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
      &ignore1, &ignore2,
      &act_msg_.act_fan_speed_cmd[0],   &act_msg_.act_fan_speed_cmd[1],
      &act_msg_.act_fan_speed_cmd[2],   &act_msg_.act_fan_speed_cmd[3],
      &act_msg_.act_fan_speed_cmd[4],   &act_msg_.act_fan_speed_cmd[5],
      &act_msg_.act_fan_pitch_cmd[0],   &act_msg_.act_fan_pitch_cmd[1],
      &act_msg_.act_fan_pitch_cmd[2],   &act_msg_.act_fan_pitch_cmd[3],
      &act_msg_.act_fan_pitch_cmd[4],   &act_msg_.act_fan_pitch_cmd[5],
      &act_msg_.act_body_force_cmd[0],  &act_msg_.act_body_force_cmd[1],
      &act_msg_.act_body_force_cmd[2],  &act_msg_.act_body_torque_cmd[0],
      &act_msg_.act_body_torque_cmd[1], &act_msg_.act_body_torque_cmd[2]);
  if (ret != 20) {
    return 1;
  }
  */

  int ret = fscanf(f_.reg, "%hhu,%hhu\n", &reg_pulse_.cvs_landmark_pulse, &reg_pulse_.cvs_optical_flow_pulse);
  if (ret != 2) {
    fprintf(stderr, "Failed to read registration line.\n");
    return 1;
  }

  // parse landmarks
  ret = fscanf(f_.landmark, "%g,%g,", &ignore1, &ignore2);
  if (ret != 2) {
    fprintf(stderr, "Failed to read landmark line.\n");
    return 1;
  }
  // this is how we tell if we got a landmark update
  if (last_landmark_sec_ != ignore1 || last_landmark_nsec_ != ignore2) {
    last_landmark_sec_ = ignore1;
    last_landmark_nsec_ = ignore2;
    landmark_msg_.cvs_timestamp_sec = seconds_;
    landmark_msg_.cvs_timestamp_nsec = nsec_;
  }
  for (int i = 0; i < 50; i++) {
    ret = fscanf(f_.landmark, "%g,%g,%g,", &landmark_msg_.cvs_landmarks[i],
                  &landmark_msg_.cvs_landmarks[50 + i], &landmark_msg_.cvs_landmarks[100 + i]);
    if (ret != 3) {
      fprintf(stderr, "Failed to read landmark line.\n");
      return 1;
    }
  }
  for (int i = 0; i < 50; i++) {
    ret = fscanf(f_.landmark, "%g,%g,",
                  &landmark_msg_.cvs_observations[i], &landmark_msg_.cvs_observations[50 + i]);
    if (ret != 2) {
      fprintf(stderr, "Failed to read landmark line.\n");
      return 1;
    }
  }
  for (int i = 0; i < 49; i++) {
    ret = fscanf(f_.landmark, "%hhu,", &landmark_msg_.cvs_valid_flag[i]);
    if (ret != 1) {
      fprintf(stderr, "Failed to read landmark line.\n");
      return 1;
    }
  }
  ret = fscanf(f_.landmark, "%hhu\n", &landmark_msg_.cvs_valid_flag[49]);
  if (ret != 1) {
    fprintf(stderr, "Failed to read landmark line.\n");
    return 1;
  }

  // parse optical flow
  ret = fscanf(f_.optical, "%g,%g,", &ignore1, &ignore2);
  if (ret != 2) {
    fprintf(stderr, "Failed to read optical line.\n");
    return 1;
  }
  // this is how we tell if we got a landmark update
  if (last_of_sec_ != ignore1 || last_of_nsec_ != ignore2) {
    last_of_sec_ = ignore1;
    last_of_nsec_ = ignore2;
    optical_msg_.cvs_timestamp_sec = seconds_;
    optical_msg_.cvs_timestamp_nsec = nsec_;
  }
  for (int i = 0; i < 30; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 5; k++) {
        ret = fscanf(f_.optical, "%g,", &optical_msg_.cvs_observations[30 * 2 * k + 30 * j + i]);
        if (ret != 1) {
          fprintf(stderr, "Failed to read optical line.\n");
          return 1;
        }
      }
    }
  }
  for (int i = 0; i < 30; i++) {
    for (int j = 0; j < 5; j++) {
      ret = fscanf(f_.optical, "%hhu,", &optical_msg_.cvs_valid_flag[30 * j + i]);
      if (ret != 1) {
        fprintf(stderr, "Failed to read optical line.\n");
        return 1;
      }
    }
  }
  for (int i = 0; i < 29; i++) {
    ret = fscanf(f_.optical, "%g,", &optical_msg_.cvs_id_tag[i]);
    if (ret != 1) {
      fprintf(stderr, "Failed to read optical line.\n");
      return 1;
    }
  }
  ret = fscanf(f_.optical, "%g\n", &optical_msg_.cvs_id_tag[29]);
  if (ret != 1) {
    fprintf(stderr, "Failed to read optical line.\n");
    return 1;
  }

  // cmc / cmd isn't used by anything so let's not bother
  //  ret = fscanf(f_.cmd, "%d,%d,%d,%d,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,"
  //                      "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
  //    &s->cmd.cmd_timestamp_sec, &s->cmd.cmd_timestamp_nsec,
  //    &s->cmd.cmd_mode, &s->cmd.cmd_B_inuse,
  //    &s->cmd.traj_pos[0], &s->cmd.traj_pos[1], &s->cmd.traj_pos[1],
  //    &s->cmd.traj_vel[0], &s->cmd.traj_vel[1], &s->cmd.traj_vel[2],
  //    &s->cmd.traj_accel[0], &s->cmd.traj_accel[1], &s->cmd.traj_accel[2],
  //    &s->cmd.cmd_pos[0], &s->cmd.cmd_pos[1], &s->cmd.cmd_pos[1],
  //    &s->cmd.cmd_vel[0], &s->cmd.cmd_vel[1], &s->cmd.cmd_vel[2],
  //    &s->cmd.cmd_accel[0], &s->cmd.cmd_accel[1], &s->cmd.cmd_accel[2],
  //    &s->cmd.traj_quat, &s->cmd.traj_omega[0], &s->cmd.traj_omega[1], &s->cmd.traj_omega[2],
  //    &s->cmd.traj_alpha[0], &s->cmd.traj_alpha[1], &s->cmd.traj_alpha[2],
  //    &s->cmd.cmd_quat, &s->cmd.cmd_omega[0], &s->cmd.cmd_omega[1], &s->cmd.cmd_omega[2],
  //    &s->cmd.cmd_alpha[0], &s->cmd.cmd_alpha[1], &s->cmd.cmd_alpha[2]);
  //  if (ret != 36)
  //  return 1;
  imu_msg_.imu_timestamp_sec = seconds_;
  imu_msg_.imu_timestamp_nsec = nsec_;
  ret = fscanf(f_.imu, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%hhu,%hhu\n",
      &ignore1, &ignore2,
      &imu_msg_.imu_A_B_ECI_sensor[0], &imu_msg_.imu_A_B_ECI_sensor[1], &imu_msg_.imu_A_B_ECI_sensor[2],
      &imu_msg_.imu_accel_bias[0], &imu_msg_.imu_accel_bias[1], &imu_msg_.imu_accel_bias[2],
      &imu_msg_.imu_omega_B_ECI_sensor[0], &imu_msg_.imu_omega_B_ECI_sensor[1], &imu_msg_.imu_omega_B_ECI_sensor[2],
      &imu_msg_.imu_gyro_bias[0], &imu_msg_.imu_gyro_bias[1], &imu_msg_.imu_gyro_bias[2],
      &imu_msg_.imu_validity_flag, &imu_msg_.imu_sat_flag);
  if (ret != 16) {
    fprintf(stderr, "Failed to read IMU line.\n");
    return 1;
  }
  ret = fscanf(f_.env, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
      &env_msg_.P_B_ISS_ISS[0], &env_msg_.P_B_ISS_ISS[1], &env_msg_.P_B_ISS_ISS[2],
      &env_msg_.V_B_ISS_ISS[0], &env_msg_.V_B_ISS_ISS[1], &env_msg_.V_B_ISS_ISS[2],
      &env_msg_.A_B_ISS_ISS[0], &env_msg_.A_B_ISS_ISS[1], &env_msg_.A_B_ISS_ISS[2],
      &env_msg_.A_B_ISS_B[0], &env_msg_.A_B_ISS_B[1], &env_msg_.A_B_ISS_B[2],
      &env_msg_.A_B_ECI_B[0], &env_msg_.A_B_ECI_B[1], &env_msg_.A_B_ECI_B[2],
      &env_msg_.Q_ISS2B[0],  &env_msg_.Q_ISS2B[1],  &env_msg_.Q_ISS2B[2],  &env_msg_.Q_ISS2B[3],
      &env_msg_.omega_B_ISS_B[0], &env_msg_.omega_B_ISS_B[1], &env_msg_.omega_B_ISS_B[2],
      &env_msg_.alpha_B_ISS_B[0], &env_msg_.alpha_B_ISS_B[1], &env_msg_.alpha_B_ISS_B[2],
      &env_msg_.fan_torques_B[0], &env_msg_.fan_torques_B[1], &env_msg_.fan_torques_B[2],
      &env_msg_.fan_forces_B[0], &env_msg_.fan_forces_B[1], &env_msg_.fan_forces_B[2]);
  if (ret != 31) {
    fprintf(stderr, "Failed to read env line.\n");
    return 1;
  }
  return 0;
}


void GncSimCSV::Step(void) {
  if (ReadStepState())
    exit(0);
  int rate = 16000000;

  // TODO(bcoltin): load time from gnc autocode
  nsec_ += rate;
  if (nsec_ >= rate) {
    nsec_ -= rate;
    seconds_++;
  }
}

}  // namespace gnc_autocode

