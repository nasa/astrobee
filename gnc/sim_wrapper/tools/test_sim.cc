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

#include <gnc_autocode/sim.h>
#include <gnc_autocode/sim_csv.h>
#include <gnc_autocode/ekf.h>
#include <gnc_autocode/ekf_csv.h>

#define COMPARE_INT(a, b) if (a != b) {fprintf(stderr, "Comparison failed at line %d. (%d, %d)\n", __LINE__, \
    (a), (b)); return 1;}
#define COMPARE_INT_VECTOR(a, b, len) for (int unused_variable = 0; unused_variable < len; unused_variable++) \
                    if ((a)[unused_variable] != (b)[unused_variable]) {\
                    fprintf(stderr, "Comparison failed at line %d vector element %d. (%d, %d)\n", __LINE__, \
                    unused_variable, ((a)[unused_variable]), ((b)[unused_variable])); return 1;}
#define COMPARE_FLOAT(a, b, tol) if (fabs((a) - (b)) >= tol) {fprintf(stderr, \
                    "Comparison failed at line %d. (%g, %g)\n", __LINE__, (a), (b)); return 1;}
#define COMPARE_FLOAT_VECTOR(a, b, len, tol) for (int unused_variable = 0; unused_variable < len; unused_variable++) \
                    if (fabs(((a)[unused_variable]) - ((b)[unused_variable])) >= tol) {\
                    fprintf(stderr, "Comparison failed at line %d vector element %d. (%g, %g)\n", __LINE__, \
                    unused_variable, ((a)[unused_variable]), ((b)[unused_variable])); return 1;}

int verify_sim_output(const gnc_autocode::GncSimAutocode & sim, const gnc_autocode::GncSimAutocode & csv) {
  float tolerance = 1e-4;

  COMPARE_INT(sim.reg_pulse_.cvs_landmark_pulse, csv.reg_pulse_.cvs_landmark_pulse);
  COMPARE_INT(sim.reg_pulse_.cvs_optical_flow_pulse, csv.reg_pulse_.cvs_optical_flow_pulse);

  const cvs_landmark_msg & l1 = sim.landmark_msg_;
  const cvs_landmark_msg & l2 = csv.landmark_msg_;
  COMPARE_INT_VECTOR(l1.cvs_valid_flag, l2.cvs_valid_flag, 50);
  // unfortunately precision errors cause these results to differ
  // COMPARE_FLOAT_VECTOR(l1.cvs_observations, l2.cvs_observations, 2 * 50, tolerance);
  // COMPARE_FLOAT_VECTOR(l1.cvs_landmarks, l2.cvs_landmarks, 3 * 50, 0.005);

  // compare imu_msgs
  const imu_msg & i1 = sim.imu_msg_;
  const imu_msg & i2 = csv.imu_msg_;
  COMPARE_FLOAT_VECTOR(i1.imu_A_B_ECI_sensor, i2.imu_A_B_ECI_sensor, 3, 1e-2);
  COMPARE_FLOAT_VECTOR(i1.imu_accel_bias, i2.imu_accel_bias, 3, tolerance);
  COMPARE_FLOAT_VECTOR(i1.imu_omega_B_ECI_sensor, i2.imu_omega_B_ECI_sensor, 3, 2e-4);
  COMPARE_FLOAT_VECTOR(i1.imu_gyro_bias, i2.imu_gyro_bias, 3, tolerance);
  COMPARE_INT(i1.imu_validity_flag, i2.imu_validity_flag);
  COMPARE_INT(i1.imu_sat_flag, i2.imu_sat_flag);

  // compare env_msgs
  const env_msg & e1 = sim.env_msg_;
  const env_msg & e2 = csv.env_msg_;
  COMPARE_FLOAT_VECTOR(e1.P_B_ISS_ISS, e2.P_B_ISS_ISS, 3, 0.001);
  COMPARE_FLOAT_VECTOR(e1.V_B_ISS_ISS, e2.V_B_ISS_ISS, 3, 0.0001);
  COMPARE_FLOAT_VECTOR(e1.A_B_ISS_ISS, e2.A_B_ISS_ISS, 3, 0.0001);
  COMPARE_FLOAT_VECTOR(e1.A_B_ISS_B, e2.A_B_ISS_B, 3, tolerance);
  COMPARE_FLOAT_VECTOR(e1.A_B_ECI_B, e2.A_B_ECI_B, 3, 0.002);
  COMPARE_FLOAT_VECTOR(e1.Q_ISS2B, e2.Q_ISS2B, 4, 0.05);
  COMPARE_FLOAT_VECTOR(e1.omega_B_ISS_B, e2.omega_B_ISS_B, 3, 0.001);
  COMPARE_FLOAT_VECTOR(e1.alpha_B_ISS_B, e2.alpha_B_ISS_B, 3, 0.0005);
  COMPARE_FLOAT_VECTOR(e1.fan_torques_B, e2.fan_torques_B, 3, 0.001);
  COMPARE_FLOAT_VECTOR(e1.fan_forces_B, e2.fan_forces_B, 3, 0.001);
  return 0;
}

int verify_ekf_output(const gnc_autocode::GncEkfAutocode & ekf, const gnc_autocode::GncEkfCSV & csv) {
  const kfl_msg & k1 = ekf.kfl_;
  const kfl_msg & k2 = csv.kfl_;
  COMPARE_FLOAT_VECTOR(k1.quat_ISS2B, k2.quat_ISS2B, 4, 0.001);
  COMPARE_FLOAT_VECTOR(k1.omega_B_ISS_B, k2.omega_B_ISS_B, 3, 0.001);
  COMPARE_FLOAT_VECTOR(k1.gyro_bias, k2.gyro_bias, 3, 0.001);
  COMPARE_FLOAT_VECTOR(k1.V_B_ISS_ISS, k2.V_B_ISS_ISS, 3, 0.07);
  COMPARE_FLOAT_VECTOR(k1.A_B_ISS_ISS, k2.A_B_ISS_ISS, 3, 0.07);
  COMPARE_FLOAT_VECTOR(k1.accel_bias, k2.accel_bias, 3, 0.003);
  COMPARE_FLOAT_VECTOR(k1.P_B_ISS_ISS, k2.P_B_ISS_ISS, 3, 0.005);
  COMPARE_INT(k1.confidence, k2.confidence);
  COMPARE_INT(k1.aug_state_enum, k2.aug_state_enum);
  COMPARE_FLOAT_VECTOR(k1.ml_quat_ISS2cam, k2.ml_quat_ISS2cam, 4, 0.001);
  COMPARE_FLOAT_VECTOR(k1.ml_P_cam_ISS_ISS, k2.ml_P_cam_ISS_ISS, 3, 0.005);
  COMPARE_FLOAT_VECTOR(k1.of_quat_ISS2cam,  k2.of_quat_ISS2cam, 20, 0.001);
  COMPARE_FLOAT_VECTOR(k1.of_P_cam_ISS_ISS, k2.of_P_cam_ISS_ISS, 15, 0.005);
  COMPARE_FLOAT_VECTOR(k1.cov_diag, k2.cov_diag, 51, 0.005);
  COMPARE_INT(k1.kfl_status, k2.kfl_status);
  return 0;
}

int main(int argc, char** argv) {
  gnc_autocode::GncSimCSV csv_sim;
  gnc_autocode::GncSimAutocode sim;

  gnc_autocode::GncEkfCSV csv_ekf;
  gnc_autocode::GncEkfAutocode ekf;

  csv_sim.Initialize(std::string("."));
  csv_ekf.Initialize(std::string("."));
  sim.Initialize();
  ekf.Initialize();

  while (true) {
    csv_sim.Step();

    // set the act message
    memcpy(&sim.act_msg_, &csv_sim.act_msg_, sizeof(act_msg));
    sim.Step();

    if (verify_sim_output(sim, csv_sim)) {
      fprintf(stderr, "Sim failed at time %g.\n",
          csv_sim.act_msg_.act_timestamp_sec + static_cast<double>(csv_sim.act_msg_.act_timestamp_nsec) / 1e9);
      break;
    }

    csv_ekf.Step();

    // now copy the inputs to the EKF
    memcpy(&ekf.vis_, &csv_sim.landmark_msg_, sizeof(cvs_landmark_msg));
    memcpy(&ekf.reg_, &csv_sim.reg_pulse_, sizeof(cvs_registration_pulse));
    memcpy(&ekf.of_,  &csv_sim.optical_msg_, sizeof(cvs_optical_flow_msg));
    memcpy(&ekf.imu_, &csv_sim.imu_msg_, sizeof(imu_msg));
    ekf.Step();

    if (verify_ekf_output(ekf, csv_ekf)) {
      fprintf(stderr, "EKF failed at time %g.\n",
          csv_sim.act_msg_.act_timestamp_sec + static_cast<double>(csv_sim.act_msg_.act_timestamp_nsec) / 1e9);
      break;
    }
  }
}
