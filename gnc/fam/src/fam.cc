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

#include <fam/fam.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_util/ff_names.h>
#include <ff_hw_msgs/PmcCommand.h>

#include <Eigen/QR>

// parameters fam_force_allocation_module_P are set in
//  matlab/code_generation/fam_force_allocation_module_ert_rtw/fam_force_allocation_module_data.c

namespace fam {

Fam::Fam(ros::NodeHandle* nh) : inertia_received_(false) {
  config_.AddFile("gnc.config");
  config_.AddFile("geometry.config");
  ReadParams();
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&Fam::ReadParams, this));}, false, true);

  const float units_inches_to_meters = 0.0254;
  abp_PM1_P_nozzle_B_B << 6.00,  4.01, -1.56,
                         -6.00,  4.01,  1.56,
                          2.83,  6.00,  2.83,
                         -2.83,  6.00, -2.83,
                         -2.66,  4.01,  6.00,
                          2.66,  4.01, -6.00;
  abp_PM1_P_nozzle_B_B *= units_inches_to_meters;  // [m] PM1: Position vector of the nozzle locations in the body frame

  abp_PM2_P_nozzle_B_B << -6.00, -4.01, -1.56,
                           6.00, -4.01,  1.56,
                          -2.83, -6.00,  2.83,
                           2.83, -6.00, -2.83,
                           2.66, -4.01,  6.00,
                          -2.66, -4.01, -6.00;
  abp_PM2_P_nozzle_B_B *= units_inches_to_meters;  // [m] PM2: Position vector of the nozzle locations in the body frame

  abp_PM1_nozzle_orientations << 1, 0,  0,  //  [unit vec] PM1: Pointing Direction of each nozzle
                                -1, 0,  0,
                                 0, 1,  0,
                                 0, 1,  0,
                                 0, 0,  1,
                                 0, 0, -1;
  abp_PM2_nozzle_orientations <<-1, 0,  0,  //  [unit vec] PM1: Pointing Direction of each nozzle
                                 1, 0,  0,
                                 0, -1, 0,
                                 0, -1, 0,
                                 0, 0,  1,
                                 0, 0, -1;

  pt_fam_.Initialize("fam");

  pmc_pub_ = nh->advertise<ff_hw_msgs::PmcCommand>(TOPIC_HARDWARE_PMC_COMMAND, 1);

  // Subscribe to the flight mode to be notified of speed gain command
  flight_mode_sub_ = nh->subscribe(
    TOPIC_MOBILITY_FLIGHT_MODE, 1, &Fam::FlightModeCallback, this);
  inertia_sub_ = nh->subscribe(
    TOPIC_MOBILITY_INERTIA, 1, &Fam::InertiaCallback, this);

  ctl_sub_ = nh->subscribe(TOPIC_GNC_CTL_COMMAND, 5,
    &Fam::CtlCallBack, this, ros::TransportHints().tcpNoDelay());
}

Fam::~Fam() {}

void Fam::CtlCallBack(const ff_msgs::FamCommand & c) {
  ex_time_msg time;
  cmd_msg cmd;
  ctl_msg ctl;

  time.timestamp_sec  = c.header.stamp.sec;
  time.timestamp_nsec = c.header.stamp.nsec;
  msg_conversions::ros_to_array_vector(c.wrench.force, ctl.body_force_cmd);
  msg_conversions::ros_to_array_vector(c.wrench.torque, ctl.body_torque_cmd);
  msg_conversions::ros_to_array_vector(c.accel, ctl.body_accel_cmd);
  msg_conversions::ros_to_array_vector(c.alpha, ctl.body_alpha_cmd);
  msg_conversions::ros_to_array_vector(c.position_error, ctl.pos_err);
  msg_conversions::ros_to_array_vector(c.position_error_integrated, ctl.pos_err_int);
  msg_conversions::ros_to_array_vector(c.attitude_error, ctl.att_err);
  msg_conversions::ros_to_array_vector(c.attitude_error_integrated, ctl.att_err_int);
  ctl.att_err_mag = c.attitude_error_mag;
  ctl.ctl_status  = c.status;
  cmd.cmd_mode    = c.control_mode;

  input_.body_force_cmd =  msg_conversions::ros_to_eigen_vector(c.wrench.force).cast<float>();
  input_.body_torque_cmd =  msg_conversions::ros_to_eigen_vector(c.wrench.torque).cast<float>();

  Step(&time, &cmd, &ctl);
}

void Fam::FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode) {
  std::lock_guard<std::mutex> lock(mutex_speed_);
  speed_ = mode->speed;
  input_.speed_gain_cmd = mode->speed;
}

void Fam::InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia) {
  std::lock_guard<std::mutex> lock(mutex_mass_);
  center_of_mass_ = msg_conversions::ros_to_eigen_vector(inertia->inertia.com).cast<float>();
  UpdateCOM(center_of_mass_);
  inertia_received_ = true;
}

void Fam::UpdateCOM(const Eigen::Vector3f & com) {
  // latch nozzle thrust matrices
  Eigen::Matrix<float, 12, 3> fam_P_nozzle_B_B, fam_nozzle_orientations;
  fam_P_nozzle_B_B << abp_PM1_P_nozzle_B_B, abp_PM2_P_nozzle_B_B;
  fam_nozzle_orientations << abp_PM1_nozzle_orientations, abp_PM2_nozzle_orientations;

  Eigen::Matrix<float, 12, 3> nozzle_moment_arm = fam_P_nozzle_B_B - com.transpose().replicate(12, 1);
  for (int i = 0; i < 12; i++)
    thrust2torque_.col(i) = -nozzle_moment_arm.row(i).cross(fam_nozzle_orientations.row(i)).transpose();

  thrust2force_ = -fam_nozzle_orientations.transpose();
  Eigen::Matrix<float, 6, 12> thrust2forcetorque;
  thrust2forcetorque << thrust2force_, thrust2torque_;
  forcetorque2thrust_ = Eigen::MatrixXf::Identity(12, 6);
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(thrust2forcetorque, Eigen::ComputeThinU | Eigen::ComputeThinV);
  forcetorque2thrust_ = svd.solve(forcetorque2thrust_);
}

void Fam::CalcThrustMatrices(const Eigen::Vector3f & force, const Eigen::Vector3f & torque,
    Eigen::Matrix<float, 6, 1>& pmc1_nozzle_thrusts, Eigen::Matrix<float, 6, 1>& pmc2_nozzle_thrusts) {
  Eigen::Matrix<float, 6, 1> forcetorque;
  forcetorque << force, torque;
  Eigen::Matrix<float, 12, 1> thrust_per_nozzle = forcetorque2thrust_ * forcetorque;
  const int fam_nozzle_dirs[][4] = {{1, 2,  7,  8}, {3, 4,  9, 10}, {5, 6, 11, 12}};
  // prevents negative thrust from being commanded by adding the min value below zero to the null vectors
  // negative thrust occurs because the inverse matrix distributes thrust accross the
  // nozzles aligned in an axis equally, even if they are pointed in opposite directions
  for (int i = 0; i < 3; i++) {
    float min_thrust = 0.0;
    for (int j = 0; j < 4; j++) {
      float t = thrust_per_nozzle(fam_nozzle_dirs[i][j] - 1);
      if (t < min_thrust)
        min_thrust = t;
    }
    if (min_thrust < 0) {
      for (int j = 0; j < 4; j++)
        thrust_per_nozzle(fam_nozzle_dirs[i][j] - 1) -= min_thrust;
    }
  }
  pmc1_nozzle_thrusts = thrust_per_nozzle.head<6>();
  pmc2_nozzle_thrusts = thrust_per_nozzle.tail<6>();
}

float Fam::Lookup(int table_size, const float lookup[], const float breakpoints[], float value) {
  if (value < breakpoints[0])
    return lookup[0];
  if (value > breakpoints[table_size - 1])
    return lookup[table_size - 1];
  int start = 0, end = table_size;
  while (start < end - 1) {
    int mid = (start + end) / 2;
    if (breakpoints[mid] <= value)
      start = mid;
    else
      end = mid;
  }

  float ratio = (value - breakpoints[start]) / (breakpoints[start + 1] - breakpoints[start]);
  return (1.0 - ratio) * lookup[start] + ratio * lookup[start + 1];
}

float Fam::ComputePlenumDeltaPressure(float impeller_speed, Eigen::Matrix<float, 6, 1> & discharge_coeff,
                                     const Eigen::Matrix<float, 6, 1> & nozzle_thrusts) {
  // lookup_data_tables
  float total_thrust = (nozzle_thrusts.array() / discharge_coeff.array()).sum();
  float cdp = Lookup(THRUST_LOOKUP_SIZE, THRUST_LOOKUP_CDP, THRUST_LOOKUP_BREAKPOINTS,
                     total_thrust / (impeller_speed * impeller_speed));
  const float units_inches_to_meters = 0.0254;
  const float impeller_diameter = 5.5 * units_inches_to_meters;
  const float air_density = 1.2;  //  [kg/m^3]   Air density inside of the ISS (some places in code this was 1.225?)
  return impeller_speed * impeller_speed * impeller_diameter * impeller_diameter * air_density * cdp;
}

void Fam::CalcPMServoCmd(bool is_pmc1, uint8_t speed_gain_cmd, const Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
    uint8_t & impeller_speed_cmd, Eigen::Matrix<float, 6, 1> & servo_pwm_cmd,
    Eigen::Matrix<float, 6, 1> & nozzle_theta_cmd,
    Eigen::Matrix<float, 6, 1> & normalized_pressure_density, Eigen::Matrix<float, 6, 1> & command_area_per_nozzle) {
  if (speed_gain_cmd > 3)
    ROS_FATAL("Invalid speed command %d.", speed_gain_cmd);

  Eigen::Matrix<float, 6, 1> discharge_coeff, nozzle_widths;
  // Nozzle Discharge coefficient.  Determined via Test
  if (is_pmc1)
    discharge_coeff << 0.914971062, 0.755778254, 0.940762925, 0.792109779, 0.92401881 , 0.930319765;
  else
    discharge_coeff << 0.947114008, 0.764468916, 1.000000000, 0.90480943 , 0.936555627, 0.893794766;
  nozzle_widths << 5.0, 5.0, 2.8, 2.8, 2.8, 2.8;
  const float units_inches_to_meters = 0.0254;
  nozzle_widths *= units_inches_to_meters;

  float impeller_speed = 0.0f, plenum_delta_pressure = 0.0f;
  impeller_speed_cmd = 0;
  if (speed_gain_cmd != 0) {
    impeller_speed = IMPELLER_SPEEDS[speed_gain_cmd - 1];
    const float impeller_speed2pwm = 0.792095;  //  [CNT/(rad/sec)] Converts impeller speeds into PWM values
    impeller_speed_cmd = static_cast<uint8_t>(impeller_speed2pwm * impeller_speed + 0.5);
    plenum_delta_pressure = ComputePlenumDeltaPressure(impeller_speed, discharge_coeff, nozzle_thrusts);
  }
  normalized_pressure_density = 2 * (discharge_coeff.array() * discharge_coeff.array()).matrix() *
                                      plenum_delta_pressure;
  command_area_per_nozzle = (nozzle_thrusts.array() / normalized_pressure_density.array()).matrix();
  const int nozzle_flap_count = 2;
  const float nozzle_intake_height = 0.5154 * units_inches_to_meters;
  const float nozzle_flap_length = 0.5353 * units_inches_to_meters;
  Eigen::Matrix<float, 6, 1> openings = (nozzle_intake_height - (1.0 / nozzle_flap_count) *
                       command_area_per_nozzle.array() / nozzle_widths.array()).matrix() / nozzle_flap_length;
  const float nozzle_min_angle = 15.68 * M_PI / 180;
  const float nozzle_max_angle = 79.91 * M_PI / 180;  //  THEORETICAL max angle that a nozzle can open to
  const float min_open_cos = cos(nozzle_max_angle);
  const float max_open_cos = cos(nozzle_min_angle);
  for (int i = 0; i < 6; i++) {
    if (openings(i) < min_open_cos || std::isnan(openings[i]))
      openings[i] = min_open_cos;
    if (openings(i) > max_open_cos)
      openings[i] = max_open_cos;
    nozzle_theta_cmd[i] = acos(openings[i]) - nozzle_min_angle;
  }
  const int servo_max_pwm = 255;
  const int servo_min_pwm = 0;
  //    %[-] Conversion between servo angle (not nozzle angle) command and resulting servo PWM
  servo_pwm_cmd = static_cast<float>(servo_max_pwm - servo_min_pwm) /
                  (nozzle_max_angle - nozzle_min_angle) * nozzle_theta_cmd;
}

void Fam::RunFam(const FamInput& in, uint8_t* speed_cmd, Eigen::Matrix<float, 12, 1> & servo_pwm_cmd) {
  Eigen::Matrix<float, 3, 12> thrust2force_B, thrust2torque_B;
  Eigen::Matrix<float, 6, 1> pmc1_nozzle_thrusts, pmc2_nozzle_thrusts;
  CalcThrustMatrices(
                     in.body_force_cmd, in.body_torque_cmd,
                     pmc1_nozzle_thrusts, pmc2_nozzle_thrusts);
  Eigen::Matrix<float, 6, 1> nozzle_theta, normalized_pressure_density, command_area_per_nozzle;
  Eigen::Matrix<float, 6, 1> servo_pwm;
  CalcPMServoCmd(true, in.speed_gain_cmd, pmc1_nozzle_thrusts, speed_cmd[0], servo_pwm, nozzle_theta,
      normalized_pressure_density, command_area_per_nozzle);
  servo_pwm_cmd.head<6>() = servo_pwm;
  CalcPMServoCmd(false, in.speed_gain_cmd, pmc2_nozzle_thrusts, speed_cmd[1], servo_pwm, nozzle_theta,
      normalized_pressure_density, command_area_per_nozzle);
  servo_pwm_cmd.tail<6>() = servo_pwm;
  // computes actual output force and torque, but currently not published
  // Eigen::Matrix<float, 12, 1> temp = (command_area_per_nozzle.array() * normalized_pressure_density.array());
  // Eigen::Vector3f predicted_force = thrust2force_B * temp;
  // Eigen::Vector3f predicted_torque = thrust2torque_B * temp;
}

void Fam::TestTwoVectors(const char* name, const Eigen::Matrix<float, 6, 1> new_array,
                                   const float old_array[], float tolerance) {
  for (int i = 0; i < 6; i++) {
    float difference = old_array[i] - new_array[i];
    float perc_difference = difference / old_array[i];
    if (fabs(difference) > tolerance) {
      std::string p1, p2;
      for (int i = 0; i < 6; i++) {
        p1 += " " + std::to_string(new_array[i]);
        p2 += " " + std::to_string(old_array[i]);
      }
      ROS_ERROR("%s New: %s, Old: %s", name, p1.c_str(), p2.c_str());
    }
  }
}

void Fam::Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl) {
  {
    std::lock_guard<std::mutex> lock(mutex_speed_);
    // Overwrite the speed command with the cached value, provided
    // through the flight mode message offered by the choreographer
    cmd->speed_gain_cmd = speed_;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_mass_);
    if (!inertia_received_) {
      ROS_DEBUG_STREAM_THROTTLE(10, "FAM step waiting for inertia.");
      return;
    }
    gnc_.cmc_.center_of_mass[0] = center_of_mass_[0];
    gnc_.cmc_.center_of_mass[1] = center_of_mass_[1];
    gnc_.cmc_.center_of_mass[2] = center_of_mass_[2];
  }

  // Step the FAM simulink code
  pt_fam_.Tick();
  uint8_t speed_cmd[2];
  Eigen::Matrix<float, 12, 1> servo_pwm_cmd;
  if (compare_fam_ || !use_old_fam_)
    RunFam(input_, speed_cmd, servo_pwm_cmd);
  if (compare_fam_ || use_old_fam_)
    gnc_.Step(ex_time, cmd, ctl);
  pt_fam_.Tock();

  if (compare_fam_) {
    if (speed_cmd[0] != gnc_.act_.act_impeller_speed_cmd[0] || speed_cmd[1] != gnc_.act_.act_impeller_speed_cmd[1])
      ROS_ERROR("Speed incorrect. New: %d %d Old: %d %d", speed_cmd[0], speed_cmd[1],
                 gnc_.act_.act_impeller_speed_cmd[0], gnc_.act_.act_impeller_speed_cmd[1]);
    TestTwoVectors("PMC1", servo_pwm_cmd.head<6>(), gnc_.act_.act_servo_pwm_cmd, 0.1);
    TestTwoVectors("PMC2", servo_pwm_cmd.tail<6>(), gnc_.act_.act_servo_pwm_cmd + 6, 0.1);
  }

  // Send the PMC command
  static ff_hw_msgs::PmcCommand pmc;
  pmc.header.stamp = ros::Time::now();
  pmc.header.frame_id = "body";
  pmc.goals.resize(2);
  if (use_old_fam_) {
    pmc.goals[0].motor_speed = gnc_.act_.act_impeller_speed_cmd[0];
    pmc.goals[1].motor_speed = gnc_.act_.act_impeller_speed_cmd[1];
    std::copy(gnc_.act_.act_servo_pwm_cmd, gnc_.act_.act_servo_pwm_cmd + 6,
        pmc.goals[0].nozzle_positions.c_array());
    std::copy(gnc_.act_.act_servo_pwm_cmd + 6, gnc_.act_.act_servo_pwm_cmd + 12,
        pmc.goals[1].nozzle_positions.c_array());
  } else {
    pmc.goals[0].motor_speed = speed_cmd[0];
    pmc.goals[1].motor_speed = speed_cmd[1];
    for (int i = 0; i < 6; i++) {
      pmc.goals[0].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[i];
      pmc.goals[1].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[6 + i];
    }
  }
  pmc_pub_.publish<ff_hw_msgs::PmcCommand>(pmc);

  pt_fam_.Send();
}

void Fam::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  if (!config_.GetBool("fam_use_old", &use_old_fam_))
    ROS_FATAL("fam_use_old not specified.");
  if (!config_.GetBool("fam_compare_old", &compare_fam_))
    ROS_FATAL("fam_compare_old not specified.");
  gnc_.ReadParams(&config_);
}

}  // end namespace fam
