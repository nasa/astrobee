//
// File: sim_model_lib0.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:22:10 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "sim_model_lib0.h"
#include "sim_model_lib0_private.h"

const env_msg sim_model_lib0_rtZenv_msg = {
  {
    0.0F, 0.0F, 0.0F }
  ,                                    // P_B_ISS_ISS

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // V_B_ISS_ISS

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // A_B_ISS_ISS

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // A_B_ISS_B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // A_B_ECI_B

  {
    0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // Q_ISS2B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // omega_B_ISS_B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // omega_B_ECI_B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // alpha_B_ISS_B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // fan_torques_B

  {
    0.0F, 0.0F, 0.0F }
  // fan_forces_B
} ;                                    // env_msg ground

const ex_time_msg sim_model_lib0_rtZex_time_msg = {
  0U,                                  // timestamp_sec
  0U                                   // timestamp_nsec
} ;                                    // ex_time_msg ground

const cvs_landmark_msg sim_model_lib0_rtZcvs_landmark_msg = {
  0U,                                  // cvs_timestamp_sec
  0U,                                  // cvs_timestamp_nsec

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // cvs_landmarks

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // cvs_observations

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  // cvs_valid_flag
} ;                                    // cvs_landmark_msg ground

const cvs_optical_flow_msg sim_model_lib0_rtZcvs_optical_flow_msg = {
  0U,                                  // cvs_timestamp_sec
  0U,                                  // cvs_timestamp_nsec

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F }
  ,                                    // cvs_observations

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U }
  ,                                    // cvs_valid_flag

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  // cvs_id_tag
} ;                                    // cvs_optical_flow_msg ground

const cvs_handrail_msg sim_model_lib0_rtZcvs_handrail_msg = {
  0U,                                  // cvs_timestamp_sec
  0U,                                  // cvs_timestamp_nsec

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // cvs_landmarks

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // cvs_observations

  {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
  ,                                    // cvs_valid_flag
  0U,                                  // cvs_3d_knowledge_flag

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // cvs_handrail_local_pos

  {
    0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // cvs_handrail_local_quat
  0U                                   // cvs_handrail_update_global_pose_flag
} ;                                    // cvs_handrail_msg ground

const cvs_registration_pulse sim_model_lib0_rtZcvs_registration_pulse = {
  0U,                                  // cvs_ar_tag_pulse
  0U,                                  // cvs_landmark_pulse
  0U,                                  // cvs_optical_flow_pulse
  0U                                   // cvs_handrail_pulse
} ;                                    // cvs_registration_pulse ground

const bpm_msg sim_model_lib0_rtZbpm_msg = {
  0U,                                  // bpm_timestamp_sec
  0U,                                  // bpm_timestamp_nsec

  {
    0.0F, 0.0F }
  ,                                    // bpm_motor_curr

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // bpm_servo_curr

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // bpm_torque_B

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // bpm_force_B

  {
    0.0F, 0.0F }
  ,                                    // bpm_motor_speed

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  // bpm_nozzle_theta
} ;                                    // bpm_msg ground

const imu_msg sim_model_lib0_rtZimu_msg = {
  0U,                                  // imu_timestamp_sec
  0U,                                  // imu_timestamp_nsec

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // imu_A_B_ECI_sensor

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // imu_accel_bias

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // imu_omega_B_ECI_sensor

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // imu_gyro_bias
  0U,                                  // imu_validity_flag
  0U                                   // imu_sat_flag
} ;                                    // imu_msg ground

// Forward declaration for local functions
static void sim_model__format_AR_tag_output(const real32_T points_in_iss[108],
  const real32_T points_in_cam[72], boolean_T valid_in[36], real32_T
  points_out_iss[150], real32_T points_out_cam[100], real32_T valid_out[50]);
static void sim_model_lib0_rand(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW);
static void sim_model_lib0_randperm(real_T n, real_T p_data[], int32_T p_sizes[2],
  DW_sim_model_lib0_T *sim_model_lib0_DW);
static void sim_mode_format_handrail_output(const real32_T points_in_iss[1512],
  const real32_T points_in_cam[1512], const boolean_T valid_in[504], real32_T
  points_out_iss[150], real32_T points_out_cam[150], real32_T valid_out[50],
  B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T *sim_model_lib0_DW);
static void sim__pphdimglohlnoppp_mergesort(int32_T idx_data[], const real32_T
  x_data[], const int32_T x_sizes[2], const int32_T dir[2], int32_T n,
  B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_mo_aaieppphglfcglng_sortIdx(const real32_T x_data[], const
  int32_T x_sizes[2], const int32_T col[2], int32_T idx_data[], int32_T
  *idx_sizes, B_sim_model_lib0_T *sim_model_lib0_B);
static void hdjmcjmophlfkngl_apply_row_perm(real32_T y_data[], int32_T y_sizes[2],
  const int32_T idx_data[], B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_m_mglflfcbkfkfbiec_sortrows(real32_T y_data[], int32_T y_sizes[2],
  real_T ndx_data[], int32_T *ndx_sizes, B_sim_model_lib0_T *sim_model_lib0_B);
static void sim__gdbacbaajekfglng_mergesort(int32_T idx_data[], const int32_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_mo_dbaaphlnfkfkphdb_sortIdx(const int32_T x_data[], const
  int32_T x_sizes, int32_T idx_data[], int32_T *idx_sizes, B_sim_model_lib0_T
  *sim_model_lib0_B);
static void sim_fkfchlfkgdbimgdj_do_vectors(const real32_T a[50], const real32_T
  b_data[], const int32_T b_sizes, real32_T c_data[], int32_T *c_sizes, int32_T
  ia_data[], int32_T *ia_sizes, int32_T ib_data[], int32_T *ib_sizes,
  B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_model_lib0_rand_n(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW);
static void sim__aiekglfcjekfdbai_mergesort(int32_T idx_data[], const real_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_mo_cbimppphbaaaaiec_sortIdx(const real_T x_data[], const int32_T
  x_sizes[2], int32_T idx_data[], int32_T idx_sizes[2], B_sim_model_lib0_T
  *sim_model_lib0_B);
static void sim_model_lib0_randperm_i(real_T n, real_T p_data[], int32_T
  p_sizes[2], B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T
  *sim_model_lib0_DW);
static void sim_mo_lfcjjmgdkfkniekn_sortIdx(const real32_T x[17264], const
  int32_T col[2], int32_T idx[8632], B_sim_model_lib0_T *sim_model_lib0_B);
static void imglgdjmcjmomgdj_apply_row_perm(real32_T y[17264], const int32_T
  idx[8632], B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_m_nopphdbaaiecfkng_sortrows(real32_T y[17264], real_T ndx[8632],
  B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_model_lib0_rand_o(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW);
static void sim__kfkflfcjohdbglno_mergesort(int32_T idx_data[], const real_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B);
static void sim_mo_bimgaiekaimglfkf_sortIdx(const real_T x_data[], const int32_T
  x_sizes[2], int32_T idx_data[], int32_T idx_sizes[2], B_sim_model_lib0_T
  *sim_model_lib0_B);
static void sim_model_lib0_randperm_e(real_T n, real_T p_data[], int32_T
  p_sizes[2], B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T
  *sim_model_lib0_DW);
static void sim_mode_format_landmark_output(const real32_T points_in_iss[25896],
  const real32_T points_in_cam[17264], const boolean_T valid_in[8632], real32_T
  points_out_iss[150], real32_T points_out_cam[100], real32_T valid_out[50],
  B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T *sim_model_lib0_DW);
const act_msg sim_model_lib0_rtZact_msg = { 0U,// act_timestamp_sec
  0U,                                  // act_timestamp_nsec
  { 0U, 0U },                          // act_impeller_speed_cmd
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// act_servo_pwm_cmd 
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// act_nozzle_theta 
  { 0.0F, 0.0F, 0.0F },                // act_predicted_force_B
  { 0.0F, 0.0F, 0.0F }                 // act_predicted_torque_B
};

const cmc_msg sim_model_lib0_rtZcmc_msg = { { 0U,// timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmc_state_cmd_a
  { 0U,                                // timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmc_state_cmd_b
  0U,                                  // cmc_mode_cmd
  0U,                                  // speed_gain_cmd
  0U,                                  // localization_mode_cmd
  { 0.0F, 0.0F, 0.0F },                // att_kp
  { 0.0F, 0.0F, 0.0F },                // att_ki
  { 0.0F, 0.0F, 0.0F },                // omega_kd
  { 0.0F, 0.0F, 0.0F },                // pos_kp
  { 0.0F, 0.0F, 0.0F },                // pos_ki
  { 0.0F, 0.0F, 0.0F },                // vel_kd
  { 0.0F, 0.0F, 0.0F },                // center_of_mass
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// inertia_matrix
  0.0F                                 // mass
};

//
// Output and update for action system:
//    '<S17>/Normalize'
//    '<S28>/Normalize'
//
void sim_model_lib0_Normalize(const real32_T rtu_Vec[4], real32_T rtu_Magnitude,
  real32_T rty_Normalized_Vec[4])
{
  // Product: '<S20>/Divide'
  rty_Normalized_Vec[0] = rtu_Vec[0] / rtu_Magnitude;
  rty_Normalized_Vec[1] = rtu_Vec[1] / rtu_Magnitude;
  rty_Normalized_Vec[2] = rtu_Vec[2] / rtu_Magnitude;
  rty_Normalized_Vec[3] = rtu_Vec[3] / rtu_Magnitude;
}

//
// Termination for action system:
//    '<S17>/Normalize'
//    '<S28>/Normalize'
//
void sim_model_li_Normalize_Term(void)
{
}

//
// Output and update for action system:
//    '<S42>/Normalize'
//    '<S197>/Normalize'
//    '<S230>/Normalize'
//
void sim_model_lib0_Normalize_e(const real32_T rtu_Vec[3], real32_T
  rtu_Magnitude, real32_T rty_Normalized_Vec[3])
{
  // Product: '<S44>/Divide'
  rty_Normalized_Vec[0] = rtu_Vec[0] / rtu_Magnitude;
  rty_Normalized_Vec[1] = rtu_Vec[1] / rtu_Magnitude;
  rty_Normalized_Vec[2] = rtu_Vec[2] / rtu_Magnitude;
}

//
// Termination for action system:
//    '<S42>/Normalize'
//    '<S197>/Normalize'
//    '<S230>/Normalize'
//
void sim_model__Normalize_m_Term(void)
{
}

//
// System initialize for iterator system:
//    '<S85>/pinhole_projection_model'
//    '<S113>/pinhole_projection_model'
//    '<S137>/pinhole_projection_model'
//    '<S161>/pinhole_projection_model'
//
void pinhole_projection_mod_Init(int32_T NumIters,
  DW_pinhole_projection_model_s_T localDW[], P_pinhole_projection_model_si_T
  *localP, real_T rtp_pixel_noise_var, real_T rtp_pixel_noise_seed)
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  uint32_T tseed;
  int32_T r;
  int32_T t;
  real_T tmp;
  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    // InitializeConditions for RandomNumber: '<S106>/pixel_noise'
    tmp = floor(rtp_pixel_noise_seed + 1.0);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)
      tmp;
    r = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)r << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    localDW[ForEach_itr].CoreSubsys.RandSeed = tseed;
    localDW[ForEach_itr].CoreSubsys.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&localDW[ForEach_itr].CoreSubsys.RandSeed) * sqrt(rtp_pixel_noise_var) +
      localP->CoreSubsys.pixel_noise_Mean;

    // End of InitializeConditions for RandomNumber: '<S106>/pixel_noise'

    // InitializeConditions for RandomNumber: '<S106>/pixel_noise1'
    tmp = floor(rtp_pixel_noise_seed + 2.0);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)
      tmp;
    r = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)r << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    localDW[ForEach_itr].CoreSubsys.RandSeed_k = tseed;
    localDW[ForEach_itr].CoreSubsys.NextOutput_n = rt_nrand_Upu32_Yd_f_pw_snf
      (&localDW[ForEach_itr].CoreSubsys.RandSeed_k) * sqrt(rtp_pixel_noise_var)
      + localP->CoreSubsys.pixel_noise1_Mean;

    // End of InitializeConditions for RandomNumber: '<S106>/pixel_noise1'
  }
}

//
// Start for iterator system:
//    '<S85>/pinhole_projection_model'
//    '<S113>/pinhole_projection_model'
//    '<S137>/pinhole_projection_model'
//    '<S161>/pinhole_projection_model'
//
void pinhole_projection_mo_Start(int32_T NumIters,
  DW_pinhole_projection_model_s_T localDW[])
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    localDW[ForEach_itr].CoreSubsys.NextOutput = 0.0;
    localDW[ForEach_itr].CoreSubsys.NextOutput_n = 0.0;
  }
}

//
// Output and update for iterator system:
//    '<S85>/pinhole_projection_model'
//    '<S113>/pinhole_projection_model'
//    '<S137>/pinhole_projection_model'
//    '<S161>/pinhole_projection_model'
//
void si_pinhole_projection_model(int32_T NumIters, const real32_T
  rtu_P_points_cam_cam[], real32_T rty_P_points_2D_cam[], boolean_T
  rty_points_in_FOV_flag[], DW_pinhole_projection_model_s_T localDW[],
  P_pinhole_projection_model_si_T *localP, uint8_T rtp_noise_on_flag, real32_T
  rtp_cam_focal_length_Y, real32_T rtp_cam_focal_length_X, real_T
  rtp_pixel_noise_var, real32_T rtp_cam_num_X_pixels, real32_T
  rtp_cam_num_Y_pixels, real32_T rtp_cam_min_dist, real32_T rtp_cam_max_dist,
  const real32_T rtp_cam_pointing[3])
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  real32_T rtb_DotProduct1;
  real32_T rtb_Switch_j_idx_0;
  real32_T rtb_Switch_j_idx_1;

  // Outputs for Iterator SubSystem: '<S85>/pinhole_projection_model' incorporates:
  //   ForEach: '<S89>/For Each'

  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    // Switch: '<S106>/Switch' incorporates:
    //   Constant: '<S106>/Constant'
    //   Constant: '<S106>/Constant3'
    //   Constant: '<S106>/Constant4'
    //   DataTypeConversion: '<S106>/Data Type Conversion1'
    //   DataTypeConversion: '<S106>/Data Type Conversion2'
    //   ForEachSliceSelector: '<S89>/ImpSel_InsertedFor_P_points_cam_cam_at_outport_0'
    //   Gain: '<S106>/Gain'
    //   Gain: '<S106>/Gain1'
    //   Product: '<S106>/Product1'
    //   Product: '<S106>/Product2'
    //   RandomNumber: '<S106>/pixel_noise'
    //   RandomNumber: '<S106>/pixel_noise1'
    //   Sum: '<S106>/Sum1'
    //   Sum: '<S106>/Sum3'

    if (rtu_P_points_cam_cam[(int32_T)((int32_T)(NumIters << 1) + ForEach_itr)] >
        localP->CoreSubsys.Switch_Threshold) {
      rtb_Switch_j_idx_0 = rtp_cam_focal_length_X *
        rtu_P_points_cam_cam[ForEach_itr] / rtu_P_points_cam_cam[(int32_T)
        ((int32_T)(NumIters << 1) + ForEach_itr)] + (real32_T)rtp_noise_on_flag *
        (real32_T)localDW[ForEach_itr].CoreSubsys.NextOutput;
      rtb_Switch_j_idx_1 = 1.0F / rtu_P_points_cam_cam[(int32_T)((int32_T)
        (NumIters << 1) + ForEach_itr)] * rtu_P_points_cam_cam[(int32_T)
        (NumIters + ForEach_itr)] * rtp_cam_focal_length_Y + (real32_T)
        rtp_noise_on_flag * (real32_T)localDW[ForEach_itr].
        CoreSubsys.NextOutput_n;
    } else {
      rtb_Switch_j_idx_0 = localP->CoreSubsys.Constant_Value[0];
      rtb_Switch_j_idx_1 = localP->CoreSubsys.Constant_Value[1];
    }

    // End of Switch: '<S106>/Switch'

    // Sqrt: '<S105>/Sqrt' incorporates:
    //   DotProduct: '<S105>/Dot Product'
    //   ForEachSliceSelector: '<S89>/ImpSel_InsertedFor_P_points_cam_cam_at_outport_0'

    rtb_DotProduct1 = (real32_T)sqrt((real_T)((rtu_P_points_cam_cam[(int32_T)
      (NumIters + ForEach_itr)] * rtu_P_points_cam_cam[(int32_T)(NumIters +
      ForEach_itr)] + rtu_P_points_cam_cam[ForEach_itr] *
      rtu_P_points_cam_cam[ForEach_itr]) + rtu_P_points_cam_cam[(int32_T)
      ((int32_T)(NumIters << 1) + ForEach_itr)] * rtu_P_points_cam_cam[(int32_T)
      ((int32_T)(NumIters << 1) + ForEach_itr)]));

    // Update for RandomNumber: '<S106>/pixel_noise'
    localDW[ForEach_itr].CoreSubsys.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&localDW[ForEach_itr].CoreSubsys.RandSeed) * sqrt(rtp_pixel_noise_var) +
      localP->CoreSubsys.pixel_noise_Mean;

    // Update for RandomNumber: '<S106>/pixel_noise1'
    localDW[ForEach_itr].CoreSubsys.NextOutput_n = rt_nrand_Upu32_Yd_f_pw_snf
      (&localDW[ForEach_itr].CoreSubsys.RandSeed_k) * sqrt(rtp_pixel_noise_var)
      + localP->CoreSubsys.pixel_noise1_Mean;

    // ForEachSliceAssignment: '<S89>/ImpAsg_InsertedFor_points_in_FOV_flag_at_inport_0' incorporates:
    //   Abs: '<S105>/Abs'
    //   Abs: '<S105>/Abs1'
    //   Constant: '<S105>/Constant'
    //   Constant: '<S105>/Constant1'
    //   Constant: '<S105>/Constant2'
    //   Constant: '<S105>/Constant3'
    //   Constant: '<S105>/Constant4'
    //   Constant: '<S107>/Constant'
    //   DotProduct: '<S105>/Dot Product1'
    //   ForEachSliceSelector: '<S89>/ImpSel_InsertedFor_P_points_cam_cam_at_outport_0'
    //   Logic: '<S105>/Logical Operator'
    //   RelationalOperator: '<S105>/Relational Operator'
    //   RelationalOperator: '<S105>/Relational Operator1'
    //   RelationalOperator: '<S105>/Relational Operator2'
    //   RelationalOperator: '<S105>/Relational Operator3'
    //   RelationalOperator: '<S107>/Compare'

    rty_points_in_FOV_flag[ForEach_itr] = (((real32_T)fabs((real_T)
      rtb_Switch_j_idx_0) <= rtp_cam_num_X_pixels / 2.0F) && ((real32_T)fabs
      ((real_T)rtb_Switch_j_idx_1) <= rtp_cam_num_Y_pixels / 2.0F) &&
      (rtb_DotProduct1 >= rtp_cam_min_dist) && (rtb_DotProduct1 <=
      rtp_cam_max_dist) && ((rtu_P_points_cam_cam[(int32_T)(NumIters +
      ForEach_itr)] * rtp_cam_pointing[1] + rtu_P_points_cam_cam[ForEach_itr] *
      rtp_cam_pointing[0]) + rtu_P_points_cam_cam[(int32_T)((int32_T)(NumIters <<
      1) + ForEach_itr)] * rtp_cam_pointing[2] >
      localP->CoreSubsys.Constant_Value_n));

    // ForEachSliceAssignment: '<S89>/ImpAsg_InsertedFor_P_points_2D_cam_at_inport_0' 
    rty_P_points_2D_cam[ForEach_itr] = rtb_Switch_j_idx_0;
    rty_P_points_2D_cam[(int32_T)(NumIters + ForEach_itr)] = rtb_Switch_j_idx_1;
  }

  // End of Outputs for SubSystem: '<S85>/pinhole_projection_model'
}

//
// Termination for iterator system:
//    '<S85>/pinhole_projection_model'
//    '<S113>/pinhole_projection_model'
//    '<S137>/pinhole_projection_model'
//    '<S161>/pinhole_projection_model'
//
void pinhole_projection_mod_Term(void)
{
}

//
// System initialize for enable system:
//    '<S188>/latch_nozzle_thrust_matricies'
//    '<S221>/latch_nozzle_thrust_matricies'
//
void latch_nozzle_thrust_ma_Init(B_latch_nozzle_thrust_matrici_T *localB,
  P_latch_nozzle_thrust_matrici_T *localP)
{
  int32_T i;
  for (i = 0; i < 18; i++) {
    // SystemInitialize for Outport: '<S196>/thrust2torque_B'
    localB->OutportBufferForthrust2torque_B[i] = localP->thrust2torque_B_Y0;

    // SystemInitialize for Outport: '<S196>/thrust2force_B'
    localB->OutportBufferForthrust2force_B[i] = localP->thrust2force_B_Y0;
  }
}

//
// Output and update for enable system:
//    '<S188>/latch_nozzle_thrust_matricies'
//    '<S221>/latch_nozzle_thrust_matricies'
//
void latch_nozzle_thrust_matrici(boolean_T rtu_Enable, const real32_T
  rtu_veh_cm[3], B_latch_nozzle_thrust_matrici_T *localB,
  P_latch_nozzle_thrust_matrici_T *localP, const real32_T
  rtp_nozzle_orientation_offset[24], const real32_T rtp_P_nozzle_B_B[18], const
  real32_T rtp_nozzle_orientation_B[18], const real32_T rtp_P_nozzle_B_B_offset
  [18], const real_T rtp_P_CG_B_B_error[3], real32_T rtp_noise_on_flag)
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  real32_T nozzle_moment_arm[18];
  real_T b[18];
  int32_T ibmat;
  int32_T itilerow;
  real32_T c[18];
  real32_T rtb_Assignment_h2[9];
  real32_T rtb_Sum_bc;
  real32_T rtb_ImpAsg_InsertedFor_rotated_[18];
  real_T rtb_Sum2_c[3];
  int32_T jcol;
  real32_T tmp[9];
  real32_T rtb_Switch_e_0[9];
  real32_T rtb_Assignment_a[9];
  const real32_T *rtb_Switch_e_1;

  // Outputs for Enabled SubSystem: '<S188>/latch_nozzle_thrust_matricies' incorporates:
  //   EnablePort: '<S196>/Enable'

  if (rtu_Enable) {
    // Switch: '<S196>/Switch' incorporates:
    //   Constant: '<S196>/Constant2'
    //   Constant: '<S196>/Constant5'
    //   Constant: '<S196>/Constant8'

    if (rtp_noise_on_flag > localP->Switch_Threshold) {
      rtb_Switch_e_1 = &rtp_nozzle_orientation_offset[0];
    } else {
      rtb_Switch_e_1 = &localP->Constant2_Value[0];
    }

    // End of Switch: '<S196>/Switch'

    // Outputs for Iterator SubSystem: '<S196>/For Each Subsystem' incorporates:
    //   ForEach: '<S200>/For Each'

    for (ForEach_itr = 0; ForEach_itr < 6; ForEach_itr++) {
      // Sum: '<S203>/Sum' incorporates:
      //   Constant: '<S203>/Constant1'
      //   DataTypeConversion: '<S205>/Conversion'
      //   ForEachSliceSelector: '<S200>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S203>/Gain'
      //   Math: '<S203>/Math Function'

      rtb_Sum_bc = rtb_Switch_e_1[(int32_T)(18 + ForEach_itr)] * rtb_Switch_e_1
        [(int32_T)(18 + ForEach_itr)] * localP->CoreSubsys.Gain_Gain - (real32_T)
        localP->CoreSubsys.Constant1_Value;

      // Assignment: '<S203>/Assignment' incorporates:
      //   Constant: '<S203>/Constant2'
      //   DataTypeConversion: '<S204>/Conversion'

      for (jcol = 0; jcol < 9; jcol++) {
        rtb_Assignment_h2[jcol] = (real32_T)localP->
          CoreSubsys.Constant2_Value[jcol];
      }

      rtb_Assignment_h2[0] = rtb_Sum_bc;
      rtb_Assignment_h2[4] = rtb_Sum_bc;
      rtb_Assignment_h2[8] = rtb_Sum_bc;

      // End of Assignment: '<S203>/Assignment'

      // Gain: '<S203>/Gain1' incorporates:
      //   ForEachSliceSelector: '<S200>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'

      rtb_Sum_bc = rtb_Switch_e_1[(int32_T)(18 + ForEach_itr)] *
        localP->CoreSubsys.Gain1_Gain;

      // Product: '<S203>/Product' incorporates:
      //   Constant: '<S206>/Constant3'
      //   DataTypeConversion: '<S207>/Conversion'
      //   ForEachSliceSelector: '<S200>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S206>/Gain'
      //   Gain: '<S206>/Gain1'
      //   Gain: '<S206>/Gain2'

      tmp[0] = (real32_T)localP->CoreSubsys.Constant3_Value;
      tmp[1] = rtb_Switch_e_1[(int32_T)(12 + ForEach_itr)];
      tmp[2] = rtb_Switch_e_1[(int32_T)(6 + ForEach_itr)] *
        localP->CoreSubsys.Gain_Gain_k;
      tmp[3] = rtb_Switch_e_1[(int32_T)(12 + ForEach_itr)] *
        localP->CoreSubsys.Gain1_Gain_f;
      tmp[4] = (real32_T)localP->CoreSubsys.Constant3_Value;
      tmp[5] = rtb_Switch_e_1[ForEach_itr];
      tmp[6] = rtb_Switch_e_1[(int32_T)(ForEach_itr + 6)];
      tmp[7] = localP->CoreSubsys.Gain2_Gain * rtb_Switch_e_1[ForEach_itr];
      tmp[8] = (real32_T)localP->CoreSubsys.Constant3_Value;

      // Product: '<S203>/Product1' incorporates:
      //   ForEachSliceSelector: '<S200>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S203>/Gain2'

      for (jcol = 0; jcol < 3; jcol++) {
        rtb_Switch_e_0[jcol] = rtb_Switch_e_1[(int32_T)((int32_T)(6 * jcol) +
          ForEach_itr)] * rtb_Switch_e_1[ForEach_itr];
        rtb_Switch_e_0[(int32_T)(jcol + 3)] = rtb_Switch_e_1[(int32_T)((int32_T)
          (6 * jcol) + ForEach_itr)] * rtb_Switch_e_1[(int32_T)(ForEach_itr + 6)];
        rtb_Switch_e_0[(int32_T)(jcol + 6)] = rtb_Switch_e_1[(int32_T)((int32_T)
          (6 * jcol) + ForEach_itr)] * rtb_Switch_e_1[(int32_T)(ForEach_itr + 12)];
      }

      // End of Product: '<S203>/Product1'

      // Sum: '<S203>/Sum1' incorporates:
      //   Gain: '<S203>/Gain2'
      //   Product: '<S202>/Product'
      //   Product: '<S203>/Product'

      for (jcol = 0; jcol < 3; jcol++) {
        rtb_Assignment_a[(int32_T)(3 * jcol)] = (rtb_Assignment_h2[(int32_T)(3 *
          jcol)] - tmp[(int32_T)(3 * jcol)] * rtb_Sum_bc) + rtb_Switch_e_0
          [(int32_T)(3 * jcol)] * localP->CoreSubsys.Gain2_Gain_k;
        rtb_Assignment_a[(int32_T)(1 + (int32_T)(3 * jcol))] =
          (rtb_Assignment_h2[(int32_T)((int32_T)(3 * jcol) + 1)] - tmp[(int32_T)
           ((int32_T)(3 * jcol) + 1)] * rtb_Sum_bc) + rtb_Switch_e_0[(int32_T)
          ((int32_T)(3 * jcol) + 1)] * localP->CoreSubsys.Gain2_Gain_k;
        rtb_Assignment_a[(int32_T)(2 + (int32_T)(3 * jcol))] =
          (rtb_Assignment_h2[(int32_T)((int32_T)(3 * jcol) + 2)] - tmp[(int32_T)
           ((int32_T)(3 * jcol) + 2)] * rtb_Sum_bc) + rtb_Switch_e_0[(int32_T)
          ((int32_T)(3 * jcol) + 2)] * localP->CoreSubsys.Gain2_Gain_k;
      }

      // End of Sum: '<S203>/Sum1'
      for (jcol = 0; jcol < 3; jcol++) {
        // ForEachSliceAssignment: '<S200>/ImpAsg_InsertedFor_rotated_vectors_at_inport_0' incorporates:
        //   Constant: '<S196>/Constant3'
        //   ForEachSliceSelector: '<S200>/ImpSel_InsertedFor_unit_vectors_at_outport_0'
        //   Product: '<S202>/Product'

        rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(ForEach_itr + (int32_T)(6 *
          jcol))] = rtb_Assignment_a[(int32_T)(jcol + 6)] *
          rtp_nozzle_orientation_B[(int32_T)(ForEach_itr + 12)] +
          (rtb_Assignment_a[(int32_T)(jcol + 3)] * rtp_nozzle_orientation_B
           [(int32_T)(ForEach_itr + 6)] + rtb_Assignment_a[jcol] *
           rtp_nozzle_orientation_B[ForEach_itr]);
      }
    }

    // End of Outputs for SubSystem: '<S196>/For Each Subsystem'
    // MATLAB Function 'bpm_blower_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/MATLAB Function': '<S201>:1' 
    // NOTE: if number of nozzles change, must change this hard coded '6' below
    // '<S201>:1:5'
    for (jcol = 0; jcol < 3; jcol++) {
      // Sum: '<S196>/Sum2' incorporates:
      //   Constant: '<S196>/Constant7'
      //   Gain: '<S196>/Gain2'

      rtb_Sum2_c[jcol] = (real_T)rtp_noise_on_flag * rtp_P_CG_B_B_error[jcol] +
        (real_T)rtu_veh_cm[jcol];

      // MATLAB Function: '<S196>/MATLAB Function'
      ibmat = (int32_T)(jcol * 6);
      for (itilerow = 0; itilerow < 6; itilerow++) {
        b[(int32_T)(ibmat + itilerow)] = rtb_Sum2_c[jcol];
      }
    }

    // MATLAB Function: '<S196>/MATLAB Function' incorporates:
    //   Constant: '<S196>/Constant1'
    //   Constant: '<S196>/Constant4'
    //   Gain: '<S196>/Gain'
    //   Sum: '<S196>/Sum'

    for (jcol = 0; jcol < 18; jcol++) {
      nozzle_moment_arm[jcol] = (rtp_noise_on_flag *
        rtp_P_nozzle_B_B_offset[jcol] + rtp_P_nozzle_B_B[jcol]) - (real32_T)
        b[jcol];
    }

    // [m] Distance from nozzles to CG
    // '<S201>:1:7'
    // [-] Converts a nozzle thrust into resulting body torques
    // '<S201>:1:8'
    // [-] Converts a nozzle thrust into resulting body forces
    for (jcol = 0; jcol < 6; jcol++) {
      c[jcol] = nozzle_moment_arm[(int32_T)(jcol + 6)] *
        rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(jcol + 12)] -
        nozzle_moment_arm[(int32_T)(jcol + 12)] *
        rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(jcol + 6)];
      c[(int32_T)(jcol + 6)] = nozzle_moment_arm[(int32_T)(jcol + 12)] *
        rtb_ImpAsg_InsertedFor_rotated_[jcol] - rtb_ImpAsg_InsertedFor_rotated_
        [(int32_T)(jcol + 12)] * nozzle_moment_arm[jcol];
      c[(int32_T)(jcol + 12)] = rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(jcol +
        6)] * nozzle_moment_arm[jcol] - nozzle_moment_arm[(int32_T)(jcol + 6)] *
        rtb_ImpAsg_InsertedFor_rotated_[jcol];

      // SignalConversion: '<S196>/OutportBufferForthrust2force_B'
      localB->OutportBufferForthrust2force_B[(int32_T)(3 * jcol)] =
        -rtb_ImpAsg_InsertedFor_rotated_[jcol];
      localB->OutportBufferForthrust2force_B[(int32_T)(1 + (int32_T)(3 * jcol))]
        = -rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(jcol + 6)];
      localB->OutportBufferForthrust2force_B[(int32_T)(2 + (int32_T)(3 * jcol))]
        = -rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(jcol + 12)];

      // SignalConversion: '<S196>/OutportBufferForthrust2torque_B'
      localB->OutportBufferForthrust2torque_B[(int32_T)(3 * jcol)] = -c[jcol];
      localB->OutportBufferForthrust2torque_B[(int32_T)(1 + (int32_T)(3 * jcol))]
        = -c[(int32_T)(jcol + 6)];
      localB->OutportBufferForthrust2torque_B[(int32_T)(2 + (int32_T)(3 * jcol))]
        = -c[(int32_T)(jcol + 12)];
    }
  }

  // End of Outputs for SubSystem: '<S188>/latch_nozzle_thrust_matricies'
}

//
// Termination for enable system:
//    '<S188>/latch_nozzle_thrust_matricies'
//    '<S221>/latch_nozzle_thrust_matricies'
//
void latch_nozzle_thrust_ma_Term(void)
{
}

//
// Output and update for atomic system:
//    '<S184>/calc_nozzle_area'
//    '<S185>/calc_nozzle_area'
//
void sim_model__calc_nozzle_area(const real32_T rtu_servo_theta[6],
  B_calc_nozzle_area_sim_model__T *localB, real32_T rtp_noz_flap_count, real32_T
  rtp_noz_min_theta, real32_T rtp_noz_flap_length, real32_T
  rtp_noz_intake_height, const real32_T rtp_noz_width[6], real32_T
  rtp_noz_gear_ratio)
{
  real32_T y;
  int32_T i;

  // Gain: '<S189>/Gain12'
  y = 1.0F / rtp_noz_gear_ratio;
  for (i = 0; i < 6; i++) {
    localB->Gain12[i] = y * rtu_servo_theta[i];

    // Product: '<S189>/Product2' incorporates:
    //   Constant: '<S189>/Constant1'
    //   Constant: '<S189>/Constant2'
    //   Constant: '<S189>/Constant4'
    //   Constant: '<S189>/Constant5'
    //   Constant: '<S189>/Constant6'
    //   Product: '<S189>/Product1'
    //   Sum: '<S189>/Subtract'
    //   Sum: '<S189>/Subtract1'
    //   Trigonometry: '<S189>/Trigonometric Function'

    localB->Product2[i] = (rtp_noz_intake_height - (real32_T)cos((real_T)
      (localB->Gain12[i] + rtp_noz_min_theta)) * rtp_noz_flap_length) *
      rtp_noz_width[i] * rtp_noz_flap_count;
  }

  // End of Gain: '<S189>/Gain12'
}

//
// Termination for atomic system:
//    '<S184>/calc_nozzle_area'
//    '<S185>/calc_nozzle_area'
//
void sim_m_calc_nozzle_area_Term(void)
{
}

//
// Output and update for atomic system:
//    '<S190>/dc_motor_model'
//    '<S223>/dc_motor_model'
//
void sim_model_li_dc_motor_model(real32_T rtu_ctl_voltage, real32_T
  rtu_curr_rotor_speed, B_dc_motor_model_sim_model_li_T *localB, real32_T
  rtp_motor_kn, real32_T rtp_motor_r, real32_T rtp_motor_kt, real32_T
  rtp_motor_friction_coeff)
{
  // Gain: '<S211>/Gain5' incorporates:
  //   Gain: '<S211>/Gain4'
  //   Sum: '<S211>/Add'

  localB->current = (rtu_ctl_voltage - 1.0F / rtp_motor_kn *
                     rtu_curr_rotor_speed) * (1.0F / rtp_motor_r);

  // Sum: '<S211>/Add1' incorporates:
  //   Gain: '<S211>/Gain1'
  //   Gain: '<S211>/Gain6'

  localB->output_torque = rtp_motor_kt * localB->current -
    rtp_motor_friction_coeff * rtu_curr_rotor_speed;
}

//
// Termination for atomic system:
//    '<S190>/dc_motor_model'
//    '<S223>/dc_motor_model'
//
void sim_mod_dc_motor_model_Term(void)
{
}

//
// System initialize for atomic system:
//    '<S190>/speed_controller'
//    '<S223>/speed_controller'
//
void sim_m_speed_controller_Init(DW_speed_controller_sim_model_T *localDW,
  P_speed_controller_sim_model__T *localP)
{
  // InitializeConditions for DiscreteIntegrator: '<S213>/Integrator'
  localDW->Integrator_DSTATE = localP->Integrator_IC;

  // InitializeConditions for DiscreteIntegrator: '<S213>/Filter'
  localDW->Filter_DSTATE = localP->Filter_IC;
}

//
// Output and update for atomic system:
//    '<S190>/speed_controller'
//    '<S223>/speed_controller'
//
void sim_model__speed_controller(real32_T rtu_battery_voltage, uint8_T
  rtu_speed_cmd, real32_T rtu_speed_curr, B_speed_controller_sim_model__T
  *localB, DW_speed_controller_sim_model_T *localDW,
  P_speed_controller_sim_model__T *localP, real32_T rtp_ctl_pwm2speed, real32_T
  rtp_ctl_cmd_filt_num, real32_T rtp_ctl_cmd_filt_den, real32_T
  rtp_ctl_speed_filt_num, real32_T rtp_ctl_speed_cilt_den, real32_T rtp_ctl_kp,
  real32_T rtp_ctl_kd, real32_T rtp_ctl_filt_n, real32_T rtp_ctl_max_voltage,
  real32_T rtp_ctl_ki)
{
  real32_T rtb_Divide_i;
  real32_T rtb_IntegralGain;
  real32_T rtb_Sum_k;
  real32_T rtb_SignPreIntegrator;
  boolean_T rtb_NotEqual;
  real32_T rtb_Gain_pl;
  int8_T rtb_SignPreIntegrator_0;

  // Sum: '<S212>/Sum' incorporates:
  //   Constant: '<S212>/Constant'
  //   DiscreteTransferFcn: '<S212>/Discrete Transfer Fcn'
  //   DiscreteTransferFcn: '<S212>/Discrete Transfer Fcn1'
  //   Product: '<S212>/Divide'

  rtb_IntegralGain = (real32_T)rtu_speed_cmd * rtp_ctl_pwm2speed *
    rtp_ctl_cmd_filt_num / rtp_ctl_cmd_filt_den - rtu_speed_curr *
    rtp_ctl_speed_filt_num / rtp_ctl_speed_cilt_den;

  // Gain: '<S213>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S213>/Filter'
  //   Gain: '<S213>/Derivative Gain'
  //   Sum: '<S213>/SumD'

  rtb_Divide_i = (rtp_ctl_kd * rtb_IntegralGain - localDW->Filter_DSTATE) *
    rtp_ctl_filt_n;

  // Sum: '<S213>/Sum' incorporates:
  //   DiscreteIntegrator: '<S213>/Integrator'
  //   Gain: '<S213>/Proportional Gain'

  rtb_Sum_k = (rtp_ctl_kp * rtb_IntegralGain + localDW->Integrator_DSTATE) +
    rtb_Divide_i;

  // DeadZone: '<S215>/DeadZone'
  if (rtb_Sum_k > rtp_ctl_max_voltage) {
    rtb_SignPreIntegrator = rtb_Sum_k - rtp_ctl_max_voltage;
  } else if (rtb_Sum_k >= localP->DiscretePIDController_LowerSatu) {
    rtb_SignPreIntegrator = 0.0F;
  } else {
    rtb_SignPreIntegrator = rtb_Sum_k - localP->DiscretePIDController_LowerSatu;
  }

  // End of DeadZone: '<S215>/DeadZone'

  // RelationalOperator: '<S215>/NotEqual' incorporates:
  //   Gain: '<S215>/ZeroGain'

  rtb_NotEqual = (localP->ZeroGain_Gain * rtb_Sum_k != rtb_SignPreIntegrator);

  // Signum: '<S215>/SignDeltaU'
  if (rtb_SignPreIntegrator < 0.0F) {
    rtb_SignPreIntegrator = -1.0F;
  } else if (rtb_SignPreIntegrator > 0.0F) {
    rtb_SignPreIntegrator = 1.0F;
  } else {
    if (rtb_SignPreIntegrator == 0.0F) {
      rtb_SignPreIntegrator = 0.0F;
    }
  }

  // End of Signum: '<S215>/SignDeltaU'

  // Gain: '<S213>/Integral Gain'
  rtb_IntegralGain *= rtp_ctl_ki;

  // Saturate: '<S213>/Saturate'
  if (rtb_Sum_k > rtp_ctl_max_voltage) {
    rtb_Sum_k = rtp_ctl_max_voltage;
  } else {
    if (rtb_Sum_k < localP->DiscretePIDController_LowerSatu) {
      rtb_Sum_k = localP->DiscretePIDController_LowerSatu;
    }
  }

  // End of Saturate: '<S213>/Saturate'

  // Switch: '<S214>/Switch2' incorporates:
  //   RelationalOperator: '<S214>/LowerRelop1'

  if (rtb_Sum_k > rtu_battery_voltage) {
    localB->Switch2 = rtu_battery_voltage;
  } else {
    // Gain: '<S212>/Gain'
    rtb_Gain_pl = localP->Gain_Gain * rtu_battery_voltage;

    // Switch: '<S214>/Switch' incorporates:
    //   RelationalOperator: '<S214>/UpperRelop'

    if (rtb_Sum_k < rtb_Gain_pl) {
      localB->Switch2 = rtb_Gain_pl;
    } else {
      localB->Switch2 = rtb_Sum_k;
    }

    // End of Switch: '<S214>/Switch'
  }

  // End of Switch: '<S214>/Switch2'

  // Signum: '<S215>/SignPreIntegrator'
  if (rtb_IntegralGain < 0.0F) {
    // DataTypeConversion: '<S215>/DataTypeConv2'
    rtb_Sum_k = -1.0F;
  } else if (rtb_IntegralGain > 0.0F) {
    // DataTypeConversion: '<S215>/DataTypeConv2'
    rtb_Sum_k = 1.0F;
  } else if (rtb_IntegralGain == 0.0F) {
    // DataTypeConversion: '<S215>/DataTypeConv2'
    rtb_Sum_k = 0.0F;
  } else {
    // DataTypeConversion: '<S215>/DataTypeConv2'
    rtb_Sum_k = rtb_IntegralGain;
  }

  // End of Signum: '<S215>/SignPreIntegrator'

  // DataTypeConversion: '<S215>/DataTypeConv2'
  if (rtIsNaNF(rtb_Sum_k) || rtIsInfF(rtb_Sum_k)) {
    rtb_Sum_k = 0.0F;
  } else {
    rtb_Sum_k = (real32_T)fmod((real_T)rtb_Sum_k, (real_T)256.0F);
  }

  // DataTypeConversion: '<S215>/DataTypeConv1'
  if (rtb_SignPreIntegrator < 128.0F) {
    rtb_SignPreIntegrator_0 = (int8_T)rtb_SignPreIntegrator;
  } else {
    rtb_SignPreIntegrator_0 = MAX_int8_T;
  }

  // End of DataTypeConversion: '<S215>/DataTypeConv1'

  // Switch: '<S213>/Switch' incorporates:
  //   Constant: '<S213>/Constant'
  //   DataTypeConversion: '<S215>/DataTypeConv2'
  //   Logic: '<S215>/AND'
  //   RelationalOperator: '<S215>/Equal'

  if (rtb_NotEqual && ((rtb_Sum_k < 0.0F ? (int32_T)(int8_T)(int32_T)-(int32_T)
                        (int8_T)(uint8_T)-rtb_Sum_k : (int32_T)(int8_T)(uint8_T)
                        rtb_Sum_k) == (int32_T)rtb_SignPreIntegrator_0)) {
    rtb_IntegralGain = localP->Constant_Value;
  }

  // End of Switch: '<S213>/Switch'

  // Update for DiscreteIntegrator: '<S213>/Integrator'
  localDW->Integrator_DSTATE += localP->Integrator_gainval * rtb_IntegralGain;

  // Update for DiscreteIntegrator: '<S213>/Filter'
  localDW->Filter_DSTATE += localP->Filter_gainval * rtb_Divide_i;
}

//
// Termination for atomic system:
//    '<S190>/speed_controller'
//    '<S223>/speed_controller'
//
void sim_m_speed_controller_Term(void)
{
}

//
// System initialize for atomic system:
//    '<S184>/servo_model'
//    '<S185>/servo_model'
//
void sim_model__servo_model_Init(DW_servo_model_sim_model_lib0_T *localDW,
  P_servo_model_sim_model_lib0_T *localP, real32_T rtp_servo_min_theta, real32_T
  rtp_motor_gear_ratio)
{
  real32_T y;
  int32_T i;

  // InitializeConditions for DiscreteIntegrator: '<S216>/Discrete-Time Integrator4' 
  y = rtp_servo_min_theta / rtp_motor_gear_ratio;
  for (i = 0; i < 6; i++) {
    localDW->DiscreteTimeIntegrator4_DSTATE[i] = y;

    // InitializeConditions for Backlash: '<S216>/Backlash1'
    localDW->PrevY[i] = localP->Backlash1_InitialOutput;

    // InitializeConditions for DiscreteIntegrator: '<S218>/Integrator'
    localDW->Integrator_DSTATE[i] = localP->Integrator_IC;

    // InitializeConditions for DiscreteIntegrator: '<S218>/Filter'
    localDW->Filter_DSTATE[i] = localP->Filter_IC;

    // InitializeConditions for DiscreteIntegrator: '<S216>/Discrete-Time Integrator3' 
    localDW->DiscreteTimeIntegrator3_DSTATE[i] =
      localP->DiscreteTimeIntegrator3_IC;
  }

  // End of InitializeConditions for DiscreteIntegrator: '<S216>/Discrete-Time Integrator4' 
}

//
// Output and update for atomic system:
//    '<S184>/servo_model'
//    '<S185>/servo_model'
//
void sim_model_lib0_servo_model(const real32_T rtu_command_PWM[6],
  B_servo_model_sim_model_lib0_T *localB, DW_servo_model_sim_model_lib0_T
  *localDW, P_servo_model_sim_model_lib0_T *localP, real32_T
  rtp_servo_pwm2angle_bias, real32_T rtp_servo_pwm2angle, real32_T
  rtp_servo_min_theta, real32_T rtp_motor_gear_ratio, real32_T
  rtp_servo_max_theta, real32_T rtp_motor_backlash_deadband, real32_T
  rtp_servo_ctl_kp, real32_T rtp_servo_ctl_kd, real32_T rtp_servo_ctl_filt_n,
  real32_T rtp_servo_max_voltage, real32_T rtp_motor_k, real32_T rtp_motor_r,
  real32_T rtp_motor_friction_coeff, real32_T rtp_motor_gear_box_inertia,
  real32_T rtp_servo_ctl_ki)
{
  real32_T halfDeadBand;
  real32_T y;
  real32_T rtb_IntegralGain[6];
  real32_T rtb_SignDeltaU[6];
  real32_T rtb_FilterCoefficient_o[6];
  boolean_T rtb_NotEqual[6];
  real32_T tmp;
  real32_T tmp_0;
  real32_T tmp_1;
  int32_T i;
  real32_T rtb_IntegralGain_c;
  int8_T rtb_SignDeltaU_0;

  // Backlash: '<S216>/Backlash1' incorporates:
  //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator4'
  //   Gain: '<S218>/Filter Coefficient'
  //   Sum: '<S217>/Sum1'

  halfDeadBand = rtp_motor_backlash_deadband / 2.0F;
  for (i = 0; i < 6; i++) {
    if (localDW->DiscreteTimeIntegrator4_DSTATE[i] < localDW->PrevY[i] -
        halfDeadBand) {
      localB->Backlash1[i] = localDW->DiscreteTimeIntegrator4_DSTATE[i] +
        halfDeadBand;
    } else if (localDW->DiscreteTimeIntegrator4_DSTATE[i] <= localDW->PrevY[i] +
               halfDeadBand) {
      localB->Backlash1[i] = localDW->PrevY[i];
    } else {
      localB->Backlash1[i] = localDW->DiscreteTimeIntegrator4_DSTATE[i] -
        halfDeadBand;
    }

    // Gain: '<S216>/Gain1' incorporates:
    //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator4'

    localB->Gain1[i] = rtp_motor_gear_ratio * localB->Backlash1[i];

    // Sum: '<S217>/Sum1' incorporates:
    //   Constant: '<S217>/Constant'
    //   Gain: '<S217>/Gain'
    //   Sum: '<S217>/Sum'

    rtb_IntegralGain_c = (rtu_command_PWM[i] + rtp_servo_pwm2angle_bias) *
      rtp_servo_pwm2angle - localB->Gain1[i];

    // Gain: '<S218>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S218>/Filter'
    //   Gain: '<S218>/Derivative Gain'
    //   Sum: '<S218>/SumD'

    y = (rtp_servo_ctl_kd * rtb_IntegralGain_c - localDW->Filter_DSTATE[i]) *
      rtp_servo_ctl_filt_n;

    // Sum: '<S218>/Sum' incorporates:
    //   DiscreteIntegrator: '<S218>/Integrator'
    //   Gain: '<S218>/Proportional Gain'

    rtb_SignDeltaU[i] = (rtp_servo_ctl_kp * rtb_IntegralGain_c +
                         localDW->Integrator_DSTATE[i]) + y;
    rtb_IntegralGain[i] = rtb_IntegralGain_c;
    rtb_FilterCoefficient_o[i] = y;
  }

  // End of Backlash: '<S216>/Backlash1'

  // Gain: '<S216>/Gain5' incorporates:
  //   DeadZone: '<S219>/DeadZone'
  //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator3'
  //   Gain: '<S216>/Gain7'
  //   Gain: '<S218>/Integral Gain'
  //   Sum: '<S216>/Add3'

  halfDeadBand = 1.0F / rtp_motor_r;
  for (i = 0; i < 6; i++) {
    // Saturate: '<S218>/Saturate'
    if (rtb_SignDeltaU[i] > rtp_servo_max_voltage) {
      y = rtp_servo_max_voltage;
    } else if (rtb_SignDeltaU[i] < localP->DiscretePIDController_LowerSatu) {
      y = localP->DiscretePIDController_LowerSatu;
    } else {
      y = rtb_SignDeltaU[i];
    }

    localB->current[i] = (y - rtp_motor_k *
                          localDW->DiscreteTimeIntegrator3_DSTATE[i]) *
      halfDeadBand;

    // DeadZone: '<S219>/DeadZone' incorporates:
    //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator3'
    //   Gain: '<S216>/Gain7'
    //   Saturate: '<S218>/Saturate'
    //   Sum: '<S216>/Add3'

    if (rtb_SignDeltaU[i] > rtp_servo_max_voltage) {
      y = rtb_SignDeltaU[i] - rtp_servo_max_voltage;
    } else if (rtb_SignDeltaU[i] >= localP->DiscretePIDController_LowerSatu) {
      y = 0.0F;
    } else {
      y = rtb_SignDeltaU[i] - localP->DiscretePIDController_LowerSatu;
    }

    // RelationalOperator: '<S219>/NotEqual' incorporates:
    //   Gain: '<S219>/ZeroGain'
    //   Saturate: '<S218>/Saturate'

    rtb_NotEqual[i] = (localP->ZeroGain_Gain * rtb_SignDeltaU[i] != y);

    // Signum: '<S219>/SignDeltaU'
    if (y < 0.0F) {
      y = -1.0F;
    } else if (y > 0.0F) {
      y = 1.0F;
    } else {
      if (y == 0.0F) {
        y = 0.0F;
      }
    }

    // End of Signum: '<S219>/SignDeltaU'

    // Update for DiscreteIntegrator: '<S216>/Discrete-Time Integrator4' incorporates:
    //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator3'

    localDW->DiscreteTimeIntegrator4_DSTATE[i] +=
      localP->DiscreteTimeIntegrator4_gainval *
      localDW->DiscreteTimeIntegrator3_DSTATE[i];
    rtb_SignDeltaU[i] = y;
    rtb_IntegralGain[i] *= rtp_servo_ctl_ki;
  }

  // End of Gain: '<S216>/Gain5'

  // Gain: '<S216>/Gain6'
  halfDeadBand = 1.0F / rtp_motor_gear_box_inertia;

  // Update for DiscreteIntegrator: '<S216>/Discrete-Time Integrator4'
  y = rtp_servo_max_theta / rtp_motor_gear_ratio;
  tmp = rtp_servo_min_theta / rtp_motor_gear_ratio;
  tmp_0 = rtp_servo_max_theta / rtp_motor_gear_ratio;
  tmp_1 = rtp_servo_min_theta / rtp_motor_gear_ratio;
  for (i = 0; i < 6; i++) {
    if (localDW->DiscreteTimeIntegrator4_DSTATE[i] >= y) {
      localDW->DiscreteTimeIntegrator4_DSTATE[i] = tmp_0;
    } else {
      if (localDW->DiscreteTimeIntegrator4_DSTATE[i] <= tmp) {
        localDW->DiscreteTimeIntegrator4_DSTATE[i] = tmp_1;
      }
    }

    // Update for Backlash: '<S216>/Backlash1'
    localDW->PrevY[i] = localB->Backlash1[i];

    // Update for DiscreteIntegrator: '<S218>/Integrator' incorporates:
    //   Constant: '<S218>/Constant'
    //   DataTypeConversion: '<S219>/DataTypeConv1'
    //   DataTypeConversion: '<S219>/DataTypeConv2'
    //   Logic: '<S219>/AND'
    //   RelationalOperator: '<S219>/Equal'
    //   Signum: '<S219>/SignPreIntegrator'
    //   Switch: '<S218>/Switch'

    if (rtb_IntegralGain[i] < 0.0F) {
      rtb_IntegralGain_c = -1.0F;
    } else if (rtb_IntegralGain[i] > 0.0F) {
      rtb_IntegralGain_c = 1.0F;
    } else if (rtb_IntegralGain[i] == 0.0F) {
      rtb_IntegralGain_c = 0.0F;
    } else {
      rtb_IntegralGain_c = rtb_IntegralGain[i];
    }

    rtb_IntegralGain_c = (real32_T)floor((real_T)rtb_IntegralGain_c);
    if (rtIsNaNF(rtb_IntegralGain_c) || rtIsInfF(rtb_IntegralGain_c)) {
      rtb_IntegralGain_c = 0.0F;
    } else {
      rtb_IntegralGain_c = (real32_T)fmod((real_T)rtb_IntegralGain_c, (real_T)
        256.0F);
    }

    if (rtb_SignDeltaU[i] < 128.0F) {
      if (rtb_SignDeltaU[i] >= -128.0F) {
        rtb_SignDeltaU_0 = (int8_T)rtb_SignDeltaU[i];
      } else {
        rtb_SignDeltaU_0 = MIN_int8_T;
      }
    } else {
      rtb_SignDeltaU_0 = MAX_int8_T;
    }

    if (rtb_NotEqual[i] && ((rtb_IntegralGain_c < 0.0F ? (int32_T)(int8_T)
          (int32_T)-(int32_T)(int8_T)(uint8_T)-rtb_IntegralGain_c : (int32_T)
          (int8_T)(uint8_T)rtb_IntegralGain_c) == (int32_T)rtb_SignDeltaU_0)) {
      rtb_IntegralGain_c = localP->Constant_Value;
    } else {
      rtb_IntegralGain_c = rtb_IntegralGain[i];
    }

    localDW->Integrator_DSTATE[i] += localP->Integrator_gainval *
      rtb_IntegralGain_c;

    // End of Update for DiscreteIntegrator: '<S218>/Integrator'

    // Update for DiscreteIntegrator: '<S218>/Filter'
    localDW->Filter_DSTATE[i] += localP->Filter_gainval *
      rtb_FilterCoefficient_o[i];

    // Update for DiscreteIntegrator: '<S216>/Discrete-Time Integrator3' incorporates:
    //   DiscreteIntegrator: '<S216>/Discrete-Time Integrator3'
    //   Gain: '<S216>/Gain11'
    //   Gain: '<S216>/Gain6'
    //   Gain: '<S216>/Gain9'
    //   Sum: '<S216>/Add1'

    localDW->DiscreteTimeIntegrator3_DSTATE[i] += (rtp_motor_k * localB->
      current[i] - rtp_motor_friction_coeff *
      localDW->DiscreteTimeIntegrator3_DSTATE[i]) * halfDeadBand *
      localP->DiscreteTimeIntegrator3_gainval;
  }
}

//
// Termination for atomic system:
//    '<S184>/servo_model'
//    '<S185>/servo_model'
//
void sim_model__servo_model_Term(void)
{
}

// Function for MATLAB Function: '<S79>/generate_output'
static void sim_model__format_AR_tag_output(const real32_T points_in_iss[108],
  const real32_T points_in_cam[72], boolean_T valid_in[36], real32_T
  points_out_iss[150], real32_T points_out_cam[100], real32_T valid_out[50])
{
  int32_T b_i;
  int32_T c_i;
  int32_T d_i;
  int32_T i;
  boolean_T tmp;
  int8_T b_data[36];
  int32_T b_sizes;
  real32_T points_in_iss_data[258];
  real32_T points_in_cam_data[172];
  int8_T valid_in_data[86];
  int32_T points_in_iss_sizes_idx_0;

  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  // format_AR_tag_output.m
  //  inputs:
  //    num_pts_out    = Required number of points to be reported
  //    points_in iss  = ALL [x,y,z] position of points in the ISS frame
  //    [NOTE: for AR tags, these are grouped in 4's.  first 4 = 1 tag, second  4 = another tag] 
  //    points_in_cam  = ALL [u,v] pixel location of points in the camera
  //    valid_in       = indicates which points are valid
  //
  //  outputs:
  //    points_out_iss = output [x,y,z] position of points in the ISS frame      
  //    points_out_cam = output [u,v] pixel location of points in the camera
  //    valid_out      = indicates which values of the output are valid!
  // initialize the outputs
  for (b_i = 0; b_i < 9; b_i++) {
    c_i = (int32_T)(b_i << 2);
    tmp = kfcjlnglmgdbgdjm_all((boolean_T *)&valid_in[c_i]);
    valid_in[c_i] = tmp;
    valid_in[(int32_T)(1 + c_i)] = tmp;
    valid_in[(int32_T)(2 + c_i)] = tmp;
    valid_in[(int32_T)(3 + c_i)] = tmp;

    // all 4 tags must be valid, or they are all invalid
  }

  // find how many pts are valid, and determine how that compares to required number 
  b_i = 0;
  for (c_i = 0; c_i < 36; c_i++) {
    if (valid_in[c_i]) {
      b_i++;
    }
  }

  // if we need more points, back fill with zeros, and zero the valid cooresponding valid flag 
  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      c_i++;
    }
  }

  b_sizes = c_i;
  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      b_data[c_i] = (int8_T)(int32_T)(d_i + 1);
      c_i++;
    }
  }

  d_i = (int32_T)(50.0F - (real32_T)b_i);
  points_in_iss_sizes_idx_0 = (int32_T)(b_sizes + d_i);
  for (c_i = 0; c_i < 3; c_i++) {
    for (i = 0; i <= (int32_T)(b_sizes - 1); i++) {
      points_in_iss_data[(int32_T)(i + (int32_T)(points_in_iss_sizes_idx_0 * c_i))]
        = points_in_iss[(int32_T)((int32_T)((int32_T)(36 * c_i) + (int32_T)
        b_data[i]) - 1)];
    }
  }

  for (c_i = 0; c_i < 3; c_i++) {
    for (i = 0; i <= (int32_T)(d_i - 1); i++) {
      points_in_iss_data[(int32_T)((int32_T)(i + b_sizes) + (int32_T)
        (points_in_iss_sizes_idx_0 * c_i))] = 0.0F;
    }
  }

  for (c_i = 0; c_i < 3; c_i++) {
    memcpy(&points_out_iss[(int32_T)(c_i * 50)], &points_in_iss_data[(int32_T)
           (c_i * 50)], (uint32_T)(50U * sizeof(real32_T)));
  }

  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      c_i++;
    }
  }

  b_sizes = c_i;
  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      b_data[c_i] = (int8_T)(int32_T)(d_i + 1);
      c_i++;
    }
  }

  d_i = (int32_T)(50.0F - (real32_T)b_i);
  points_in_iss_sizes_idx_0 = (int32_T)(b_sizes + d_i);
  for (c_i = 0; c_i <= (int32_T)(b_sizes - 1); c_i++) {
    points_in_cam_data[c_i] = points_in_cam[(int32_T)((int32_T)b_data[c_i] - 1)];
  }

  for (c_i = 0; c_i <= (int32_T)(b_sizes - 1); c_i++) {
    points_in_cam_data[(int32_T)(c_i + points_in_iss_sizes_idx_0)] =
      points_in_cam[(int32_T)((int32_T)b_data[c_i] + 35)];
  }

  for (c_i = 0; c_i <= (int32_T)(d_i - 1); c_i++) {
    points_in_cam_data[(int32_T)(c_i + b_sizes)] = 0.0F;
  }

  for (c_i = 0; c_i <= (int32_T)(d_i - 1); c_i++) {
    points_in_cam_data[(int32_T)((int32_T)(c_i + b_sizes) +
      points_in_iss_sizes_idx_0)] = 0.0F;
  }

  for (c_i = 0; c_i < 2; c_i++) {
    memcpy(&points_out_cam[(int32_T)(c_i * 50)], &points_in_cam_data[(int32_T)
           (c_i * 50)], (uint32_T)(50U * sizeof(real32_T)));
  }

  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      c_i++;
    }
  }

  b_sizes = c_i;
  c_i = 0;
  for (d_i = 0; d_i < 36; d_i++) {
    if (valid_in[d_i]) {
      b_data[c_i] = (int8_T)(int32_T)(d_i + 1);
      c_i++;
    }
  }

  for (c_i = 0; c_i <= (int32_T)(b_sizes - 1); c_i++) {
    valid_in_data[c_i] = (int8_T)valid_in[(int32_T)((int32_T)b_data[c_i] - 1)];
  }

  b_i = (int32_T)(50.0F - (real32_T)b_i);
  for (c_i = 0; c_i <= (int32_T)(b_i - 1); c_i++) {
    valid_in_data[(int32_T)(c_i + b_sizes)] = 0;
  }

  for (c_i = 0; c_i < 50; c_i++) {
    valid_out[c_i] = (real32_T)valid_in_data[c_i];
  }
}

// Function for MATLAB Function: '<S81>/generate_output'
static void sim_model_lib0_rand(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  real_T b;
  int32_T k;
  r_sizes[0] = 1;
  r_sizes[1] = (int32_T)varargin_2;
  for (k = 0; k <= (int32_T)(r_sizes[1] - 1); k++) {
    b = aimgnophaiekjecb_eml_rand_mt19937ar(sim_model_lib0_DW->state_m);
    r_data[k] = b;
  }
}

// Function for MATLAB Function: '<S81>/generate_output'
static void sim_model_lib0_randperm(real_T n, real_T p_data[], int32_T p_sizes[2],
  DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  int32_T i;
  int32_T tmp_data[504];
  int32_T tmp_sizes[2];
  int32_T p_idx_1;
  int32_T p_idx_0;
  sim_model_lib0_rand(n, p_data, p_sizes, sim_model_lib0_DW);
  baaiimglmglniekf_sortIdx(p_data, p_sizes, tmp_data, tmp_sizes);
  p_idx_0 = p_sizes[0];
  p_idx_1 = p_sizes[1];
  p_sizes[0] = 1;
  for (i = 0; i <= (int32_T)(p_idx_1 - 1); i++) {
    p_data[i] = (real_T)tmp_data[(int32_T)(p_idx_0 * i)];
  }
}

// Function for MATLAB Function: '<S81>/generate_output'
static void sim_mode_format_handrail_output(const real32_T points_in_iss[1512],
  const real32_T points_in_cam[1512], const boolean_T valid_in[504], real32_T
  points_out_iss[150], real32_T points_out_cam[150], real32_T valid_out[50],
  B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  int32_T trueCount;
  int32_T i;
  real_T random_order_data[504];
  int32_T random_order_sizes[2];
  int16_T e_data[504];
  int32_T e_sizes;

  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  // format_handrail_output.m
  //  inputs:
  //    num_pts_out    = Required number of points to be reported
  //    points_in_iss  = ALL [x,y,z] position of points in the ISS frame
  //    points_in_cam  = ALL [u,v] pixel location of points in the camera
  //    valid_in       = indicates which points are valid
  //
  //  outputs:
  //    points_out_iss = output [x,y,z] position of points in the ISS frame      
  //    points_out_cam = output [x,y,z] pixel location of points in the camera frame 
  //    valid_out      = indicates which values of the output are valid!
  // initialize the outputs
  memset(&points_out_iss[0], 0, (uint32_T)(150U * sizeof(real32_T)));
  memset(&points_out_cam[0], 0, (uint32_T)(150U * sizeof(real32_T)));
  memset(&valid_out[0], 0, (uint32_T)(50U * sizeof(real32_T)));

  // find how many pts are valid, and determine how that compares to required number 
  trueCount = 0;
  for (i = 0; i < 504; i++) {
    if (valid_in[i]) {
      trueCount++;
    }
  }

  if (50.0F - (real32_T)trueCount >= 0.0F) {
    // if exactly the right ammount or need more, just report what we got! (rest will be zeros) 
    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        i++;
      }
    }

    e_sizes = i;
    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        e_data[i] = (int16_T)(int32_T)(trueCount + 1);
        i++;
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount <= (int32_T)(e_sizes - 1); trueCount++) {
        points_out_iss[(int32_T)(trueCount + (int32_T)(50 * i))] =
          points_in_iss[(int32_T)((int32_T)((int32_T)(504 * i) + (int32_T)
          e_data[trueCount]) - 1)];
      }
    }

    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        i++;
      }
    }

    e_sizes = i;
    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        e_data[i] = (int16_T)(int32_T)(trueCount + 1);
        i++;
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount <= (int32_T)(e_sizes - 1); trueCount++) {
        points_out_cam[(int32_T)(trueCount + (int32_T)(50 * i))] =
          points_in_cam[(int32_T)((int32_T)((int32_T)(504 * i) + (int32_T)
          e_data[trueCount]) - 1)];
      }
    }

    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        i++;
      }
    }

    e_sizes = i;
    i = 0;
    for (trueCount = 0; trueCount < 504; trueCount++) {
      if (valid_in[trueCount]) {
        e_data[i] = (int16_T)(int32_T)(trueCount + 1);
        i++;
      }
    }

    for (i = 0; i <= (int32_T)(e_sizes - 1); i++) {
      valid_out[i] = (real32_T)valid_in[(int32_T)((int32_T)e_data[i] - 1)];
    }
  } else {
    // if we have too many points, take a random sample
    trueCount = 0;
    for (i = 0; i < 504; i++) {
      if (valid_in[i]) {
        trueCount++;
      }
    }

    if (trueCount == 0) {
      trueCount = 0;
    } else {
      if (!(trueCount >= 3)) {
        trueCount = 3;
      }
    }

    sim_model_lib0_randperm((real_T)trueCount, random_order_data,
      random_order_sizes, sim_model_lib0_DW);
    trueCount = 0;
    for (i = 0; i < 504; i++) {
      if (valid_in[i]) {
        trueCount++;
      }
    }

    e_sizes = trueCount;
    trueCount = 0;
    for (i = 0; i < 504; i++) {
      if (valid_in[i]) {
        e_data[trueCount] = (int16_T)(int32_T)(i + 1);
        trueCount++;
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount <= (int32_T)(e_sizes - 1); trueCount++) {
        sim_model_lib0_B->points_in_iss_data[(int32_T)(trueCount + (int32_T)
          (e_sizes * i))] = points_in_iss[(int32_T)((int32_T)((int32_T)(504 * i)
          + (int32_T)e_data[trueCount]) - 1)];
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount < 50; trueCount++) {
        points_out_iss[(int32_T)(trueCount + (int32_T)(50 * i))] =
          sim_model_lib0_B->points_in_iss_data[(int32_T)((int32_T)((int32_T)
          (e_sizes * i) + (int32_T)random_order_data[trueCount]) - 1)];
      }
    }

    trueCount = 0;
    for (i = 0; i < 504; i++) {
      if (valid_in[i]) {
        trueCount++;
      }
    }

    e_sizes = trueCount;
    trueCount = 0;
    for (i = 0; i < 504; i++) {
      if (valid_in[i]) {
        e_data[trueCount] = (int16_T)(int32_T)(i + 1);
        trueCount++;
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount <= (int32_T)(e_sizes - 1); trueCount++) {
        sim_model_lib0_B->points_in_iss_data[(int32_T)(trueCount + (int32_T)
          (e_sizes * i))] = points_in_cam[(int32_T)((int32_T)((int32_T)(504 * i)
          + (int32_T)e_data[trueCount]) - 1)];
      }
    }

    for (i = 0; i < 3; i++) {
      for (trueCount = 0; trueCount < 50; trueCount++) {
        points_out_cam[(int32_T)(trueCount + (int32_T)(50 * i))] =
          sim_model_lib0_B->points_in_iss_data[(int32_T)((int32_T)((int32_T)
          (e_sizes * i) + (int32_T)random_order_data[trueCount]) - 1)];
      }
    }

    for (i = 0; i < 50; i++) {
      valid_out[i] = 1.0F;
    }
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim__pphdimglohlnoppp_mergesort(int32_T idx_data[], const real32_T
  x_data[], const int32_T x_sizes[2], const int32_T dir[2], int32_T n,
  B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (i = 1; i <= (int32_T)(n - 1); i += 2) {
    if (dbiekfcbglfkkfcb_sortLE(x_data, x_sizes, dir, i, (int32_T)(i + 1))) {
      idx_data[(int32_T)(i - 1)] = i;
      idx_data[i] = (int32_T)(i + 1);
    } else {
      idx_data[(int32_T)(i - 1)] = (int32_T)(i + 1);
      idx_data[i] = i;
    }
  }

  if ((int32_T)(n & 1) != 0) {
    idx_data[(int32_T)(n - 1)] = n;
  }

  i = 2;
  while (i < n) {
    i2 = (int32_T)(i << 1);
    j = 1;
    pEnd = (int32_T)(1 + i);
    while (pEnd < (int32_T)(n + 1)) {
      p = j;
      q = pEnd;
      qEnd = (int32_T)(j + i2);
      if (qEnd > (int32_T)(n + 1)) {
        qEnd = (int32_T)(n + 1);
      }

      k = 0;
      kEnd = (int32_T)(qEnd - j);
      while ((int32_T)(k + 1) <= kEnd) {
        if (dbiekfcbglfkkfcb_sortLE(x_data, x_sizes, dir, idx_data[(int32_T)(p -
              1)], idx_data[(int32_T)(q - 1)])) {
          sim_model_lib0_B->iwork_data_n[k] = idx_data[(int32_T)(p - 1)];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              sim_model_lib0_B->iwork_data_n[k] = idx_data[(int32_T)(q - 1)];
              q++;
            }
          }
        } else {
          sim_model_lib0_B->iwork_data_n[k] = idx_data[(int32_T)(q - 1)];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              sim_model_lib0_B->iwork_data_n[k] = idx_data[(int32_T)(p - 1)];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
        idx_data[(int32_T)((int32_T)(j + pEnd) - 1)] =
          sim_model_lib0_B->iwork_data_n[pEnd];
      }

      j = qEnd;
      pEnd = (int32_T)(qEnd + i);
    }

    i = i2;
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_mo_aaieppphglfcglng_sortIdx(const real32_T x_data[], const
  int32_T x_sizes[2], const int32_T col[2], int32_T idx_data[], int32_T
  *idx_sizes, B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T i;
  int32_T loop_ub;
  *idx_sizes = x_sizes[0];
  loop_ub = x_sizes[0];
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    idx_data[i] = 0;
  }

  sim__pphdimglohlnoppp_mergesort(idx_data, x_data, x_sizes, col, x_sizes[0],
    sim_model_lib0_B);
}

// Function for MATLAB Function: '<S83>/generate_output'
static void hdjmcjmophlfkngl_apply_row_perm(real32_T y_data[], int32_T y_sizes[2],
  const int32_T idx_data[], B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T m;
  int32_T i;
  m = y_sizes[0];
  for (i = 0; (int32_T)(i + 1) <= m; i++) {
    sim_model_lib0_B->ycol_data[i] = y_data[(int32_T)(idx_data[i] - 1)];
  }

  for (i = 0; (int32_T)(i + 1) <= m; i++) {
    y_data[i] = sim_model_lib0_B->ycol_data[i];
  }

  for (i = 0; (int32_T)(i + 1) <= m; i++) {
    sim_model_lib0_B->ycol_data[i] = y_data[(int32_T)((int32_T)(idx_data[i] +
      y_sizes[0]) - 1)];
  }

  for (i = 0; (int32_T)(i + 1) <= m; i++) {
    y_data[(int32_T)(i + y_sizes[0])] = sim_model_lib0_B->ycol_data[i];
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_m_mglflfcbkfkfbiec_sortrows(real32_T y_data[], int32_T y_sizes[2],
  real_T ndx_data[], int32_T *ndx_sizes, B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T m;
  int32_T col[2];
  int32_T idx_sizes;
  m = y_sizes[0];
  *ndx_sizes = y_sizes[0];
  col[0] = 1;
  col[1] = 2;
  sim_mo_aaieppphglfcglng_sortIdx(y_data, y_sizes, col,
    sim_model_lib0_B->idx_data_m, &idx_sizes, sim_model_lib0_B);
  hdjmcjmophlfkngl_apply_row_perm(y_data, y_sizes, sim_model_lib0_B->idx_data_m,
    sim_model_lib0_B);
  for (idx_sizes = 0; (int32_T)(idx_sizes + 1) <= m; idx_sizes++) {
    ndx_data[idx_sizes] = (real_T)sim_model_lib0_B->idx_data_m[idx_sizes];
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim__gdbacbaajekfglng_mergesort(int32_T idx_data[], const int32_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (i = 1; i <= (int32_T)(n - 1); i += 2) {
    if (x_data[(int32_T)(i - 1)] <= x_data[i]) {
      idx_data[(int32_T)(i - 1)] = i;
      idx_data[i] = (int32_T)(i + 1);
    } else {
      idx_data[(int32_T)(i - 1)] = (int32_T)(i + 1);
      idx_data[i] = i;
    }
  }

  if ((int32_T)(n & 1) != 0) {
    idx_data[(int32_T)(n - 1)] = n;
  }

  i = 2;
  while (i < n) {
    i2 = (int32_T)(i << 1);
    j = 1;
    pEnd = (int32_T)(1 + i);
    while (pEnd < (int32_T)(n + 1)) {
      p = j;
      q = pEnd;
      qEnd = (int32_T)(j + i2);
      if (qEnd > (int32_T)(n + 1)) {
        qEnd = (int32_T)(n + 1);
      }

      k = 0;
      kEnd = (int32_T)(qEnd - j);
      while ((int32_T)(k + 1) <= kEnd) {
        if (x_data[(int32_T)(idx_data[(int32_T)(p - 1)] - 1)] <= x_data[(int32_T)
            (idx_data[(int32_T)(q - 1)] - 1)]) {
          sim_model_lib0_B->iwork_data_p[k] = idx_data[(int32_T)(p - 1)];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              sim_model_lib0_B->iwork_data_p[k] = idx_data[(int32_T)(q - 1)];
              q++;
            }
          }
        } else {
          sim_model_lib0_B->iwork_data_p[k] = idx_data[(int32_T)(q - 1)];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              sim_model_lib0_B->iwork_data_p[k] = idx_data[(int32_T)(p - 1)];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
        idx_data[(int32_T)((int32_T)(j + pEnd) - 1)] =
          sim_model_lib0_B->iwork_data_p[pEnd];
      }

      j = qEnd;
      pEnd = (int32_T)(qEnd + i);
    }

    i = i2;
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_mo_dbaaphlnfkfkphdb_sortIdx(const int32_T x_data[], const
  int32_T x_sizes, int32_T idx_data[], int32_T *idx_sizes, B_sim_model_lib0_T
  *sim_model_lib0_B)
{
  int32_T i;
  int32_T loop_ub;
  int16_T b_x_idx_0;
  b_x_idx_0 = (int16_T)x_sizes;
  *idx_sizes = (int32_T)b_x_idx_0;
  loop_ub = (int32_T)b_x_idx_0;
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    idx_data[i] = 0;
  }

  if (x_sizes != 0) {
    sim__gdbacbaajekfglng_mergesort(idx_data, x_data, x_sizes, sim_model_lib0_B);
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_fkfchlfkgdbimgdj_do_vectors(const real32_T a[50], const real32_T
  b_data[], const int32_T b_sizes, real32_T c_data[], int32_T *c_sizes, int32_T
  ia_data[], int32_T *ia_sizes, int32_T ib_data[], int32_T *ib_sizes,
  B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T nc;
  int32_T nib;
  int32_T iafirst;
  int32_T ialast;
  int32_T ibfirst;
  int32_T iblast;
  real32_T bk;
  int32_T c_ialast;
  int32_T c_iblast;
  real32_T b_bk;
  boolean_T p;
  real32_T absxk;
  int32_T exponent;
  int32_T ia_data_0[50];
  *ib_sizes = b_sizes;
  nc = -1;
  *ia_sizes = 0;
  nib = 0;
  iafirst = 0;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= 50) && (iblast <= b_sizes)) {
    c_ialast = ialast;
    bk = kngdbaaaiecjbiec_skip_to_last_equal_value(&c_ialast, a);
    ialast = c_ialast;
    c_iblast = iblast;
    b_bk = mgdjimgdopphcjek_skip_to_last_equal_value(&c_iblast, b_data, b_sizes);
    iblast = c_iblast;
    absxk = (real32_T)fabs((real_T)(b_bk / 2.0F));
    if ((!rtIsInfF(absxk)) && (!rtIsNaNF(absxk))) {
      if (absxk <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        frexp((real_T)absxk, &exponent);
        absxk = (real32_T)ldexp((real_T)1.0F, (int32_T)(exponent - 24));
      }
    } else {
      absxk = (rtNaNF);
    }

    if (((real32_T)fabs((real_T)(b_bk - bk)) < absxk) || (rtIsInfF(bk) &&
         rtIsInfF(b_bk) && ((bk > 0.0F) == (b_bk > 0.0F)))) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      ialast = (int32_T)(c_ialast + 1);
      iafirst = c_ialast;
      iblast = (int32_T)(c_iblast + 1);
      ibfirst = c_iblast;
    } else {
      if ((bk < b_bk) || rtIsNaNF(b_bk)) {
        p = true;
      }

      if (p) {
        nc++;
        (*ia_sizes)++;
        c_data[nc] = bk;
        ia_data[(int32_T)(*ia_sizes - 1)] = (int32_T)(iafirst + 1);
        ialast = (int32_T)(c_ialast + 1);
        iafirst = c_ialast;
      } else {
        nc++;
        nib++;
        c_data[nc] = b_bk;
        ib_data[(int32_T)(nib - 1)] = (int32_T)(ibfirst + 1);
        iblast = (int32_T)(c_iblast + 1);
        ibfirst = c_iblast;
      }
    }
  }

  while (ialast <= 50) {
    iafirst = ialast;
    bk = kngdbaaaiecjbiec_skip_to_last_equal_value(&iafirst, a);
    nc++;
    (*ia_sizes)++;
    c_data[nc] = bk;
    ia_data[(int32_T)(*ia_sizes - 1)] = ialast;
    ialast = (int32_T)(iafirst + 1);
  }

  while (iblast <= b_sizes) {
    ialast = iblast;
    bk = mgdjimgdopphcjek_skip_to_last_equal_value(&ialast, b_data, b_sizes);
    nc++;
    nib++;
    c_data[nc] = bk;
    ib_data[(int32_T)(nib - 1)] = iblast;
    iblast = (int32_T)(ialast + 1);
  }

  if (1 > *ia_sizes) {
    *ia_sizes = 0;
  }

  for (iafirst = 0; iafirst <= (int32_T)(*ia_sizes - 1); iafirst++) {
    ia_data_0[iafirst] = ia_data[iafirst];
  }

  for (iafirst = 0; iafirst <= (int32_T)(*ia_sizes - 1); iafirst++) {
    ia_data[iafirst] = ia_data_0[iafirst];
  }

  if (b_sizes > 0) {
    if (1 > nib) {
      nib = 0;
    }

    for (iafirst = 0; iafirst <= (int32_T)(nib - 1); iafirst++) {
      sim_model_lib0_B->ib_data[iafirst] = ib_data[iafirst];
    }

    *ib_sizes = nib;
    for (iafirst = 0; iafirst <= (int32_T)(nib - 1); iafirst++) {
      ib_data[iafirst] = sim_model_lib0_B->ib_data[iafirst];
    }
  }

  if (1 > (int32_T)(nc + 1)) {
    *c_sizes = 0;
  } else {
    *c_sizes = (int32_T)(nc + 1);
  }

  for (iafirst = 0; iafirst <= (int32_T)(*c_sizes - 1); iafirst++) {
    sim_model_lib0_B->c_data[iafirst] = c_data[iafirst];
  }

  for (iafirst = 0; iafirst <= (int32_T)(*c_sizes - 1); iafirst++) {
    c_data[iafirst] = sim_model_lib0_B->c_data[iafirst];
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_model_lib0_rand_n(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  real_T b;
  int32_T k;
  r_sizes[0] = 1;
  r_sizes[1] = (int32_T)varargin_2;
  for (k = 0; k <= (int32_T)(r_sizes[1] - 1); k++) {
    b = aimgnophaiekjecb_eml_rand_mt19937ar(sim_model_lib0_DW->state);
    r_data[k] = b;
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim__aiekglfcjekfdbai_mergesort(int32_T idx_data[], const real_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (i = 1; i <= (int32_T)(n - 1); i += 2) {
    if (x_data[(int32_T)(i - 1)] <= x_data[i]) {
      idx_data[(int32_T)(i - 1)] = i;
      idx_data[i] = (int32_T)(i + 1);
    } else {
      idx_data[(int32_T)(i - 1)] = (int32_T)(i + 1);
      idx_data[i] = i;
    }
  }

  if ((int32_T)(n & 1) != 0) {
    idx_data[(int32_T)(n - 1)] = n;
  }

  i = 2;
  while (i < n) {
    i2 = (int32_T)(i << 1);
    j = 1;
    pEnd = (int32_T)(1 + i);
    while (pEnd < (int32_T)(n + 1)) {
      p = j;
      q = pEnd;
      qEnd = (int32_T)(j + i2);
      if (qEnd > (int32_T)(n + 1)) {
        qEnd = (int32_T)(n + 1);
      }

      k = 0;
      kEnd = (int32_T)(qEnd - j);
      while ((int32_T)(k + 1) <= kEnd) {
        if (x_data[(int32_T)(idx_data[(int32_T)(p - 1)] - 1)] <= x_data[(int32_T)
            (idx_data[(int32_T)(q - 1)] - 1)]) {
          sim_model_lib0_B->iwork_data_l[k] = idx_data[(int32_T)(p - 1)];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              sim_model_lib0_B->iwork_data_l[k] = idx_data[(int32_T)(q - 1)];
              q++;
            }
          }
        } else {
          sim_model_lib0_B->iwork_data_l[k] = idx_data[(int32_T)(q - 1)];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              sim_model_lib0_B->iwork_data_l[k] = idx_data[(int32_T)(p - 1)];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
        idx_data[(int32_T)((int32_T)(j + pEnd) - 1)] =
          sim_model_lib0_B->iwork_data_l[pEnd];
      }

      j = qEnd;
      pEnd = (int32_T)(qEnd + i);
    }

    i = i2;
  }
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_mo_cbimppphbaaaaiec_sortIdx(const real_T x_data[], const int32_T
  x_sizes[2], int32_T idx_data[], int32_T idx_sizes[2], B_sim_model_lib0_T
  *sim_model_lib0_B)
{
  int32_T i;
  int32_T loop_ub;
  idx_sizes[0] = 1;
  idx_sizes[1] = x_sizes[1];
  loop_ub = x_sizes[1];
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    idx_data[i] = 0;
  }

  sim__aiekglfcjekfdbai_mergesort(idx_data, x_data, x_sizes[1], sim_model_lib0_B);
}

// Function for MATLAB Function: '<S83>/generate_output'
static void sim_model_lib0_randperm_i(real_T n, real_T p_data[], int32_T
  p_sizes[2], B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T
  *sim_model_lib0_DW)
{
  int32_T i;
  int32_T idx_sizes[2];
  int32_T p_idx_1;
  int32_T p_idx_0;
  sim_model_lib0_rand_n(n, p_data, p_sizes, sim_model_lib0_DW);
  sim_mo_cbimppphbaaaaiec_sortIdx(p_data, p_sizes, sim_model_lib0_B->idx_data_g,
    idx_sizes, sim_model_lib0_B);
  p_idx_0 = p_sizes[0];
  p_idx_1 = p_sizes[1];
  p_sizes[0] = 1;
  for (i = 0; i <= (int32_T)(p_idx_1 - 1); i++) {
    p_data[i] = (real_T)sim_model_lib0_B->idx_data_g[(int32_T)(p_idx_0 * i)];
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_mo_lfcjjmgdkfkniekn_sortIdx(const real32_T x[17264], const
  int32_T col[2], int32_T idx[8632], B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (i = 0; i <= 8631; i += 2) {
    if (mohdcjmgnohlknoh_sortLE(x, col, (int32_T)(i + 1), (int32_T)(i + 2))) {
      idx[i] = (int32_T)(i + 1);
      idx[(int32_T)(i + 1)] = (int32_T)(i + 2);
    } else {
      idx[i] = (int32_T)(i + 2);
      idx[(int32_T)(i + 1)] = (int32_T)(i + 1);
    }
  }

  i = 2;
  while (i < 8632) {
    i2 = (int32_T)(i << 1);
    j = 1;
    pEnd = (int32_T)(1 + i);
    while (pEnd < 8633) {
      p = j;
      q = pEnd;
      qEnd = (int32_T)(j + i2);
      if (qEnd > 8633) {
        qEnd = 8633;
      }

      k = 0;
      kEnd = (int32_T)(qEnd - j);
      while ((int32_T)(k + 1) <= kEnd) {
        if (mohdcjmgnohlknoh_sortLE(x, col, idx[(int32_T)(p - 1)], idx[(int32_T)
             (q - 1)])) {
          sim_model_lib0_B->iwork[k] = idx[(int32_T)(p - 1)];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              sim_model_lib0_B->iwork[k] = idx[(int32_T)(q - 1)];
              q++;
            }
          }
        } else {
          sim_model_lib0_B->iwork[k] = idx[(int32_T)(q - 1)];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              sim_model_lib0_B->iwork[k] = idx[(int32_T)(p - 1)];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
        idx[(int32_T)((int32_T)(j + pEnd) - 1)] = sim_model_lib0_B->iwork[pEnd];
      }

      j = qEnd;
      pEnd = (int32_T)(qEnd + i);
    }

    i = i2;
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void imglgdjmcjmomgdj_apply_row_perm(real32_T y[17264], const int32_T
  idx[8632], B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T j;
  int32_T i;
  for (j = 0; j < 2; j++) {
    for (i = 0; i < 8632; i++) {
      sim_model_lib0_B->ycol[i] = y[(int32_T)((int32_T)((int32_T)(8632 * j) +
        idx[i]) - 1)];
    }

    memcpy(&y[(int32_T)(j * 8632)], &sim_model_lib0_B->ycol[0], (uint32_T)(8632U
            * sizeof(real32_T)));
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_m_nopphdbaaiecfkng_sortrows(real32_T y[17264], real_T ndx[8632],
  B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T col[2];
  int32_T k;
  col[0] = 1;
  col[1] = 2;
  sim_mo_lfcjjmgdkfkniekn_sortIdx(y, col, sim_model_lib0_B->idx_p,
    sim_model_lib0_B);
  imglgdjmcjmomgdj_apply_row_perm(y, sim_model_lib0_B->idx_p, sim_model_lib0_B);
  for (k = 0; k < 8632; k++) {
    ndx[k] = (real_T)sim_model_lib0_B->idx_p[k];
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_model_lib0_rand_o(real_T varargin_2, real_T r_data[], int32_T
  r_sizes[2], DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  real_T b;
  int32_T k;
  r_sizes[0] = 1;
  r_sizes[1] = (int32_T)varargin_2;
  for (k = 0; k <= (int32_T)(r_sizes[1] - 1); k++) {
    b = aimgnophaiekjecb_eml_rand_mt19937ar(sim_model_lib0_DW->state_d);
    r_data[k] = b;
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim__kfkflfcjohdbglno_mergesort(int32_T idx_data[], const real_T
  x_data[], int32_T n, B_sim_model_lib0_T *sim_model_lib0_B)
{
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (i = 1; i <= (int32_T)(n - 1); i += 2) {
    if (x_data[(int32_T)(i - 1)] <= x_data[i]) {
      idx_data[(int32_T)(i - 1)] = i;
      idx_data[i] = (int32_T)(i + 1);
    } else {
      idx_data[(int32_T)(i - 1)] = (int32_T)(i + 1);
      idx_data[i] = i;
    }
  }

  if ((int32_T)(n & 1) != 0) {
    idx_data[(int32_T)(n - 1)] = n;
  }

  i = 2;
  while (i < n) {
    i2 = (int32_T)(i << 1);
    j = 1;
    pEnd = (int32_T)(1 + i);
    while (pEnd < (int32_T)(n + 1)) {
      p = j;
      q = pEnd;
      qEnd = (int32_T)(j + i2);
      if (qEnd > (int32_T)(n + 1)) {
        qEnd = (int32_T)(n + 1);
      }

      k = 0;
      kEnd = (int32_T)(qEnd - j);
      while ((int32_T)(k + 1) <= kEnd) {
        if (x_data[(int32_T)(idx_data[(int32_T)(p - 1)] - 1)] <= x_data[(int32_T)
            (idx_data[(int32_T)(q - 1)] - 1)]) {
          sim_model_lib0_B->iwork_data[k] = idx_data[(int32_T)(p - 1)];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              sim_model_lib0_B->iwork_data[k] = idx_data[(int32_T)(q - 1)];
              q++;
            }
          }
        } else {
          sim_model_lib0_B->iwork_data[k] = idx_data[(int32_T)(q - 1)];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              sim_model_lib0_B->iwork_data[k] = idx_data[(int32_T)(p - 1)];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
        idx_data[(int32_T)((int32_T)(j + pEnd) - 1)] =
          sim_model_lib0_B->iwork_data[pEnd];
      }

      j = qEnd;
      pEnd = (int32_T)(qEnd + i);
    }

    i = i2;
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_mo_bimgaiekaimglfkf_sortIdx(const real_T x_data[], const int32_T
  x_sizes[2], int32_T idx_data[], int32_T idx_sizes[2], B_sim_model_lib0_T
  *sim_model_lib0_B)
{
  int32_T i;
  int32_T loop_ub;
  idx_sizes[0] = 1;
  idx_sizes[1] = x_sizes[1];
  loop_ub = x_sizes[1];
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    idx_data[i] = 0;
  }

  if (x_sizes[1] != 0) {
    sim__kfkflfcjohdbglno_mergesort(idx_data, x_data, x_sizes[1],
      sim_model_lib0_B);
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_model_lib0_randperm_e(real_T n, real_T p_data[], int32_T
  p_sizes[2], B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T
  *sim_model_lib0_DW)
{
  int32_T i;
  int32_T idx_sizes[2];
  int32_T p_idx_1;
  int32_T p_idx_0;
  sim_model_lib0_rand_o(n, p_data, p_sizes, sim_model_lib0_DW);
  sim_mo_bimgaiekaimglfkf_sortIdx(p_data, p_sizes, sim_model_lib0_B->idx_data,
    idx_sizes, sim_model_lib0_B);
  p_idx_0 = p_sizes[0];
  p_idx_1 = p_sizes[1];
  p_sizes[0] = 1;
  for (i = 0; i <= (int32_T)(p_idx_1 - 1); i++) {
    p_data[i] = (real_T)sim_model_lib0_B->idx_data[(int32_T)(p_idx_0 * i)];
  }
}

// Function for MATLAB Function: '<S82>/generate_output'
static void sim_mode_format_landmark_output(const real32_T points_in_iss[25896],
  const real32_T points_in_cam[17264], const boolean_T valid_in[8632], real32_T
  points_out_iss[150], real32_T points_out_cam[100], real32_T valid_out[50],
  B_sim_model_lib0_T *sim_model_lib0_B, DW_sim_model_lib0_T *sim_model_lib0_DW)
{
  int32_T nb;
  int32_T k;
  int32_T k0;
  int32_T loop_ub;
  int32_T i;
  int32_T random_order_sizes[2];
  int32_T unique_rows_sizes;
  int32_T c_valid_in_sizes;
  int32_T b_points_in_iss_sizes_idx_0;
  int32_T b_points_in_iss_sizes_idx_0_0;

  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  // format_landmark_output.m
  //  inputs:
  //    num_pts_out    = Required number of points to be reported
  //    points_in iss  = ALL [x,y,z] position of points in the ISS frame
  //    points_in_cam  = ALL [u,v] pixel location of points in the camera
  //    valid_in       = indicates which points are valid
  //
  //  outputs:
  //    points_out_iss = output [x,y,z] position of points in the ISS frame      
  //    points_out_cam = output [u,v] pixel location of points in the camera
  //    valid_out      = indicates which values of the output are valid!
  // initialize the outputs
  // Find unique 2D points in the cam can be thought of as either:
  //    a) points blocked by other points are not visible to the camera, despite technically being in the FOV of the camera 
  //    b) Only can have 1 point reported per pixel location
  memcpy(&sim_model_lib0_B->b_b[0], &points_in_cam[0], (uint32_T)(17264U *
          sizeof(real32_T)));
  sim_m_nopphdbaaiecfkng_sortrows(sim_model_lib0_B->b_b, sim_model_lib0_B->idx,
    sim_model_lib0_B);
  random_order_sizes[0] = 8632;
  random_order_sizes[1] = 2;
  nb = 0;
  k = 1;
  while (k <= 8632) {
    k0 = k;
    do {
      k++;
    } while (!((k > 8632) || ohdbiekfdjechlno_rows_differ(sim_model_lib0_B->b_b,
               random_order_sizes, k0, k)));

    nb++;
    sim_model_lib0_B->b_b[(int32_T)(nb - 1)] = sim_model_lib0_B->b_b[(int32_T)
      (k0 - 1)];
    sim_model_lib0_B->b_b[(int32_T)(nb + 8631)] = sim_model_lib0_B->b_b[(int32_T)
      (k0 + 8631)];
    sim_model_lib0_B->idx[(int32_T)(nb - 1)] = sim_model_lib0_B->idx[(int32_T)
      (k0 - 1)];
  }

  if (1 > nb) {
    loop_ub = 0;
  } else {
    loop_ub = nb;
  }

  for (k = 0; k <= (int32_T)(loop_ub - 1); k++) {
    sim_model_lib0_B->b_data_c[k] = sim_model_lib0_B->b_b[k];
  }

  for (k = 0; k <= (int32_T)(loop_ub - 1); k++) {
    sim_model_lib0_B->b_data_c[(int32_T)(k + loop_ub)] = sim_model_lib0_B->b_b
      [(int32_T)(k + 8632)];
  }

  for (k = 0; k <= (int32_T)(loop_ub - 1); k++) {
    sim_model_lib0_B->b_b[k] = sim_model_lib0_B->b_data_c[k];
  }

  for (k = 0; k <= (int32_T)(loop_ub - 1); k++) {
    sim_model_lib0_B->b_b[(int32_T)(k + loop_ub)] = sim_model_lib0_B->b_data_c
      [(int32_T)(k + loop_ub)];
  }

  for (k = 0; (int32_T)(k + 1) <= nb; k++) {
    sim_model_lib0_B->indx_data[k] = (int32_T)sim_model_lib0_B->idx[k];
  }

  unique_rows_sizes = (int32_T)(nb - 1);
  for (k = 0; k <= (int32_T)(nb - 1); k++) {
    sim_model_lib0_B->unique_rows_data[k] = sim_model_lib0_B->indx_data[k];
  }

  b_points_in_iss_sizes_idx_0 = nb;
  for (k = 0; k < 3; k++) {
    for (i = 0; i <= (int32_T)(nb - 1); i++) {
      sim_model_lib0_B->b_points_in_iss_data_c[(int32_T)(i + (int32_T)(nb * k))]
        = points_in_iss[(int32_T)((int32_T)((int32_T)(8632 * k) +
        sim_model_lib0_B->indx_data[i]) - 1)];
    }
  }

  for (k = 0; k <= (int32_T)(nb - 1); k++) {
    sim_model_lib0_B->b_valid_in_data_j[k] = valid_in[(int32_T)
      (sim_model_lib0_B->indx_data[k] - 1)];
  }

  // find how many pts are valid, and determine how that compares to required number 
  c_valid_in_sizes = (int32_T)(nb - 1);
  nb = 0;
  for (k = 0; k <= c_valid_in_sizes; k++) {
    if (valid_in[(int32_T)(sim_model_lib0_B->indx_data[k] - 1)]) {
      nb++;
    }
  }

  if (50.0F - (real32_T)nb == 0.0F) {
    // if exactly the right ammount, just report them!
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        k++;
      }
    }

    c_valid_in_sizes = k;
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        sim_model_lib0_B->indx_data[k] = (int32_T)(k0 + 1);
        k++;
      }
    }

    for (k = 0; k < 3; k++) {
      for (i = 0; i <= (int32_T)(c_valid_in_sizes - 1); i++) {
        sim_model_lib0_B->b_points_in_iss_data_k[(int32_T)(i + (int32_T)
          (c_valid_in_sizes * k))] = sim_model_lib0_B->b_points_in_iss_data_c
          [(int32_T)((int32_T)((int32_T)(b_points_in_iss_sizes_idx_0 * k) +
          sim_model_lib0_B->indx_data[i]) - 1)];
      }
    }

    for (k = 0; k < 3; k++) {
      memcpy(&points_out_iss[(int32_T)(k * 50)],
             &sim_model_lib0_B->b_points_in_iss_data_k[(int32_T)(k * 50)],
             (uint32_T)(50U * sizeof(real32_T)));
    }

    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        k++;
      }
    }

    c_valid_in_sizes = k;
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        sim_model_lib0_B->indx_data[k] = (int32_T)(k0 + 1);
        k++;
      }
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data_c[k] = sim_model_lib0_B->b_b[(int32_T)
        (sim_model_lib0_B->indx_data[k] - 1)];
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data_c[(int32_T)(k + c_valid_in_sizes)] =
        sim_model_lib0_B->b_b[(int32_T)((int32_T)(sim_model_lib0_B->indx_data[k]
        + loop_ub) - 1)];
    }

    for (k = 0; k < 2; k++) {
      memcpy(&points_out_cam[(int32_T)(k * 50)], &sim_model_lib0_B->b_data_c
             [(int32_T)(k * 50)], (uint32_T)(50U * sizeof(real32_T)));
    }

    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        sim_model_lib0_B->indx_data[k] = (int32_T)(k0 + 1);
        k++;
      }
    }

    for (k = 0; k < 50; k++) {
      valid_out[k] = (real32_T)sim_model_lib0_B->b_valid_in_data_j[(int32_T)
        (sim_model_lib0_B->indx_data[k] - 1)];
    }
  } else if (50.0F - (real32_T)nb > 0.0F) {
    // if we need more points, back fill with zeros, and zero the valid cooresponding valid flag 
    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        k0++;
      }
    }

    c_valid_in_sizes = k0;
    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        sim_model_lib0_B->indx_data[k0] = (int32_T)(k + 1);
        k0++;
      }
    }

    k0 = (int32_T)(50.0F - (real32_T)nb);
    b_points_in_iss_sizes_idx_0_0 = (int32_T)(c_valid_in_sizes + k0);
    for (k = 0; k < 3; k++) {
      for (i = 0; i <= (int32_T)(c_valid_in_sizes - 1); i++) {
        sim_model_lib0_B->b_points_in_iss_data[(int32_T)(i + (int32_T)
          (b_points_in_iss_sizes_idx_0_0 * k))] =
          sim_model_lib0_B->b_points_in_iss_data_c[(int32_T)((int32_T)((int32_T)
          (b_points_in_iss_sizes_idx_0 * k) + sim_model_lib0_B->indx_data[i]) -
          1)];
      }
    }

    for (k = 0; k < 3; k++) {
      for (i = 0; i <= (int32_T)(k0 - 1); i++) {
        sim_model_lib0_B->b_points_in_iss_data[(int32_T)((int32_T)(i +
          c_valid_in_sizes) + (int32_T)(b_points_in_iss_sizes_idx_0_0 * k))] =
          0.0F;
      }
    }

    for (k = 0; k < 3; k++) {
      memcpy(&points_out_iss[(int32_T)(k * 50)],
             &sim_model_lib0_B->b_points_in_iss_data[(int32_T)(k * 50)],
             (uint32_T)(50U * sizeof(real32_T)));
    }

    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        k0++;
      }
    }

    c_valid_in_sizes = k0;
    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        sim_model_lib0_B->indx_data[k0] = (int32_T)(k + 1);
        k0++;
      }
    }

    k0 = (int32_T)(50.0F - (real32_T)nb);
    b_points_in_iss_sizes_idx_0 = (int32_T)(c_valid_in_sizes + k0);
    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data[k] = sim_model_lib0_B->b_b[(int32_T)
        (sim_model_lib0_B->indx_data[k] - 1)];
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data[(int32_T)(k + b_points_in_iss_sizes_idx_0)] =
        sim_model_lib0_B->b_b[(int32_T)((int32_T)(sim_model_lib0_B->indx_data[k]
        + loop_ub) - 1)];
    }

    for (k = 0; k <= (int32_T)(k0 - 1); k++) {
      sim_model_lib0_B->b_data[(int32_T)(k + c_valid_in_sizes)] = 0.0F;
    }

    for (k = 0; k <= (int32_T)(k0 - 1); k++) {
      sim_model_lib0_B->b_data[(int32_T)((int32_T)(k + c_valid_in_sizes) +
        b_points_in_iss_sizes_idx_0)] = 0.0F;
    }

    for (k = 0; k < 2; k++) {
      memcpy(&points_out_cam[(int32_T)(k * 50)], &sim_model_lib0_B->b_data
             [(int32_T)(k * 50)], (uint32_T)(50U * sizeof(real32_T)));
    }

    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        k0++;
      }
    }

    c_valid_in_sizes = k0;
    k0 = 0;
    for (k = 0; k <= unique_rows_sizes; k++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k] - 1)]) {
        sim_model_lib0_B->indx_data[k0] = (int32_T)(k + 1);
        k0++;
      }
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_valid_in_data[k] = (int8_T)
        sim_model_lib0_B->b_valid_in_data_j[(int32_T)
        (sim_model_lib0_B->indx_data[k] - 1)];
    }

    loop_ub = (int32_T)(50.0F - (real32_T)nb);
    for (k = 0; k <= (int32_T)(loop_ub - 1); k++) {
      sim_model_lib0_B->b_valid_in_data[(int32_T)(k + c_valid_in_sizes)] = 0;
    }

    for (k = 0; k < 50; k++) {
      valid_out[k] = (real32_T)sim_model_lib0_B->b_valid_in_data[k];
    }
  } else {
    // if we have too many points, take a random sample
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        k++;
      }
    }

    if (k == 0) {
      k = 0;
    } else {
      if (!(k >= 3)) {
        k = 3;
      }
    }

    sim_model_lib0_randperm_e((real_T)k, sim_model_lib0_B->idx,
      random_order_sizes, sim_model_lib0_B, sim_model_lib0_DW);
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        k++;
      }
    }

    c_valid_in_sizes = k;
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        sim_model_lib0_B->indx_data[k] = (int32_T)(k0 + 1);
        k++;
      }
    }

    for (k = 0; k < 3; k++) {
      for (i = 0; i <= (int32_T)(c_valid_in_sizes - 1); i++) {
        sim_model_lib0_B->b_points_in_iss_data_k[(int32_T)(i + (int32_T)
          (c_valid_in_sizes * k))] = sim_model_lib0_B->b_points_in_iss_data_c
          [(int32_T)((int32_T)((int32_T)(b_points_in_iss_sizes_idx_0 * k) +
          sim_model_lib0_B->indx_data[i]) - 1)];
      }
    }

    for (k = 0; k < 3; k++) {
      for (i = 0; i < 50; i++) {
        points_out_iss[(int32_T)(i + (int32_T)(50 * k))] =
          sim_model_lib0_B->b_points_in_iss_data_k[(int32_T)((int32_T)((int32_T)
          (c_valid_in_sizes * k) + (int32_T)sim_model_lib0_B->idx[i]) - 1)];
      }
    }

    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        k++;
      }
    }

    c_valid_in_sizes = k;
    k = 0;
    for (k0 = 0; k0 <= unique_rows_sizes; k0++) {
      if (valid_in[(int32_T)(sim_model_lib0_B->unique_rows_data[k0] - 1)]) {
        sim_model_lib0_B->indx_data[k] = (int32_T)(k0 + 1);
        k++;
      }
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data_c[k] = sim_model_lib0_B->b_b[(int32_T)
        (sim_model_lib0_B->indx_data[k] - 1)];
    }

    for (k = 0; k <= (int32_T)(c_valid_in_sizes - 1); k++) {
      sim_model_lib0_B->b_data_c[(int32_T)(k + c_valid_in_sizes)] =
        sim_model_lib0_B->b_b[(int32_T)((int32_T)(sim_model_lib0_B->indx_data[k]
        + loop_ub) - 1)];
    }

    for (k = 0; k < 2; k++) {
      for (i = 0; i < 50; i++) {
        points_out_cam[(int32_T)(i + (int32_T)(50 * k))] =
          sim_model_lib0_B->b_data_c[(int32_T)((int32_T)((int32_T)
          (c_valid_in_sizes * k) + (int32_T)sim_model_lib0_B->idx[i]) - 1)];
      }
    }

    for (nb = 0; nb < 50; nb++) {
      valid_out[nb] = 1.0F;
    }
  }
}

// Model step function for TID0
void sim_model_lib0_step0(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M,
  act_msg *sim_model_lib0_U_act_msg_l, cmc_msg *sim_model_lib0_U_cmc_msg_in,
  cvs_optical_flow_msg *sim_model_lib0_Y_cvs_optical_flow_msg_n,
  cvs_handrail_msg *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg
  *sim_model_lib0_Y_cmc_msg_c, imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg
  *sim_model_lib0_Y_env_msg_i, bpm_msg *sim_model_lib0_Y_bpm_msg_h,
  cvs_registration_pulse *sim_model_lib0_Y_cvs_registration_pulse_d,
  cvs_landmark_msg *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
  *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg *sim_model_lib0_Y_ex_time_msg_m)
  // Sample time: [0.016s, 0.0s]
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);
  real32_T rtb_P_B_ISS_SS[3];
  real32_T rtb_Switch_al[19];
  real32_T rtb_V_B_ISS_ISS[3];
  real32_T rtb_A_B_ISS_ISS[3];
  real32_T rtb_alpha_B_ECI_B[3];
  uint8_T rtb_Bias;
  real32_T rtb_imu_gyro_bias[3];
  real32_T rtb_bpm_force_B[3];
  real32_T rtb_bpm_torque_B[3];
  real_T rtb_Product_l[4];
  real_T rtb_Assignment_jy[9];
  real_T rtb_MatrixConcatenate[16];
  real_T rtb_Assignment[9];
  real_T rtb_Sum;
  real32_T rtb_Sqrt_m;
  real32_T rtb_Product_e[4];
  real32_T rtb_Merge_kq[4];
  real32_T rtb_Merge_m[4];
  real32_T rtb_Assignment_m[9];
  real32_T rtb_Elementproduct_c[6];
  uint32_T rtb_Switch1;
  uint32_T rtb_Switch_g;
  real32_T rtb_bpm_servo_curr[12];
  real32_T rtb_bpm_nozzle_theta[12];
  real32_T rtb_VectorConcatenate_d[16];
  real32_T rtb_Product_p;
  real32_T rtb_TrigonometricFunction1_c;
  real32_T rtb_Assignment_ok[16];
  real32_T rtb_Delay1_bpm_torque_B[3];
  real32_T rtb_Merge_o[3];
  real32_T rtb_Sum2_f[3];
  real_T rtb_Add3_a[3];
  real_T rtb_Add3[3];
  real_T rtb_random_noise[3];
  real_T rtb_linearaccelbias;
  real_T rtb_P_sensor_CG_B[3];
  cmc_msg rtb_BusAssignment;
  cmc_msg rtb_BusAssignment1;
  boolean_T rtb_LogicalOperator2[12];
  int32_T i;
  real32_T tmp[3];
  int32_T i_0;
  boolean_T tmp_0;
  real_T rtb_Product_p_0[9];
  real32_T tmp_1[9];
  real32_T rtb_Merge_b[9];
  real32_T rtb_Assignment_i[9];
  real32_T tmp_2[9];
  real32_T tmp_3[9];
  real32_T tmp_4[9];
  real32_T tmp_5[9];
  real_T rtb_Assignment_3[9];
  real_T tmp_6[9];
  real_T tmp_7[9];
  real_T rtb_Assignment_c[9];
  real32_T tmp_8[9];
  real32_T tmp_9[9];
  real32_T tmp_a[3];
  real32_T tmp_b[3];
  real32_T rtb_bpm_torque_B_c;
  real32_T rtb_Merge_c;
  real_T rtb_Product;
  real32_T rtb_bpm_motor_curr_idx_0;
  uint32_T rtb_Switch1_p_idx_0;
  uint32_T rtb_Switch1_p_idx_1;
  real_T rtb_DataTypeConversion5_idx_1;
  real_T rtb_DataTypeConversion5_idx_2;
  real_T rtb_DataTypeConversion5_idx_0;
  real32_T rtb_Delay1_bpm_force_B_idx_1;
  real32_T rtb_Delay1_bpm_force_B_idx_2;
  real_T y;
  real_T y_0;

  // Update the flag to indicate when data transfers from
  //   Sample time: [0.016s, 0.0s] to Sample time: [0.16s, 0.0s]
  (sim_model_lib0_M->Timing.RateInteraction.TID0_1)++;
  if ((sim_model_lib0_M->Timing.RateInteraction.TID0_1) > 9) {
    sim_model_lib0_M->Timing.RateInteraction.TID0_1 = 0;
  }

  // Update the flag to indicate when data transfers from
  //   Sample time: [0.016s, 0.0s] to Sample time: [0.20800000000000002s, 0.0s]  
  (sim_model_lib0_M->Timing.RateInteraction.TID0_2)++;
  if ((sim_model_lib0_M->Timing.RateInteraction.TID0_2) > 12) {
    sim_model_lib0_M->Timing.RateInteraction.TID0_2 = 0;
  }

  // Update the flag to indicate when data transfers from
  //   Sample time: [0.016s, 0.0s] to Sample time: [0.272s, 0.0s]
  (sim_model_lib0_M->Timing.RateInteraction.TID0_3)++;
  if ((sim_model_lib0_M->Timing.RateInteraction.TID0_3) > 16) {
    sim_model_lib0_M->Timing.RateInteraction.TID0_3 = 0;
  }

  // Update the flag to indicate when data transfers from
  //   Sample time: [0.016s, 0.0s] to Sample time: [0.336s, 0.0s]
  (sim_model_lib0_M->Timing.RateInteraction.TID0_4)++;
  if ((sim_model_lib0_M->Timing.RateInteraction.TID0_4) > 20) {
    sim_model_lib0_M->Timing.RateInteraction.TID0_4 = 0;
  }

  // RateTransition: '<S81>/Rate Transition3'
  if (sim_model_lib0_M->Timing.RateInteraction.TID0_2 == 1) {
    sim_model_lib0_B->RateTransition3 =
      sim_model_lib0_DW->RateTransition3_Buffer0;

    // RateTransition: '<S81>/Rate Transition1'
    sim_model_lib0_B->RateTransition1 =
      sim_model_lib0_DW->RateTransition1_Buffer0;

    // RateTransition: '<S81>/Rate Transition5'
    memcpy(&sim_model_lib0_B->RateTransition5[0],
           &sim_model_lib0_DW->RateTransition5_Buffer0[0], (uint32_T)(150U *
            sizeof(real32_T)));

    // RateTransition: '<S81>/Rate Transition9'
    memcpy(&sim_model_lib0_B->RateTransition9[0],
           &sim_model_lib0_DW->RateTransition9_Buffer0[0], (uint32_T)(150U *
            sizeof(real32_T)));

    // RateTransition: '<S81>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7[i] =
        sim_model_lib0_DW->RateTransition7_Buffer0[i];
    }

    // End of RateTransition: '<S81>/Rate Transition7'
  }

  // End of RateTransition: '<S81>/Rate Transition3'

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In1' incorporates:
  //   Constant: '<S68>/Constant1'
  //   Selector: '<S74>/select_current_command'
  //   UnitDelay: '<S70>/Unit Delay'

  rtb_A_B_ISS_ISS[0] = sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 71)];

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In2' incorporates:
  //   Constant: '<S68>/Constant1'
  //   Selector: '<S74>/select_current_command'
  //   UnitDelay: '<S70>/Unit Delay'

  rtb_A_B_ISS_ISS[1] = sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 83)];

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In3' incorporates:
  //   Constant: '<S68>/Constant1'
  //   Selector: '<S74>/select_current_command'
  //   UnitDelay: '<S70>/Unit Delay'

  rtb_A_B_ISS_ISS[2] = sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 95)];

  // Bias: '<S70>/Bias' incorporates:
  //   UnitDelay: '<S70>/Unit Delay'

  rtb_Bias = (uint8_T)(uint32_T)((uint32_T)sim_model_lib0_DW->UnitDelay_DSTATE_i
    + (uint32_T)sim_model_lib0_P->Bias_Bias);

  // Switch: '<S74>/Switch1' incorporates:
  //   Constant: '<S68>/Constant1'
  //   Constant: '<S68>/Constant2'
  //   Constant: '<S74>/Constant'
  //   Constant: '<S74>/Constant1'
  //   Constant: '<S74>/Constant2'
  //   RelationalOperator: '<S74>/Relational Operator1'
  //   Selector: '<S74>/select_next_command'
  //   Selector: '<S74>/select_next_time'
  //   Switch: '<S74>/Switch'

  if ((real32_T)rtb_Bias <= sim_model_lib0_P->mlp_num_commands) {
    rtb_Switch1_p_idx_0 = sim_model_lib0_P->mlp_command_times[(int32_T)((int32_T)
      rtb_Bias - 1)];
    rtb_Switch1_p_idx_1 = sim_model_lib0_P->mlp_command_times[(int32_T)((int32_T)
      rtb_Bias + 11)];
    for (i_0 = 0; i_0 < 19; i_0++) {
      rtb_Switch_al[i_0] = sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
        ((int32_T)(12 * i_0) + (int32_T)rtb_Bias) - 1)];
    }
  } else {
    rtb_Switch1_p_idx_0 = sim_model_lib0_P->Constant2_Value_p1[0];
    rtb_Switch1_p_idx_1 = sim_model_lib0_P->Constant2_Value_p1[1];
    memcpy(&rtb_Switch_al[0], &sim_model_lib0_P->mlp_dummy_state_cmd[0],
           (uint32_T)(19U * sizeof(real32_T)));
  }

  // End of Switch: '<S74>/Switch1'

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In1'
  rtb_Merge_o[0] = rtb_Switch_al[6];

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In2'
  rtb_Merge_o[1] = rtb_Switch_al[7];

  // SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In3'
  rtb_Merge_o[2] = rtb_Switch_al[8];

  // Switch: '<S8>/Switch1' incorporates:
  //   Constant: '<S67>/Constant'
  //   Constant: '<S8>/Constant1'
  //   Constant: '<S8>/Constant2'
  //   RelationalOperator: '<S67>/Compare'
  //   Sum: '<S8>/Sum1'
  //   Sum: '<S8>/Sum2'
  //   Switch: '<S8>/Switch'
  //   UnitDelay: '<S8>/Unit Delay'
  //   UnitDelay: '<S8>/Unit Delay1'

  if (sim_model_lib0_DW->UnitDelay_DSTATE >=
      sim_model_lib0_P->CompareToConstant_const_d) {
    rtb_Switch1 = (uint32_T)(sim_model_lib0_P->Constant2_Value_jp +
      sim_model_lib0_DW->UnitDelay1_DSTATE);
    rtb_Switch_g = (uint32_T)(sim_model_lib0_DW->UnitDelay_DSTATE -
      sim_model_lib0_P->Constant1_Value_hc);
  } else {
    rtb_Switch1 = sim_model_lib0_DW->UnitDelay1_DSTATE;
    rtb_Switch_g = sim_model_lib0_DW->UnitDelay_DSTATE;
  }

  // End of Switch: '<S8>/Switch1'

  // Assignment: '<S270>/Assignment' incorporates:
  //   Constant: '<S257>/Constant8'
  //   Constant: '<S270>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'

  memcpy(&rtb_Assignment_jy[0], &sim_model_lib0_P->Constant2_Value_i[0],
         (uint32_T)(9U * sizeof(real_T)));
  rtb_Assignment_jy[0] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];
  rtb_Assignment_jy[4] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];
  rtb_Assignment_jy[8] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];

  // Sum: '<S270>/Sum2' incorporates:
  //   Constant: '<S257>/Constant8'
  //   Constant: '<S272>/Constant3'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'
  //   Gain: '<S272>/Gain'
  //   Gain: '<S272>/Gain1'
  //   Gain: '<S272>/Gain2'

  rtb_Product_p_0[0] = sim_model_lib0_P->Constant3_Value;
  rtb_Product_p_0[1] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[2];
  rtb_Product_p_0[2] = sim_model_lib0_P->Gain_Gain_p * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_Product_p_0[3] = sim_model_lib0_P->Gain1_Gain * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[2];
  rtb_Product_p_0[4] = sim_model_lib0_P->Constant3_Value;
  rtb_Product_p_0[5] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[0];
  rtb_Product_p_0[6] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_Product_p_0[7] = sim_model_lib0_P->Gain2_Gain * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[0];
  rtb_Product_p_0[8] = sim_model_lib0_P->Constant3_Value;

  // Concatenate: '<S270>/Matrix Concatenate' incorporates:
  //   Constant: '<S257>/Constant8'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'
  //   Gain: '<S270>/Gain1'
  //   Sum: '<S270>/Sum2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_MatrixConcatenate[(int32_T)(i_0 << 2)] = rtb_Assignment_jy[(int32_T)(3 *
      i_0)] + rtb_Product_p_0[(int32_T)(3 * i_0)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(i_0 << 2))] =
      rtb_Assignment_jy[(int32_T)((int32_T)(3 * i_0) + 1)] + rtb_Product_p_0
      [(int32_T)((int32_T)(3 * i_0) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(i_0 << 2))] =
      rtb_Assignment_jy[(int32_T)((int32_T)(3 * i_0) + 2)] + rtb_Product_p_0
      [(int32_T)((int32_T)(3 * i_0) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_a * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_a * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_a * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[2];

  // End of Concatenate: '<S270>/Matrix Concatenate'

  // Switch: '<S257>/Switch3' incorporates:
  //   Constant: '<S257>/Constant3'

  tmp_0 = ((int32_T)sim_model_lib0_P->tun_epson_report_truth != 0);

  // Reshape: '<S262>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant8'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'

  rtb_MatrixConcatenate[12] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[0];

  // Product: '<S262>/Product' incorporates:
  //   Constant: '<S257>/Constant1'
  //   Constant: '<S257>/Constant13'
  //   Switch: '<S257>/Switch3'

  if (tmp_0) {
    rtb_Sum = sim_model_lib0_P->Constant1_Value_o[0];
    rtb_DataTypeConversion5_idx_0 = sim_model_lib0_P->Constant1_Value_o[1];
  } else {
    rtb_Sum = sim_model_lib0_P->epson_Q_B2accel_error[0];
    rtb_DataTypeConversion5_idx_0 = sim_model_lib0_P->epson_Q_B2accel_error[1];
  }

  // Reshape: '<S262>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant8'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'

  rtb_MatrixConcatenate[13] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_MatrixConcatenate[14] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[2];

  // Product: '<S262>/Product' incorporates:
  //   Constant: '<S257>/Constant1'
  //   Constant: '<S257>/Constant13'
  //   Switch: '<S257>/Switch3'

  if (tmp_0) {
    rtb_DataTypeConversion5_idx_1 = sim_model_lib0_P->Constant1_Value_o[2];
    rtb_DataTypeConversion5_idx_2 = sim_model_lib0_P->Constant1_Value_o[3];
  } else {
    rtb_DataTypeConversion5_idx_1 = sim_model_lib0_P->epson_Q_B2accel_error[2];
    rtb_DataTypeConversion5_idx_2 = sim_model_lib0_P->epson_Q_B2accel_error[3];
  }

  // Reshape: '<S262>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant8'
  //   DataTypeConversion: '<S257>/Data Type Conversion1'

  rtb_MatrixConcatenate[15] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];

  // Product: '<S262>/Product'
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_linearaccelbias = rtb_MatrixConcatenate[(int32_T)(i_0 + 12)] *
      rtb_DataTypeConversion5_idx_2 + (rtb_MatrixConcatenate[(int32_T)(i_0 + 8)]
      * rtb_DataTypeConversion5_idx_1 + (rtb_MatrixConcatenate[(int32_T)(i_0 + 4)]
      * rtb_DataTypeConversion5_idx_0 + rtb_MatrixConcatenate[i_0] * rtb_Sum));
    rtb_Product_l[i_0] = rtb_linearaccelbias;
  }

  // Sum: '<S284>/Sum' incorporates:
  //   Constant: '<S284>/Constant1'
  //   Gain: '<S284>/Gain'
  //   Math: '<S284>/Math Function'

  rtb_Sum = rtb_Product_l[3] * rtb_Product_l[3] * sim_model_lib0_P->Gain_Gain_f
    - sim_model_lib0_P->Constant1_Value_c;

  // Assignment: '<S284>/Assignment' incorporates:
  //   Constant: '<S284>/Constant2'

  memcpy(&rtb_Assignment[0], &sim_model_lib0_P->Constant2_Value_m[0], (uint32_T)
         (9U * sizeof(real_T)));
  rtb_Assignment[0] = rtb_Sum;
  rtb_Assignment[4] = rtb_Sum;
  rtb_Assignment[8] = rtb_Sum;

  // Gain: '<S284>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_d * rtb_Product_l[3];

  // Product: '<S284>/Product' incorporates:
  //   Constant: '<S287>/Constant3'
  //   Gain: '<S287>/Gain'
  //   Gain: '<S287>/Gain1'
  //   Gain: '<S287>/Gain2'

  rtb_Assignment_3[0] = sim_model_lib0_P->Constant3_Value_n;
  rtb_Assignment_3[1] = rtb_Product_l[2];
  rtb_Assignment_3[2] = sim_model_lib0_P->Gain_Gain_fd * rtb_Product_l[1];
  rtb_Assignment_3[3] = sim_model_lib0_P->Gain1_Gain_l * rtb_Product_l[2];
  rtb_Assignment_3[4] = sim_model_lib0_P->Constant3_Value_n;
  rtb_Assignment_3[5] = rtb_Product_l[0];
  rtb_Assignment_3[6] = rtb_Product_l[1];
  rtb_Assignment_3[7] = sim_model_lib0_P->Gain2_Gain_a * rtb_Product_l[0];
  rtb_Assignment_3[8] = sim_model_lib0_P->Constant3_Value_n;

  // Product: '<S284>/Product1' incorporates:
  //   Gain: '<S284>/Gain2'
  //   Math: '<S284>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_p_0[i_0] = rtb_Product_l[i_0] * rtb_Product_l[0];
    rtb_Product_p_0[(int32_T)(i_0 + 3)] = rtb_Product_l[i_0] * rtb_Product_l[1];
    rtb_Product_p_0[(int32_T)(i_0 + 6)] = rtb_Product_l[i_0] * rtb_Product_l[2];
  }

  // End of Product: '<S284>/Product1'

  // DotProduct: '<S27>/Dot Product'
  rtb_Sqrt_m = 0.0F;
  for (i_0 = 0; i_0 < 3; i_0++) {
    // Sum: '<S284>/Sum1' incorporates:
    //   Gain: '<S284>/Gain2'
    //   Product: '<S284>/Product'

    rtb_Assignment_jy[(int32_T)(3 * i_0)] = (rtb_Assignment[(int32_T)(3 * i_0)]
      - rtb_Assignment_3[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_p_0
      [(int32_T)(3 * i_0)] * sim_model_lib0_P->Gain2_Gain_d;
    rtb_Assignment_jy[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 1)] - rtb_Assignment_3[(int32_T)((int32_T)
      (3 * i_0) + 1)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0)
      + 1)] * sim_model_lib0_P->Gain2_Gain_d;
    rtb_Assignment_jy[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 2)] - rtb_Assignment_3[(int32_T)((int32_T)
      (3 * i_0) + 2)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0)
      + 2)] * sim_model_lib0_P->Gain2_Gain_d;

    // DotProduct: '<S27>/Dot Product' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_Sqrt_m += sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[i_0] *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[i_0];
  }

  // Sqrt: '<S27>/Sqrt' incorporates:
  //   DotProduct: '<S27>/Dot Product'

  rtb_Sqrt_m = (real32_T)sqrt((real_T)rtb_Sqrt_m);

  // If: '<S11>/If' incorporates:
  //   Constant: '<S22>/Constant'
  //   Constant: '<S23>/Constant'
  //   Constant: '<S30>/Constant'
  //   Constant: '<S30>/Constant1'
  //   Constant: '<S30>/Constant2'
  //   Constant: '<S30>/Constant3'
  //   Constant: '<S31>/Constant'
  //   Constant: '<S31>/Constant1'
  //   Constant: '<S31>/Constant2'
  //   Constant: '<S31>/Constant3'
  //   RelationalOperator: '<S22>/Compare'
  //   RelationalOperator: '<S23>/Compare'

  if (rtb_Sqrt_m == sim_model_lib0_P->CompareToConstant1_const) {
    // Outputs for IfAction SubSystem: '<S11>/Stopped' incorporates:
    //   ActionPort: '<S24>/Action Port'

    // SignalConversion: '<S24>/OutportBufferForTheta' incorporates:
    //   Constant: '<S24>/Constant'

    memcpy(&rtb_VectorConcatenate_d[0], &sim_model_lib0_P->Constant_Value_c[0],
           (uint32_T)(sizeof(real32_T) << 4U));

    // End of Outputs for SubSystem: '<S11>/Stopped'
  } else if (!(rtb_Sqrt_m <= sim_model_lib0_P->CompareToConstant_const)) {
    // Outputs for IfAction SubSystem: '<S11>/slow_speed_approx' incorporates:
    //   ActionPort: '<S26>/Action Port'

    // DataTypeConversion: '<S26>/Data Type Conversion' incorporates:
    //   Gain: '<S26>/Gain'
    //   SampleTimeMath: '<S26>/Weighted Sample Time Math'
    //
    //  About '<S26>/Weighted Sample Time Math':
    //   y = K where K = ( w * Ts )

    rtb_Sqrt_m = (real32_T)(sim_model_lib0_P->Gain_Gain *
      sim_model_lib0_P->WeightedSampleTimeMath_WtEt);
    rtb_VectorConcatenate_d[0] = sim_model_lib0_P->Constant3_Value_g2;

    // Gain: '<S31>/Gain' incorporates:
    //   Constant: '<S31>/Constant3'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[1] = sim_model_lib0_P->Gain_Gain_dy *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn3' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // Gain: '<S31>/Gain1' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[3] = sim_model_lib0_P->Gain1_Gain_ow *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn5' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[4] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    rtb_VectorConcatenate_d[5] = sim_model_lib0_P->Constant2_Value_l;

    // Gain: '<S31>/Gain2' incorporates:
    //   Constant: '<S31>/Constant2'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[6] = sim_model_lib0_P->Gain2_Gain_oe *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // Gain: '<S31>/Gain3' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[7] = sim_model_lib0_P->Gain3_Gain *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // Gain: '<S31>/Gain4' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[8] = sim_model_lib0_P->Gain4_Gain *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn10' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[9] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    rtb_VectorConcatenate_d[10] = sim_model_lib0_P->Constant1_Value_p4;

    // Gain: '<S31>/Gain5' incorporates:
    //   Constant: '<S31>/Constant1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[11] = sim_model_lib0_P->Gain5_Gain *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn13' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[12] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn14' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[13] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // SignalConversion: '<S31>/ConcatBufferAtVector ConcatenateIn15' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[14] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    rtb_VectorConcatenate_d[15] = sim_model_lib0_P->Constant_Value_d;

    // Sum: '<S26>/Sum' incorporates:
    //   Constant: '<S26>/Constant'
    //   Constant: '<S31>/Constant'
    //   Product: '<S26>/Product'

    for (i_0 = 0; i_0 < 16; i_0++) {
      rtb_VectorConcatenate_d[i_0] = rtb_Sqrt_m * rtb_VectorConcatenate_d[i_0] +
        sim_model_lib0_P->Constant_Value_p[i_0];
    }

    // End of Sum: '<S26>/Sum'
    // End of Outputs for SubSystem: '<S11>/slow_speed_approx'
  } else {
    // Outputs for IfAction SubSystem: '<S11>/normal_speed' incorporates:
    //   ActionPort: '<S25>/Action Port'

    // Product: '<S25>/Product' incorporates:
    //   DataTypeConversion: '<S25>/Data Type Conversion'
    //   Gain: '<S25>/Gain'
    //   SampleTimeMath: '<S25>/Weighted Sample Time Math'
    //
    //  About '<S25>/Weighted Sample Time Math':
    //   y = K where K = ( w * Ts )

    rtb_Product_p = (real32_T)(sim_model_lib0_P->Gain_Gain_j *
      sim_model_lib0_P->WeightedSampleTimeMath_WtEt_o) * rtb_Sqrt_m;

    // Trigonometry: '<S25>/Trigonometric Function1'
    rtb_TrigonometricFunction1_c = (real32_T)cos((real_T)rtb_Product_p);

    // Assignment: '<S25>/Assignment' incorporates:
    //   Constant: '<S25>/Constant2'
    //   DataTypeConversion: '<S29>/Conversion'

    for (i = 0; i < 16; i++) {
      rtb_Assignment_ok[i] = (real32_T)sim_model_lib0_P->Constant2_Value[i];
    }

    rtb_Assignment_ok[0] = rtb_TrigonometricFunction1_c;
    rtb_Assignment_ok[5] = rtb_TrigonometricFunction1_c;
    rtb_Assignment_ok[10] = rtb_TrigonometricFunction1_c;
    rtb_Assignment_ok[15] = rtb_TrigonometricFunction1_c;

    // End of Assignment: '<S25>/Assignment'

    // Trigonometry: '<S25>/Trigonometric Function'
    rtb_Product_p = (real32_T)sin((real_T)rtb_Product_p);
    rtb_VectorConcatenate_d[0] = sim_model_lib0_P->Constant3_Value_pa;

    // Gain: '<S30>/Gain' incorporates:
    //   Constant: '<S30>/Constant3'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[1] = sim_model_lib0_P->Gain_Gain_g *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn3' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // Gain: '<S30>/Gain1' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[3] = sim_model_lib0_P->Gain1_Gain_c *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn5' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[4] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    rtb_VectorConcatenate_d[5] = sim_model_lib0_P->Constant2_Value_dl;

    // Gain: '<S30>/Gain2' incorporates:
    //   Constant: '<S30>/Constant2'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[6] = sim_model_lib0_P->Gain2_Gain_l *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // Gain: '<S30>/Gain3' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[7] = sim_model_lib0_P->Gain3_Gain_i *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // Gain: '<S30>/Gain4' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[8] = sim_model_lib0_P->Gain4_Gain_f *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn10' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[9] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    rtb_VectorConcatenate_d[10] = sim_model_lib0_P->Constant1_Value_of;

    // Gain: '<S30>/Gain5' incorporates:
    //   Constant: '<S30>/Constant1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[11] = sim_model_lib0_P->Gain5_Gain_l *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn13' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[12] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn14' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[13] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];

    // SignalConversion: '<S30>/ConcatBufferAtVector ConcatenateIn15' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtb_VectorConcatenate_d[14] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    rtb_VectorConcatenate_d[15] = sim_model_lib0_P->Constant_Value_f;

    // Product: '<S25>/Divide' incorporates:
    //   Constant: '<S30>/Constant'

    rtb_Sqrt_m = 1.0F / rtb_Sqrt_m * rtb_Product_p;

    // Sum: '<S25>/Sum' incorporates:
    //   Product: '<S25>/Divide'

    for (i_0 = 0; i_0 < 16; i_0++) {
      rtb_VectorConcatenate_d[i_0] = rtb_Sqrt_m * rtb_VectorConcatenate_d[i_0] +
        rtb_Assignment_ok[i_0];
    }

    // End of Sum: '<S25>/Sum'
    // End of Outputs for SubSystem: '<S11>/normal_speed'
  }

  // End of If: '<S11>/If'

  // Delay: '<S11>/Delay' incorporates:
  //   Constant: '<S5>/Constant1'

  if ((int32_T)sim_model_lib0_DW->icLoad != 0) {
    sim_model_lib0_DW->Delay_DSTATE[0] = sim_model_lib0_P->tun_ini_Q_ISS2B[0];
    sim_model_lib0_DW->Delay_DSTATE[1] = sim_model_lib0_P->tun_ini_Q_ISS2B[1];
    sim_model_lib0_DW->Delay_DSTATE[2] = sim_model_lib0_P->tun_ini_Q_ISS2B[2];
    sim_model_lib0_DW->Delay_DSTATE[3] = sim_model_lib0_P->tun_ini_Q_ISS2B[3];
  }

  // DotProduct: '<S34>/Dot Product'
  rtb_Sqrt_m = 0.0F;
  for (i_0 = 0; i_0 < 4; i_0++) {
    // Product: '<S11>/Product' incorporates:
    //   Delay: '<S11>/Delay'

    rtb_TrigonometricFunction1_c = rtb_VectorConcatenate_d[(int32_T)(i_0 + 12)] *
      sim_model_lib0_DW->Delay_DSTATE[3] + (rtb_VectorConcatenate_d[(int32_T)
      (i_0 + 8)] * sim_model_lib0_DW->Delay_DSTATE[2] +
      (rtb_VectorConcatenate_d[(int32_T)(i_0 + 4)] *
       sim_model_lib0_DW->Delay_DSTATE[1] + rtb_VectorConcatenate_d[i_0] *
       sim_model_lib0_DW->Delay_DSTATE[0]));

    // DotProduct: '<S34>/Dot Product'
    rtb_Sqrt_m += rtb_TrigonometricFunction1_c * rtb_TrigonometricFunction1_c;

    // Product: '<S11>/Product'
    rtb_Product_e[i_0] = rtb_TrigonometricFunction1_c;
  }

  // Sqrt: '<S34>/Sqrt' incorporates:
  //   DotProduct: '<S34>/Dot Product'

  rtb_Sqrt_m = (real32_T)sqrt((real_T)rtb_Sqrt_m);

  // If: '<S28>/If' incorporates:
  //   DataTypeConversion: '<S28>/Data Type Conversion'
  //   Inport: '<S32>/In1'

  if ((real_T)rtb_Sqrt_m > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S28>/Normalize' incorporates:
    //   ActionPort: '<S33>/Action Port'

    sim_model_lib0_Normalize(rtb_Product_e, rtb_Sqrt_m, rtb_Merge_kq);

    // End of Outputs for SubSystem: '<S28>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S28>/No-op' incorporates:
    //   ActionPort: '<S32>/Action Port'

    rtb_Merge_kq[0] = rtb_Product_e[0];
    rtb_Merge_kq[1] = rtb_Product_e[1];
    rtb_Merge_kq[2] = rtb_Product_e[2];
    rtb_Merge_kq[3] = rtb_Product_e[3];

    // End of Outputs for SubSystem: '<S28>/No-op'
  }

  // End of If: '<S28>/If'

  // If: '<S10>/If' incorporates:
  //   Inport: '<S15>/In1'

  if (rtb_Merge_kq[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S10>/Normalize' incorporates:
    //   ActionPort: '<S16>/Action Port'

    // Product: '<S16>/Product' incorporates:
    //   Constant: '<S16>/Constant1'
    //   DataTypeConversion: '<S18>/Conversion'

    rtb_Product_e[0] = rtb_Merge_kq[0] * (real32_T)
      sim_model_lib0_P->Constant1_Value;
    rtb_Product_e[1] = rtb_Merge_kq[1] * (real32_T)
      sim_model_lib0_P->Constant1_Value;
    rtb_Product_e[2] = rtb_Merge_kq[2] * (real32_T)
      sim_model_lib0_P->Constant1_Value;
    rtb_Product_e[3] = rtb_Merge_kq[3] * (real32_T)
      sim_model_lib0_P->Constant1_Value;

    // End of Outputs for SubSystem: '<S10>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S10>/No-op' incorporates:
    //   ActionPort: '<S15>/Action Port'

    rtb_Product_e[0] = rtb_Merge_kq[0];
    rtb_Product_e[1] = rtb_Merge_kq[1];
    rtb_Product_e[2] = rtb_Merge_kq[2];
    rtb_Product_e[3] = rtb_Merge_kq[3];

    // End of Outputs for SubSystem: '<S10>/No-op'
  }

  // End of If: '<S10>/If'

  // Sqrt: '<S21>/Sqrt' incorporates:
  //   DotProduct: '<S21>/Dot Product'

  rtb_Sqrt_m = (real32_T)sqrt((real_T)(((rtb_Product_e[0] * rtb_Product_e[0] +
    rtb_Product_e[1] * rtb_Product_e[1]) + rtb_Product_e[2] * rtb_Product_e[2])
    + rtb_Product_e[3] * rtb_Product_e[3]));

  // If: '<S17>/If' incorporates:
  //   DataTypeConversion: '<S17>/Data Type Conversion'
  //   Inport: '<S19>/In1'

  if ((real_T)rtb_Sqrt_m > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S17>/Normalize' incorporates:
    //   ActionPort: '<S20>/Action Port'

    sim_model_lib0_Normalize(rtb_Product_e, rtb_Sqrt_m, rtb_Merge_m);

    // End of Outputs for SubSystem: '<S17>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S17>/No-op' incorporates:
    //   ActionPort: '<S19>/Action Port'

    rtb_Merge_m[0] = rtb_Product_e[0];
    rtb_Merge_m[1] = rtb_Product_e[1];
    rtb_Merge_m[2] = rtb_Product_e[2];
    rtb_Merge_m[3] = rtb_Product_e[3];

    // End of Outputs for SubSystem: '<S17>/No-op'
  }

  // End of If: '<S17>/If'

  // Sum: '<S62>/Sum' incorporates:
  //   Constant: '<S62>/Constant1'
  //   DataTypeConversion: '<S64>/Conversion'
  //   Gain: '<S62>/Gain'
  //   Math: '<S62>/Math Function'

  rtb_Sqrt_m = rtb_Merge_m[3] * rtb_Merge_m[3] * sim_model_lib0_P->Gain_Gain_m -
    (real32_T)sim_model_lib0_P->Constant1_Value_oo;

  // Assignment: '<S62>/Assignment' incorporates:
  //   Constant: '<S62>/Constant2'
  //   DataTypeConversion: '<S63>/Conversion'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_m[i] = (real32_T)sim_model_lib0_P->Constant2_Value_m1[i];
  }

  rtb_Assignment_m[0] = rtb_Sqrt_m;

  // Delay: '<S1>/Delay1'
  rtb_Delay1_bpm_torque_B[0] = sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[0];
  rtb_TrigonometricFunction1_c = sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[0];

  // Assignment: '<S62>/Assignment'
  rtb_Assignment_m[4] = rtb_Sqrt_m;

  // Delay: '<S1>/Delay1'
  rtb_Delay1_bpm_torque_B[1] = sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[1];
  rtb_Delay1_bpm_force_B_idx_1 = sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[1];

  // Assignment: '<S62>/Assignment'
  rtb_Assignment_m[8] = rtb_Sqrt_m;

  // Delay: '<S1>/Delay1'
  rtb_Delay1_bpm_torque_B[2] = sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[2];
  rtb_Delay1_bpm_force_B_idx_2 = sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[2];

  // Gain: '<S62>/Gain1'
  rtb_Sqrt_m = sim_model_lib0_P->Gain1_Gain_oi * rtb_Merge_m[3];

  // BusAssignment: '<S1>/Bus Assignment1' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Constant: '<S1>/Constant10'
  //   Constant: '<S1>/Constant11'
  //   Constant: '<S1>/Constant2'
  //   Constant: '<S1>/Constant3'
  //   Constant: '<S1>/Constant4'
  //   Constant: '<S1>/Constant7'
  //   Constant: '<S1>/Constant8'
  //   Constant: '<S1>/Constant9'
  //   Inport: '<Root>/cmc_msg_in'

  rtb_BusAssignment1 = *sim_model_lib0_U_cmc_msg_in;
  rtb_BusAssignment1.att_kp[0] = sim_model_lib0_P->tun_default_att_kp[0];
  rtb_BusAssignment1.att_ki[0] = sim_model_lib0_P->tun_default_att_ki[0];
  rtb_BusAssignment1.omega_kd[0] = sim_model_lib0_P->tun_default_omega_kd[0];
  rtb_BusAssignment1.pos_kp[0] = sim_model_lib0_P->tun_default_pos_kp[0];
  rtb_BusAssignment1.pos_ki[0] = sim_model_lib0_P->tun_default_pos_ki[0];
  rtb_BusAssignment1.vel_kd[0] = sim_model_lib0_P->tun_default_vel_kd[0];
  rtb_BusAssignment1.center_of_mass[0] =
    sim_model_lib0_P->tun_default_center_of_mass[0];
  rtb_BusAssignment1.att_kp[1] = sim_model_lib0_P->tun_default_att_kp[1];
  rtb_BusAssignment1.att_ki[1] = sim_model_lib0_P->tun_default_att_ki[1];
  rtb_BusAssignment1.omega_kd[1] = sim_model_lib0_P->tun_default_omega_kd[1];
  rtb_BusAssignment1.pos_kp[1] = sim_model_lib0_P->tun_default_pos_kp[1];
  rtb_BusAssignment1.pos_ki[1] = sim_model_lib0_P->tun_default_pos_ki[1];
  rtb_BusAssignment1.vel_kd[1] = sim_model_lib0_P->tun_default_vel_kd[1];
  rtb_BusAssignment1.center_of_mass[1] =
    sim_model_lib0_P->tun_default_center_of_mass[1];
  rtb_BusAssignment1.att_kp[2] = sim_model_lib0_P->tun_default_att_kp[2];
  rtb_BusAssignment1.att_ki[2] = sim_model_lib0_P->tun_default_att_ki[2];
  rtb_BusAssignment1.omega_kd[2] = sim_model_lib0_P->tun_default_omega_kd[2];
  rtb_BusAssignment1.pos_kp[2] = sim_model_lib0_P->tun_default_pos_kp[2];
  rtb_BusAssignment1.pos_ki[2] = sim_model_lib0_P->tun_default_pos_ki[2];
  rtb_BusAssignment1.vel_kd[2] = sim_model_lib0_P->tun_default_vel_kd[2];
  rtb_BusAssignment1.center_of_mass[2] =
    sim_model_lib0_P->tun_default_center_of_mass[2];
  for (i = 0; i < 9; i++) {
    rtb_BusAssignment1.inertia_matrix[i] =
      sim_model_lib0_P->tun_default_inertia_matrix[i];
  }

  rtb_BusAssignment1.mass = sim_model_lib0_P->tun_default_mass;

  // BusAssignment: '<S3>/Bus Assignment' incorporates:
  //   BusCreator: '<S3>/BusConversion_InsertedFor_Bus Assignment_at_inport_1'
  //   BusCreator: '<S3>/BusConversion_InsertedFor_Bus Assignment_at_inport_2'
  //   Constant: '<S68>/Constant1'
  //   Constant: '<S68>/Constant2'
  //   Constant: '<S68>/Constant4'
  //   Constant: '<S68>/Constant6'
  //   Constant: '<S68>/Constant8'
  //   Selector: '<S68>/select_current_command'
  //   Selector: '<S68>/select_current_command1'
  //   Selector: '<S68>/select_current_command2'
  //   Selector: '<S74>/select_current_command'
  //   Selector: '<S74>/select_current_time'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate10In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate10In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate10In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate10In4'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate11In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate11In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate11In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate12In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate12In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate12In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate1In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate1In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate1In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate2In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate2In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate2In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate3In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate4In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate4In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate4In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate4In4'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate5In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate5In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate5In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate6In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate6In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate6In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate7In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate7In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate7In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate8In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate8In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate8In3'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In1'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In2'
  //   SignalConversion: '<S74>/ConcatBufferAtVector Concatenate9In3'
  //   UnitDelay: '<S70>/Unit Delay'
  //   UnitDelay: '<S71>/Unit Delay'
  //   UnitDelay: '<S72>/Unit Delay'
  //   UnitDelay: '<S73>/Unit Delay'

  rtb_BusAssignment = rtb_BusAssignment1;
  rtb_BusAssignment.cmc_state_cmd_a.timestamp_sec =
    sim_model_lib0_P->mlp_command_times[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i - 1)];
  rtb_BusAssignment.cmc_state_cmd_a.timestamp_nsec =
    sim_model_lib0_P->mlp_command_times[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 11)];
  rtb_BusAssignment.cmc_state_cmd_a.P_B_ISS_ISS[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i - 1)];
  rtb_BusAssignment.cmc_state_cmd_a.V_B_ISS_ISS[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 35)];
  rtb_BusAssignment.cmc_state_cmd_a.A_B_ISS_ISS[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 71)];
  rtb_BusAssignment.cmc_state_cmd_a.P_B_ISS_ISS[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 11)];
  rtb_BusAssignment.cmc_state_cmd_a.V_B_ISS_ISS[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 47)];
  rtb_BusAssignment.cmc_state_cmd_a.A_B_ISS_ISS[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 83)];
  rtb_BusAssignment.cmc_state_cmd_a.P_B_ISS_ISS[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 23)];
  rtb_BusAssignment.cmc_state_cmd_a.V_B_ISS_ISS[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 59)];
  rtb_BusAssignment.cmc_state_cmd_a.A_B_ISS_ISS[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 95)];
  rtb_BusAssignment.cmc_state_cmd_a.quat_ISS2B[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 107)];
  rtb_BusAssignment.cmc_state_cmd_a.quat_ISS2B[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 119)];
  rtb_BusAssignment.cmc_state_cmd_a.quat_ISS2B[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 131)];
  rtb_BusAssignment.cmc_state_cmd_a.quat_ISS2B[3] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 143)];
  rtb_BusAssignment.cmc_state_cmd_b.timestamp_sec = rtb_Switch1_p_idx_0;
  rtb_BusAssignment.cmc_state_cmd_b.timestamp_nsec = rtb_Switch1_p_idx_1;
  rtb_BusAssignment.cmc_state_cmd_a.omega_B_ISS_B[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 155)];
  rtb_BusAssignment.cmc_state_cmd_a.alpha_B_ISS_B[0] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 191)];
  rtb_BusAssignment.cmc_state_cmd_b.P_B_ISS_ISS[0] = rtb_Switch_al[0];
  rtb_BusAssignment.cmc_state_cmd_b.V_B_ISS_ISS[0] = rtb_Switch_al[3];
  rtb_BusAssignment.cmc_state_cmd_b.A_B_ISS_ISS[0] = rtb_Switch_al[6];
  rtb_BusAssignment.cmc_state_cmd_a.omega_B_ISS_B[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 167)];
  rtb_BusAssignment.cmc_state_cmd_a.alpha_B_ISS_B[1] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 203)];
  rtb_BusAssignment.cmc_state_cmd_b.P_B_ISS_ISS[1] = rtb_Switch_al[1];
  rtb_BusAssignment.cmc_state_cmd_b.V_B_ISS_ISS[1] = rtb_Switch_al[4];
  rtb_BusAssignment.cmc_state_cmd_b.A_B_ISS_ISS[1] = rtb_Switch_al[7];
  rtb_BusAssignment.cmc_state_cmd_a.omega_B_ISS_B[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 179)];
  rtb_BusAssignment.cmc_state_cmd_a.alpha_B_ISS_B[2] =
    sim_model_lib0_P->mlp_command_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_i + 215)];
  rtb_BusAssignment.cmc_state_cmd_b.P_B_ISS_ISS[2] = rtb_Switch_al[2];
  rtb_BusAssignment.cmc_state_cmd_b.V_B_ISS_ISS[2] = rtb_Switch_al[5];
  rtb_BusAssignment.cmc_state_cmd_b.A_B_ISS_ISS[2] = rtb_Switch_al[8];
  rtb_BusAssignment.cmc_state_cmd_b.quat_ISS2B[0] = rtb_Switch_al[9];
  rtb_BusAssignment.cmc_state_cmd_b.quat_ISS2B[1] = rtb_Switch_al[10];
  rtb_BusAssignment.cmc_state_cmd_b.quat_ISS2B[2] = rtb_Switch_al[11];
  rtb_BusAssignment.cmc_state_cmd_b.quat_ISS2B[3] = rtb_Switch_al[12];
  rtb_BusAssignment.cmc_state_cmd_b.omega_B_ISS_B[0] = rtb_Switch_al[13];
  rtb_BusAssignment.cmc_state_cmd_b.alpha_B_ISS_B[0] = rtb_Switch_al[16];
  rtb_BusAssignment.cmc_state_cmd_b.omega_B_ISS_B[1] = rtb_Switch_al[14];
  rtb_BusAssignment.cmc_state_cmd_b.alpha_B_ISS_B[1] = rtb_Switch_al[17];
  rtb_BusAssignment.cmc_state_cmd_b.omega_B_ISS_B[2] = rtb_Switch_al[15];
  rtb_BusAssignment.cmc_state_cmd_b.alpha_B_ISS_B[2] = rtb_Switch_al[18];
  rtb_BusAssignment.cmc_mode_cmd = sim_model_lib0_P->mlp_mode_cmd_list[(int32_T)
    ((int32_T)sim_model_lib0_DW->UnitDelay_DSTATE_if - 1)];
  rtb_BusAssignment.speed_gain_cmd = sim_model_lib0_P->mlp_speed_gain_cmd_list
    [(int32_T)((int32_T)sim_model_lib0_DW->UnitDelay_DSTATE_h - 1)];
  rtb_BusAssignment.localization_mode_cmd =
    sim_model_lib0_P->mlp_loc_mode_cmd_list[(int32_T)((int32_T)
    sim_model_lib0_DW->UnitDelay_DSTATE_ht - 1)];

  // Outport: '<Root>/cmc_msg'
  *sim_model_lib0_Y_cmc_msg_c = rtb_BusAssignment;

  // Sqrt: '<S45>/Sqrt' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   DotProduct: '<S45>/Dot Product'

  rtb_Product_p = (real32_T)sqrt((real_T)
    ((sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] +
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] *
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1]) +
     sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] *
     sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2]));

  // If: '<S42>/If' incorporates:
  //   DataTypeConversion: '<S42>/Data Type Conversion'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   Inport: '<S43>/In1'

  if ((real_T)rtb_Product_p > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S42>/Normalize' incorporates:
    //   ActionPort: '<S44>/Action Port'

    sim_model_lib0_Normalize_e
      (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d, rtb_Product_p,
       rtb_Merge_o);

    // End of Outputs for SubSystem: '<S42>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S42>/No-op' incorporates:
    //   ActionPort: '<S43>/Action Port'

    rtb_Merge_o[0] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0];
    rtb_Merge_o[1] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1];
    rtb_Merge_o[2] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2];

    // End of Outputs for SubSystem: '<S42>/No-op'
  }

  // End of If: '<S42>/If'

  // Switch: '<S6>/Switch1' incorporates:
  //   Constant: '<S6>/Constant1'
  //   Constant: '<S6>/Constant6'
  //   Constant: '<S6>/Constant7'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   Gain: '<S6>/Gain'
  //   Product: '<S6>/Divide'
  //   Sum: '<S6>/Sum'

  if ((int32_T)sim_model_lib0_P->tun_env_drag_disturb_on != 0) {
    // Switch: '<S40>/Switch' incorporates:
    //   Constant: '<S6>/Constant9'
    //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
    //   DotProduct: '<S41>/Dot Product'
    //   Product: '<S40>/Product'
    //   RelationalOperator: '<S40>/Relational Operator'
    //   Sqrt: '<S41>/Sqrt'

    if ((real32_T)sqrt((real_T)
                       ((sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] *
                         sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] +
                         sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] *
                         sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1])
                        + sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] *
                        sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2])) <
        sim_model_lib0_P->env_max_ext_air_vel) {
      rtb_Merge_o[0] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0];
      rtb_Merge_o[1] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1];
      rtb_Merge_o[2] = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2];
    } else {
      rtb_Merge_o[0] *= sim_model_lib0_P->env_max_ext_air_vel;
      rtb_Merge_o[1] *= sim_model_lib0_P->env_max_ext_air_vel;
      rtb_Merge_o[2] *= sim_model_lib0_P->env_max_ext_air_vel;
    }

    // End of Switch: '<S40>/Switch'

    // Sum: '<S6>/Sum' incorporates:
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

    rtb_Merge_c = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0] -
      rtb_Merge_o[0];

    // Product: '<S6>/Divide'
    if (rtb_Merge_c < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Merge_c > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Merge_c == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Merge_c;
    }

    rtb_Merge_c = sim_model_lib0_P->Gain_Gain_go * rtb_Product_p * rtb_Merge_c *
      rtb_Merge_c * sim_model_lib0_P->env_avg_drag_coeff;
    rtb_Merge_o[0] = rtb_Merge_c;

    // Sum: '<S6>/Sum' incorporates:
    //   Constant: '<S6>/Constant1'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   Gain: '<S6>/Gain'
    //   Product: '<S6>/Divide'

    rtb_Merge_c = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1] -
      rtb_Merge_o[1];

    // Product: '<S6>/Divide'
    if (rtb_Merge_c < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Merge_c > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Merge_c == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Merge_c;
    }

    rtb_Merge_c = sim_model_lib0_P->Gain_Gain_go * rtb_Product_p * rtb_Merge_c *
      rtb_Merge_c * sim_model_lib0_P->env_avg_drag_coeff;
    rtb_Merge_o[1] = rtb_Merge_c;

    // Sum: '<S6>/Sum' incorporates:
    //   Constant: '<S6>/Constant1'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   Gain: '<S6>/Gain'
    //   Product: '<S6>/Divide'

    rtb_Merge_c = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2] -
      rtb_Merge_o[2];

    // Product: '<S6>/Divide'
    if (rtb_Merge_c < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Merge_c > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Merge_c == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Merge_c;
    }

    rtb_Merge_c = sim_model_lib0_P->Gain_Gain_go * rtb_Product_p * rtb_Merge_c *
      rtb_Merge_c * sim_model_lib0_P->env_avg_drag_coeff;
    rtb_Merge_o[2] = rtb_Merge_c;
  } else {
    rtb_Merge_o[0] = sim_model_lib0_P->Constant7_Value[0];
    rtb_Merge_o[1] = sim_model_lib0_P->Constant7_Value[1];
    rtb_Merge_o[2] = sim_model_lib0_P->Constant7_Value[2];
  }

  // End of Switch: '<S6>/Switch1'

  // Sum: '<S2>/Sum' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant11'
  //   Constant: '<S2>/Constant'

  rtb_Product_p = sim_model_lib0_P->tun_mass_error +
    sim_model_lib0_P->tun_default_mass;

  // Product: '<S62>/Product' incorporates:
  //   Constant: '<S65>/Constant3'
  //   DataTypeConversion: '<S66>/Conversion'
  //   Gain: '<S65>/Gain'
  //   Gain: '<S65>/Gain1'
  //   Gain: '<S65>/Gain2'

  tmp_1[0] = (real32_T)sim_model_lib0_P->Constant3_Value_k;
  tmp_1[1] = rtb_Merge_m[2];
  tmp_1[2] = sim_model_lib0_P->Gain_Gain_j1 * rtb_Merge_m[1];
  tmp_1[3] = sim_model_lib0_P->Gain1_Gain_e * rtb_Merge_m[2];
  tmp_1[4] = (real32_T)sim_model_lib0_P->Constant3_Value_k;
  tmp_1[5] = rtb_Merge_m[0];
  tmp_1[6] = rtb_Merge_m[1];
  tmp_1[7] = sim_model_lib0_P->Gain2_Gain_p * rtb_Merge_m[0];
  tmp_1[8] = (real32_T)sim_model_lib0_P->Constant3_Value_k;

  // Math: '<S51>/Math Function' incorporates:
  //   Gain: '<S62>/Gain2'
  //   Math: '<S62>/Math Function1'
  //   Product: '<S62>/Product1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Merge_b[i_0] = rtb_Merge_m[0] * rtb_Merge_m[i_0];
    rtb_Merge_b[(int32_T)(i_0 + 3)] = rtb_Merge_m[1] * rtb_Merge_m[i_0];
    rtb_Merge_b[(int32_T)(i_0 + 6)] = rtb_Merge_m[2] * rtb_Merge_m[i_0];
  }

  // End of Math: '<S51>/Math Function'

  // Sum: '<S62>/Sum1' incorporates:
  //   Gain: '<S62>/Gain2'
  //   Product: '<S51>/Product'
  //   Product: '<S62>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_i[(int32_T)(3 * i_0)] = (rtb_Assignment_m[i_0] - rtb_Sqrt_m *
      tmp_1[i_0]) + rtb_Merge_b[(int32_T)(3 * i_0)] *
      sim_model_lib0_P->Gain2_Gain_h;
    rtb_Assignment_i[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)(i_0 + 3)] - tmp_1[(int32_T)(i_0 + 3)] * rtb_Sqrt_m) +
      rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_h;
    rtb_Assignment_i[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)(i_0 + 6)] - tmp_1[(int32_T)(i_0 + 6)] * rtb_Sqrt_m) +
      rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_h;
  }

  // End of Sum: '<S62>/Sum1'

  // Product: '<S46>/Element product' incorporates:
  //   Constant: '<S7>/Constant1'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  rtb_Elementproduct_c[0] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
  rtb_Elementproduct_c[1] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
  rtb_Elementproduct_c[2] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
  rtb_Elementproduct_c[3] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
  rtb_Elementproduct_c[4] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
  rtb_Elementproduct_c[5] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
  for (i = 0; i < 3; i++) {
    // Product: '<S7>/Product1' incorporates:
    //   Constant: '<S6>/Constant2'
    //   Constant: '<S6>/Constant3'
    //   Constant: '<S6>/Constant5'
    //   Product: '<S51>/Product'
    //   Product: '<S6>/Divide1'
    //   Sum: '<S6>/Sum1'
    //   Sum: '<S7>/Add1'
    //   Switch: '<S6>/Switch'

    if (sim_model_lib0_P->env_grav_disturb_on) {
      rtb_Sqrt_m = rtb_Product_p * sim_model_lib0_P->const_gravity_local[i];
    } else {
      rtb_Sqrt_m = sim_model_lib0_P->Constant5_Value_av[i];
    }

    rtb_bpm_force_B[i] = (((rtb_Assignment_i[(int32_T)(i + 3)] *
      rtb_Delay1_bpm_force_B_idx_1 + rtb_Assignment_i[i] *
      rtb_TrigonometricFunction1_c) + rtb_Assignment_i[(int32_T)(i + 6)] *
      rtb_Delay1_bpm_force_B_idx_2) + (rtb_Merge_o[i] + rtb_Sqrt_m)) /
      rtb_Product_p;

    // End of Product: '<S7>/Product1'

    // Product: '<S7>/Product3' incorporates:
    //   Constant: '<S7>/Constant10'
    //   Sum: '<S46>/Add3'

    rtb_P_B_ISS_SS[i] = (rtb_Elementproduct_c[i] - rtb_Elementproduct_c[(int32_T)
                         (i + 3)]) * sim_model_lib0_P->Constant10_Value_a;
  }

  // Sum: '<S47>/Add3' incorporates:
  //   Constant: '<S7>/Constant1'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
  //   Product: '<S47>/Element product'

  rtb_Merge_o[0] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2] -
    sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
  rtb_Merge_o[1] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0] -
    sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
  rtb_Merge_o[2] = sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1] -
    sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];

  // Gain: '<S7>/Gain' incorporates:
  //   Constant: '<S7>/Constant1'
  //   Product: '<S48>/Element product'
  //   Sum: '<S48>/Add3'
  //   Sum: '<S7>/Add'

  rtb_P_B_ISS_SS[0] = ((rtb_bpm_force_B[0] - rtb_P_B_ISS_SS[0]) -
                       (sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
                        rtb_Merge_o[2] -
                        sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
                        rtb_Merge_o[1])) *
    sim_model_lib0_P->tun_env_accel_dof_gain[0];
  rtb_P_B_ISS_SS[1] = ((rtb_bpm_force_B[1] - rtb_P_B_ISS_SS[1]) -
                       (sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2] *
                        rtb_Merge_o[0] -
                        sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] *
                        rtb_Merge_o[2])) *
    sim_model_lib0_P->tun_env_accel_dof_gain[1];
  rtb_Merge_c = ((rtb_bpm_force_B[2] - rtb_P_B_ISS_SS[2]) -
                 (sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0] * rtb_Merge_o[1]
                  - sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] *
                  rtb_Merge_o[0])) * sim_model_lib0_P->tun_env_accel_dof_gain[2];

  // Sum: '<S57>/Sum' incorporates:
  //   Constant: '<S57>/Constant1'
  //   DataTypeConversion: '<S59>/Conversion'
  //   Gain: '<S57>/Gain'
  //   Math: '<S57>/Math Function'

  rtb_Sqrt_m = rtb_Merge_m[3] * rtb_Merge_m[3] * sim_model_lib0_P->Gain_Gain_b -
    (real32_T)sim_model_lib0_P->Constant1_Value_i;

  // Assignment: '<S57>/Assignment' incorporates:
  //   Constant: '<S57>/Constant2'
  //   DataTypeConversion: '<S58>/Conversion'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_m[i] = (real32_T)sim_model_lib0_P->Constant2_Value_d[i];
  }

  rtb_Assignment_m[0] = rtb_Sqrt_m;
  rtb_Assignment_m[4] = rtb_Sqrt_m;
  rtb_Assignment_m[8] = rtb_Sqrt_m;

  // End of Assignment: '<S57>/Assignment'

  // Gain: '<S57>/Gain1'
  rtb_Product_p = sim_model_lib0_P->Gain1_Gain_m * rtb_Merge_m[3];

  // Product: '<S57>/Product' incorporates:
  //   Constant: '<S60>/Constant3'
  //   DataTypeConversion: '<S61>/Conversion'
  //   Gain: '<S60>/Gain'
  //   Gain: '<S60>/Gain1'
  //   Gain: '<S60>/Gain2'

  tmp_2[0] = (real32_T)sim_model_lib0_P->Constant3_Value_a;
  tmp_2[1] = rtb_Merge_m[2];
  tmp_2[2] = sim_model_lib0_P->Gain_Gain_la * rtb_Merge_m[1];
  tmp_2[3] = sim_model_lib0_P->Gain1_Gain_hb * rtb_Merge_m[2];
  tmp_2[4] = (real32_T)sim_model_lib0_P->Constant3_Value_a;
  tmp_2[5] = rtb_Merge_m[0];
  tmp_2[6] = rtb_Merge_m[1];
  tmp_2[7] = sim_model_lib0_P->Gain2_Gain_gu * rtb_Merge_m[0];
  tmp_2[8] = (real32_T)sim_model_lib0_P->Constant3_Value_a;

  // Product: '<S57>/Product1' incorporates:
  //   Gain: '<S57>/Gain2'
  //   Math: '<S57>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Merge_b[i_0] = rtb_Merge_m[i_0] * rtb_Merge_m[0];
    rtb_Merge_b[(int32_T)(i_0 + 3)] = rtb_Merge_m[i_0] * rtb_Merge_m[1];
    rtb_Merge_b[(int32_T)(i_0 + 6)] = rtb_Merge_m[i_0] * rtb_Merge_m[2];
  }

  // End of Product: '<S57>/Product1'

  // Sum: '<S57>/Sum1' incorporates:
  //   Gain: '<S57>/Gain2'
  //   Product: '<S50>/Product'
  //   Product: '<S57>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_i[(int32_T)(3 * i_0)] = (rtb_Assignment_m[(int32_T)(3 * i_0)]
      - tmp_2[(int32_T)(3 * i_0)] * rtb_Product_p) + rtb_Merge_b[(int32_T)(3 *
      i_0)] * sim_model_lib0_P->Gain2_Gain_lk;
    rtb_Assignment_i[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_2[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_lk;
    rtb_Assignment_i[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_2[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_lk;
  }

  // End of Sum: '<S57>/Sum1'

  // Product: '<S50>/Product'
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Merge_o[i_0] = rtb_Assignment_i[(int32_T)(i_0 + 6)] * rtb_Merge_c +
      (rtb_Assignment_i[(int32_T)(i_0 + 3)] * rtb_P_B_ISS_SS[1] +
       rtb_Assignment_i[i_0] * rtb_P_B_ISS_SS[0]);
  }

  // Sum: '<S52>/Sum' incorporates:
  //   Constant: '<S52>/Constant1'
  //   DataTypeConversion: '<S54>/Conversion'
  //   Gain: '<S52>/Gain'
  //   Math: '<S52>/Math Function'

  rtb_Sqrt_m = rtb_Merge_m[3] * rtb_Merge_m[3] * sim_model_lib0_P->Gain_Gain_h -
    (real32_T)sim_model_lib0_P->Constant1_Value_c4;

  // Assignment: '<S52>/Assignment' incorporates:
  //   Constant: '<S52>/Constant2'
  //   DataTypeConversion: '<S53>/Conversion'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_m[i] = (real32_T)sim_model_lib0_P->Constant2_Value_p[i];
  }

  rtb_Assignment_m[0] = rtb_Sqrt_m;
  rtb_Assignment_m[4] = rtb_Sqrt_m;
  rtb_Assignment_m[8] = rtb_Sqrt_m;

  // End of Assignment: '<S52>/Assignment'

  // Gain: '<S52>/Gain1'
  rtb_Product_p = sim_model_lib0_P->Gain1_Gain_al * rtb_Merge_m[3];

  // Product: '<S52>/Product' incorporates:
  //   Constant: '<S55>/Constant3'
  //   DataTypeConversion: '<S56>/Conversion'
  //   Gain: '<S55>/Gain'
  //   Gain: '<S55>/Gain1'
  //   Gain: '<S55>/Gain2'

  tmp_3[0] = (real32_T)sim_model_lib0_P->Constant3_Value_b;
  tmp_3[1] = rtb_Merge_m[2];
  tmp_3[2] = sim_model_lib0_P->Gain_Gain_kk * rtb_Merge_m[1];
  tmp_3[3] = sim_model_lib0_P->Gain1_Gain_aj * rtb_Merge_m[2];
  tmp_3[4] = (real32_T)sim_model_lib0_P->Constant3_Value_b;
  tmp_3[5] = rtb_Merge_m[0];
  tmp_3[6] = rtb_Merge_m[1];
  tmp_3[7] = sim_model_lib0_P->Gain2_Gain_m * rtb_Merge_m[0];
  tmp_3[8] = (real32_T)sim_model_lib0_P->Constant3_Value_b;

  // Product: '<S52>/Product1' incorporates:
  //   Gain: '<S52>/Gain2'
  //   Math: '<S52>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Merge_b[i_0] = rtb_Merge_m[i_0] * rtb_Merge_m[0];
    rtb_Merge_b[(int32_T)(i_0 + 3)] = rtb_Merge_m[i_0] * rtb_Merge_m[1];
    rtb_Merge_b[(int32_T)(i_0 + 6)] = rtb_Merge_m[i_0] * rtb_Merge_m[2];
  }

  // End of Product: '<S52>/Product1'

  // Sum: '<S52>/Sum1' incorporates:
  //   Gain: '<S52>/Gain2'
  //   Product: '<S49>/Product'
  //   Product: '<S52>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_i[(int32_T)(3 * i_0)] = (rtb_Assignment_m[(int32_T)(3 * i_0)]
      - tmp_3[(int32_T)(3 * i_0)] * rtb_Product_p) + rtb_Merge_b[(int32_T)(3 *
      i_0)] * sim_model_lib0_P->Gain2_Gain_h4;
    rtb_Assignment_i[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_3[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_h4;
    rtb_Assignment_i[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_3[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_h4;
  }

  // End of Sum: '<S52>/Sum1'

  // Product: '<S49>/Product'
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_imu_gyro_bias[i_0] = rtb_Assignment_i[(int32_T)(i_0 + 6)] *
      rtb_bpm_force_B[2] + (rtb_Assignment_i[(int32_T)(i_0 + 3)] *
      rtb_bpm_force_B[1] + rtb_Assignment_i[i_0] * rtb_bpm_force_B[0]);
  }

  // Sum: '<S35>/Sum' incorporates:
  //   Constant: '<S35>/Constant1'
  //   DataTypeConversion: '<S37>/Conversion'
  //   Gain: '<S35>/Gain'
  //   Math: '<S35>/Math Function'

  rtb_Sqrt_m = rtb_Merge_m[3] * rtb_Merge_m[3] * sim_model_lib0_P->Gain_Gain_c -
    (real32_T)sim_model_lib0_P->Constant1_Value_o5;

  // Assignment: '<S35>/Assignment' incorporates:
  //   Constant: '<S35>/Constant2'
  //   DataTypeConversion: '<S36>/Conversion'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_m[i] = (real32_T)sim_model_lib0_P->Constant2_Value_c[i];
  }

  rtb_Assignment_m[0] = rtb_Sqrt_m;
  rtb_Assignment_m[4] = rtb_Sqrt_m;
  rtb_Assignment_m[8] = rtb_Sqrt_m;

  // End of Assignment: '<S35>/Assignment'

  // Gain: '<S35>/Gain1'
  rtb_Product_p = sim_model_lib0_P->Gain1_Gain_de * rtb_Merge_m[3];

  // Product: '<S35>/Product' incorporates:
  //   Constant: '<S38>/Constant3'
  //   DataTypeConversion: '<S39>/Conversion'
  //   Gain: '<S38>/Gain'
  //   Gain: '<S38>/Gain1'
  //   Gain: '<S38>/Gain2'

  tmp_4[0] = (real32_T)sim_model_lib0_P->Constant3_Value_j;
  tmp_4[1] = rtb_Merge_m[2];
  tmp_4[2] = sim_model_lib0_P->Gain_Gain_g5 * rtb_Merge_m[1];
  tmp_4[3] = sim_model_lib0_P->Gain1_Gain_k * rtb_Merge_m[2];
  tmp_4[4] = (real32_T)sim_model_lib0_P->Constant3_Value_j;
  tmp_4[5] = rtb_Merge_m[0];
  tmp_4[6] = rtb_Merge_m[1];
  tmp_4[7] = sim_model_lib0_P->Gain2_Gain_ku * rtb_Merge_m[0];
  tmp_4[8] = (real32_T)sim_model_lib0_P->Constant3_Value_j;

  // Product: '<S35>/Product1' incorporates:
  //   Gain: '<S35>/Gain2'
  //   Math: '<S35>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Merge_b[i_0] = rtb_Merge_m[i_0] * rtb_Merge_m[0];
    rtb_Merge_b[(int32_T)(i_0 + 3)] = rtb_Merge_m[i_0] * rtb_Merge_m[1];
    rtb_Merge_b[(int32_T)(i_0 + 6)] = rtb_Merge_m[i_0] * rtb_Merge_m[2];
  }

  // End of Product: '<S35>/Product1'

  // Sum: '<S35>/Sum1' incorporates:
  //   Gain: '<S35>/Gain2'
  //   Product: '<S12>/Product'
  //   Product: '<S35>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_i[(int32_T)(3 * i_0)] = (rtb_Assignment_m[(int32_T)(3 * i_0)]
      - tmp_4[(int32_T)(3 * i_0)] * rtb_Product_p) + rtb_Merge_b[(int32_T)(3 *
      i_0)] * sim_model_lib0_P->Gain2_Gain_ax;
    rtb_Assignment_i[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_4[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_ax;
    rtb_Assignment_i[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment_m
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_4[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Product_p) + rtb_Merge_b[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_ax;
  }

  // End of Sum: '<S35>/Sum1'

  // Sum: '<S5>/Sum1' incorporates:
  //   Constant: '<S5>/Constant4'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   Product: '<S12>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_bpm_force_B[i_0] = ((rtb_Assignment_i[(int32_T)(i_0 + 3)] *
      sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[1] + rtb_Assignment_i[i_0] *
      sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[0]) + rtb_Assignment_i
      [(int32_T)(i_0 + 6)] * sim_model_lib0_P->tun_iss_omega_ISS_ECI_ISS[2]) +
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[i_0];
  }

  // End of Sum: '<S5>/Sum1'

  // Sum: '<S2>/Sum1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant10'
  //   Constant: '<S2>/Constant1'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_m[i] = sim_model_lib0_P->tun_default_inertia_matrix[i] +
      sim_model_lib0_P->tun_inertia_error_mat[i];
  }

  // End of Sum: '<S2>/Sum1'

  // Switch: '<S6>/Switch2' incorporates:
  //   Constant: '<S6>/Constant10'
  //   Constant: '<S6>/Constant11'
  //   Constant: '<S6>/Constant8'
  //   Gain: '<S6>/Gain2'
  //   Product: '<S6>/Divide3'
  //   Sum: '<S6>/Sum2'

  if ((int32_T)sim_model_lib0_P->tun_env_drag_disturb_on != 0) {
    // Sum: '<S6>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'

    rtb_Sqrt_m = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0] -
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0];

    // Product: '<S6>/Divide3'
    if (rtb_Sqrt_m < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Sqrt_m > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Sqrt_m == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Sqrt_m;
    }

    rtb_Sqrt_m = (real32_T)((real_T)(sim_model_lib0_P->Gain2_Gain_f *
      rtb_Product_p * rtb_Sqrt_m * rtb_Sqrt_m) *
      sim_model_lib0_P->env_rotational_drag_coeff);
    rtb_Sum2_f[0] = rtb_Sqrt_m;

    // Sum: '<S6>/Sum2' incorporates:
    //   Constant: '<S6>/Constant11'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
    //   Gain: '<S6>/Gain2'
    //   Product: '<S6>/Divide3'

    rtb_Sqrt_m = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1] -
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1];

    // Product: '<S6>/Divide3'
    if (rtb_Sqrt_m < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Sqrt_m > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Sqrt_m == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Sqrt_m;
    }

    rtb_Sqrt_m = (real32_T)((real_T)(sim_model_lib0_P->Gain2_Gain_f *
      rtb_Product_p * rtb_Sqrt_m * rtb_Sqrt_m) *
      sim_model_lib0_P->env_rotational_drag_coeff);
    rtb_Sum2_f[1] = rtb_Sqrt_m;

    // Sum: '<S6>/Sum2' incorporates:
    //   Constant: '<S6>/Constant11'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
    //   Gain: '<S6>/Gain2'
    //   Product: '<S6>/Divide3'

    rtb_Sqrt_m = sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2] -
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2];

    // Product: '<S6>/Divide3'
    if (rtb_Sqrt_m < 0.0F) {
      rtb_Product_p = -1.0F;
    } else if (rtb_Sqrt_m > 0.0F) {
      rtb_Product_p = 1.0F;
    } else if (rtb_Sqrt_m == 0.0F) {
      rtb_Product_p = 0.0F;
    } else {
      rtb_Product_p = rtb_Sqrt_m;
    }

    rtb_Sqrt_m = (real32_T)((real_T)(sim_model_lib0_P->Gain2_Gain_f *
      rtb_Product_p * rtb_Sqrt_m * rtb_Sqrt_m) *
      sim_model_lib0_P->env_rotational_drag_coeff);
    rtb_Sum2_f[2] = rtb_Sqrt_m;
  } else {
    rtb_Sum2_f[0] = sim_model_lib0_P->Constant10_Value_g[0];
    rtb_Sum2_f[1] = sim_model_lib0_P->Constant10_Value_g[1];
    rtb_Sum2_f[2] = sim_model_lib0_P->Constant10_Value_g[2];
  }

  // End of Switch: '<S6>/Switch2'

  // Product: '<S5>/Product2'
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_alpha_B_ECI_B[i_0] = rtb_Assignment_m[(int32_T)(i_0 + 6)] *
      rtb_bpm_force_B[2] + (rtb_Assignment_m[(int32_T)(i_0 + 3)] *
      rtb_bpm_force_B[1] + rtb_Assignment_m[i_0] * rtb_bpm_force_B[0]);
  }

  // End of Product: '<S5>/Product2'

  // SignalConversion: '<S9>/TmpSignal ConversionAtProductInport1' incorporates:
  //   Constant: '<S13>/Constant3'
  //   DataTypeConversion: '<S14>/Conversion'
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain1'
  //   Gain: '<S13>/Gain2'
  //   Product: '<S9>/Product'

  tmp_5[0] = (real32_T)sim_model_lib0_P->Constant3_Value_e;
  tmp_5[1] = rtb_alpha_B_ECI_B[2];
  tmp_5[2] = sim_model_lib0_P->Gain_Gain_hs * rtb_alpha_B_ECI_B[1];
  tmp_5[3] = sim_model_lib0_P->Gain1_Gain_ep * rtb_alpha_B_ECI_B[2];
  tmp_5[4] = (real32_T)sim_model_lib0_P->Constant3_Value_e;
  tmp_5[5] = rtb_alpha_B_ECI_B[0];
  tmp_5[6] = rtb_alpha_B_ECI_B[1];
  tmp_5[7] = sim_model_lib0_P->Gain2_Gain_c * rtb_alpha_B_ECI_B[0];
  tmp_5[8] = (real32_T)sim_model_lib0_P->Constant3_Value_e;

  // Sum: '<S5>/Sum' incorporates:
  //   Product: '<S9>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Sum2_f[i_0] = ((tmp_5[(int32_T)(i_0 + 3)] * rtb_bpm_force_B[1] +
                        tmp_5[i_0] * rtb_bpm_force_B[0]) + tmp_5[(int32_T)(i_0 +
      6)] * rtb_bpm_force_B[2]) + (rtb_Delay1_bpm_torque_B[i_0] + rtb_Sum2_f[i_0]);
  }

  // End of Sum: '<S5>/Sum'

  // Product: '<S5>/Divide'
  rt_mldivide_U1f3x3_U2f_XeZWzB4d(rtb_Assignment_m, rtb_Sum2_f, tmp);

  // Gain: '<S5>/Gain'
  rtb_alpha_B_ECI_B[0] = sim_model_lib0_P->tun_env_alpha_dof_gain[0] * tmp[0];

  // BusCreator: '<S2>/Bus Creator'
  rtb_Sum2_f[0] = rtb_bpm_force_B[0];

  // Sum: '<S257>/Subtract' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'
  //   Constant: '<S257>/ 1'
  //   Constant: '<S257>/ 2'
  //   DataTypeConversion: '<S186>/Data Type Conversion6'
  //   Gain: '<S257>/Gain'

  rtb_P_sensor_CG_B[0] = ((real_T)sim_model_lib0_P->tun_epson_report_truth *
    sim_model_lib0_P->epson_P_sensor_B_B_error[0] + (real_T)
    sim_model_lib0_P->tun_abp_p_imu_body_body[0]) - (real_T)
    sim_model_lib0_P->tun_default_center_of_mass[0];

  // Gain: '<S5>/Gain'
  rtb_alpha_B_ECI_B[1] = sim_model_lib0_P->tun_env_alpha_dof_gain[1] * tmp[1];

  // BusCreator: '<S2>/Bus Creator'
  rtb_Sum2_f[1] = rtb_bpm_force_B[1];

  // Sum: '<S257>/Subtract' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'
  //   Constant: '<S257>/ 1'
  //   Constant: '<S257>/ 2'
  //   DataTypeConversion: '<S186>/Data Type Conversion6'
  //   Gain: '<S257>/Gain'

  rtb_P_sensor_CG_B[1] = ((real_T)sim_model_lib0_P->tun_epson_report_truth *
    sim_model_lib0_P->epson_P_sensor_B_B_error[1] + (real_T)
    sim_model_lib0_P->tun_abp_p_imu_body_body[1]) - (real_T)
    sim_model_lib0_P->tun_default_center_of_mass[1];

  // Gain: '<S5>/Gain'
  rtb_alpha_B_ECI_B[2] = sim_model_lib0_P->tun_env_alpha_dof_gain[2] * tmp[2];

  // BusCreator: '<S2>/Bus Creator'
  rtb_Sum2_f[2] = rtb_bpm_force_B[2];

  // Sum: '<S257>/Subtract' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'
  //   Constant: '<S257>/ 1'
  //   Constant: '<S257>/ 2'
  //   DataTypeConversion: '<S186>/Data Type Conversion6'
  //   Gain: '<S257>/Gain'

  rtb_P_sensor_CG_B[2] = ((real_T)sim_model_lib0_P->tun_epson_report_truth *
    sim_model_lib0_P->epson_P_sensor_B_B_error[2] + (real_T)
    sim_model_lib0_P->tun_abp_p_imu_body_body[2]) - (real_T)
    sim_model_lib0_P->tun_default_center_of_mass[2];

  // Switch: '<S257>/Switch1' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   Constant: '<S257>/Constant4'
  //   Constant: '<S257>/Constant5'
  //   DataTypeConversion: '<S186>/Data Type Conversion5'
  //   Product: '<S259>/Element product'
  //   Sum: '<S257>/Add1'
  //   Sum: '<S259>/Add3'

  if (sim_model_lib0_P->epson_no_rot_effects != 0.0) {
    rtb_Add3[0] = sim_model_lib0_P->Constant5_Value;
    rtb_Add3[1] = sim_model_lib0_P->Constant5_Value;
    rtb_Add3[2] = sim_model_lib0_P->Constant5_Value;
  } else {
    // Sum: '<S260>/Add3' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   DataTypeConversion: '<S186>/Data Type Conversion3'
    //   Product: '<S260>/Element product'

    rtb_Add3[0] = (real_T)rtb_alpha_B_ECI_B[1] * rtb_P_sensor_CG_B[2] - (real_T)
      rtb_alpha_B_ECI_B[2] * rtb_P_sensor_CG_B[1];
    rtb_Add3[1] = (real_T)rtb_alpha_B_ECI_B[2] * rtb_P_sensor_CG_B[0] - (real_T)
      rtb_alpha_B_ECI_B[0] * rtb_P_sensor_CG_B[2];
    rtb_Add3[2] = (real_T)rtb_alpha_B_ECI_B[0] * rtb_P_sensor_CG_B[1] - (real_T)
      rtb_alpha_B_ECI_B[1] * rtb_P_sensor_CG_B[0];

    // Sum: '<S258>/Add3' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   DataTypeConversion: '<S186>/Data Type Conversion5'
    //   Product: '<S258>/Element product'

    rtb_Add3_a[0] = (real_T)rtb_bpm_force_B[1] * rtb_P_sensor_CG_B[2] - (real_T)
      rtb_bpm_force_B[2] * rtb_P_sensor_CG_B[1];
    rtb_Add3_a[1] = (real_T)rtb_bpm_force_B[2] * rtb_P_sensor_CG_B[0] - (real_T)
      rtb_bpm_force_B[0] * rtb_P_sensor_CG_B[2];
    rtb_Add3_a[2] = (real_T)rtb_bpm_force_B[0] * rtb_P_sensor_CG_B[1] - (real_T)
      rtb_bpm_force_B[1] * rtb_P_sensor_CG_B[0];
    rtb_Add3[0] += (real_T)rtb_bpm_force_B[1] * rtb_Add3_a[2] - (real_T)
      rtb_bpm_force_B[2] * rtb_Add3_a[1];
    rtb_Add3[1] += (real_T)rtb_bpm_force_B[2] * rtb_Add3_a[0] - (real_T)
      rtb_bpm_force_B[0] * rtb_Add3_a[2];
    rtb_Add3[2] += (real_T)rtb_bpm_force_B[0] * rtb_Add3_a[1] - (real_T)
      rtb_bpm_force_B[1] * rtb_Add3_a[0];
  }

  // End of Switch: '<S257>/Switch1'

  // DataTypeConversion: '<S186>/Data Type Conversion2' incorporates:
  //   BusCreator: '<S2>/Bus Creator'

  rtb_Product_l[0] = (real_T)rtb_Merge_m[0];
  rtb_Product_l[1] = (real_T)rtb_Merge_m[1];
  rtb_Product_l[2] = (real_T)rtb_Merge_m[2];
  rtb_Product_l[3] = (real_T)rtb_Merge_m[3];

  // Sum: '<S279>/Sum' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   Constant: '<S279>/Constant1'
  //   DataTypeConversion: '<S186>/Data Type Conversion2'
  //   Gain: '<S279>/Gain'
  //   Math: '<S279>/Math Function'

  rtb_Sum = (real_T)rtb_Merge_m[3] * (real_T)rtb_Merge_m[3] *
    sim_model_lib0_P->Gain_Gain_e - sim_model_lib0_P->Constant1_Value_b;

  // Assignment: '<S279>/Assignment' incorporates:
  //   Constant: '<S279>/Constant2'

  memcpy(&rtb_Assignment[0], &sim_model_lib0_P->Constant2_Value_cr[0], (uint32_T)
         (9U * sizeof(real_T)));
  rtb_Assignment[0] = rtb_Sum;
  rtb_Assignment[4] = rtb_Sum;
  rtb_Assignment[8] = rtb_Sum;

  // Gain: '<S279>/Gain1' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DataTypeConversion: '<S186>/Data Type Conversion2'

  rtb_Sum = sim_model_lib0_P->Gain1_Gain_o * (real_T)rtb_Merge_m[3];

  // Product: '<S279>/Product' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   Constant: '<S282>/Constant3'
  //   DataTypeConversion: '<S186>/Data Type Conversion2'
  //   Gain: '<S282>/Gain'
  //   Gain: '<S282>/Gain1'
  //   Gain: '<S282>/Gain2'

  rtb_Assignment_c[0] = sim_model_lib0_P->Constant3_Value_d;
  rtb_Assignment_c[1] = (real_T)rtb_Merge_m[2];
  rtb_Assignment_c[2] = sim_model_lib0_P->Gain_Gain_l * (real_T)rtb_Merge_m[1];
  rtb_Assignment_c[3] = sim_model_lib0_P->Gain1_Gain_oh * (real_T)rtb_Merge_m[2];
  rtb_Assignment_c[4] = sim_model_lib0_P->Constant3_Value_d;
  rtb_Assignment_c[5] = (real_T)rtb_Merge_m[0];
  rtb_Assignment_c[6] = (real_T)rtb_Merge_m[1];
  rtb_Assignment_c[7] = sim_model_lib0_P->Gain2_Gain_o * (real_T)rtb_Merge_m[0];
  rtb_Assignment_c[8] = sim_model_lib0_P->Constant3_Value_d;

  // Product: '<S279>/Product1' incorporates:
  //   Gain: '<S279>/Gain2'
  //   Math: '<S279>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_p_0[i_0] = rtb_Product_l[i_0] * rtb_Product_l[0];
    rtb_Product_p_0[(int32_T)(i_0 + 3)] = rtb_Product_l[i_0] * rtb_Product_l[1];
    rtb_Product_p_0[(int32_T)(i_0 + 6)] = rtb_Product_l[i_0] * rtb_Product_l[2];
  }

  // End of Product: '<S279>/Product1'

  // Switch: '<S257>/Switch' incorporates:
  //   Constant: '<S257>/Constant6'

  tmp_0 = (sim_model_lib0_P->tun_ase_gravity_removal >
           sim_model_lib0_P->Switch_Threshold_p);
  for (i_0 = 0; i_0 < 3; i_0++) {
    // Sum: '<S279>/Sum1' incorporates:
    //   Gain: '<S279>/Gain2'
    //   Product: '<S264>/Product'
    //   Product: '<S279>/Product'

    rtb_Assignment_3[(int32_T)(3 * i_0)] = (rtb_Assignment[(int32_T)(3 * i_0)] -
      rtb_Assignment_c[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_p_0[(int32_T)
      (3 * i_0)] * sim_model_lib0_P->Gain2_Gain_ar;
    rtb_Assignment_3[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 1)] - rtb_Assignment_c[(int32_T)((int32_T)
      (3 * i_0) + 1)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0)
      + 1)] * sim_model_lib0_P->Gain2_Gain_ar;
    rtb_Assignment_3[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 2)] - rtb_Assignment_c[(int32_T)((int32_T)
      (3 * i_0) + 2)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0)
      + 2)] * sim_model_lib0_P->Gain2_Gain_ar;

    // Sum: '<S257>/Subtract1' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   Constant: '<S186>/Constant4'
    //   Constant: '<S257>/Constant7'
    //   DataTypeConversion: '<S186>/Data Type Conversion4'
    //   DataTypeConversion: '<S186>/Data Type Conversion9'
    //   Product: '<S264>/Product'
    //   Switch: '<S257>/Switch'

    if (tmp_0) {
      rtb_Product = (real_T)sim_model_lib0_P->tun_ase_gravity_accel[i_0];
    } else {
      rtb_Product = (real_T)sim_model_lib0_P->Constant7_Value_j[i_0];
    }

    rtb_Add3_a[i_0] = (real_T)rtb_imu_gyro_bias[i_0] + rtb_Product;

    // End of Sum: '<S257>/Subtract1'
  }

  // Sum: '<S257>/Add2' incorporates:
  //   Product: '<S264>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_P_sensor_CG_B[i_0] = ((rtb_Assignment_3[(int32_T)(i_0 + 3)] *
      rtb_Add3_a[1] + rtb_Assignment_3[i_0] * rtb_Add3_a[0]) + rtb_Assignment_3
      [(int32_T)(i_0 + 6)] * rtb_Add3_a[2]) + rtb_Add3[i_0];
  }

  // End of Sum: '<S257>/Add2'

  // Sum: '<S254>/Subtract' incorporates:
  //   Constant: '<S186>/Constant'
  //   Constant: '<S254>/Constant'
  //   DataTypeConversion: '<S186>/Data Type Conversion7'

  rtb_Sum = (real_T)sim_model_lib0_P->Constant_Value_g -
    sim_model_lib0_P->Constant_Value;
  for (i = 0; i < 3; i++) {
    // RandomNumber: '<S254>/random_noise'
    rtb_random_noise[i] = sim_model_lib0_DW->NextOutput[i];

    // Sum: '<S254>/Sum6' incorporates:
    //   Constant: '<S254>/Constant2'
    //   DiscreteIntegrator: '<S254>/Discrete-Time Integrator1'
    //   DiscreteIntegrator: '<S254>/Discrete-Time Integrator2'
    //   Product: '<S254>/Divide4'

    rtb_Add3_a[i] = (rtb_Sum * sim_model_lib0_P->epson_accel_temp_bias_coeff[i]
                     + sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[i]) +
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[i];

    // Product: '<S265>/Product'
    rtb_Add3[i] = rtb_Assignment_jy[(int32_T)(i + 6)] * rtb_P_sensor_CG_B[2] +
      (rtb_Assignment_jy[(int32_T)(i + 3)] * rtb_P_sensor_CG_B[1] +
       rtb_Assignment_jy[i] * rtb_P_sensor_CG_B[0]);
  }

  // Switch: '<S254>/Switch' incorporates:
  //   Constant: '<S254>/Constant1'
  //   Constant: '<S254>/Constant11'
  //   Constant: '<S254>/Constant3'
  //   Constant: '<S254>/Constant6'
  //   Gain: '<S254>/Gain4'
  //   Gain: '<S254>/scale_factor'
  //   Product: '<S254>/Divide'
  //   Product: '<S254>/Product2'
  //   Sum: '<S254>/Sum2'

  if ((int32_T)sim_model_lib0_P->tun_epson_report_truth == 0) {
    // Gain: '<S254>/Gain4'
    rtb_Sum = 1.0 / sqrt(sim_model_lib0_P->astrobee_time_step_size);

    // Gain: '<S254>/scale_factor'
    rtb_Product = sim_model_lib0_P->epson_accel_sf_coef * rtb_Add3[0];
    rtb_Product = ((sin(rtb_Product * sim_model_lib0_P->Constant1_Value_p /
                        sim_model_lib0_P->epson_accel_upper_sat) *
                    sim_model_lib0_P->epson_accel_nonlinearity_coeff +
                    rtb_Product) + rtb_Sum * rtb_random_noise[0]) + rtb_Add3_a[0];
    rtb_Add3[0] = rtb_Product;

    // Gain: '<S254>/scale_factor' incorporates:
    //   Constant: '<S254>/Constant1'
    //   Constant: '<S254>/Constant11'
    //   Constant: '<S254>/Constant6'
    //   Gain: '<S254>/Gain4'
    //   Product: '<S254>/Divide'
    //   Product: '<S254>/Product2'
    //   Sum: '<S254>/Sum2'

    rtb_Product = sim_model_lib0_P->epson_accel_sf_coef * rtb_Add3[1];
    rtb_Product = ((sin(rtb_Product * sim_model_lib0_P->Constant1_Value_p /
                        sim_model_lib0_P->epson_accel_upper_sat) *
                    sim_model_lib0_P->epson_accel_nonlinearity_coeff +
                    rtb_Product) + rtb_Sum * rtb_random_noise[1]) + rtb_Add3_a[1];
    rtb_Add3[1] = rtb_Product;

    // Gain: '<S254>/scale_factor' incorporates:
    //   Constant: '<S254>/Constant1'
    //   Constant: '<S254>/Constant11'
    //   Constant: '<S254>/Constant6'
    //   Gain: '<S254>/Gain4'
    //   Product: '<S254>/Divide'
    //   Product: '<S254>/Product2'
    //   Sum: '<S254>/Sum2'

    rtb_Product = sim_model_lib0_P->epson_accel_sf_coef * rtb_Add3[2];
    rtb_Product = ((sin(rtb_Product * sim_model_lib0_P->Constant1_Value_p /
                        sim_model_lib0_P->epson_accel_upper_sat) *
                    sim_model_lib0_P->epson_accel_nonlinearity_coeff +
                    rtb_Product) + rtb_Sum * rtb_random_noise[2]) + rtb_Add3_a[2];
    rtb_Add3[2] = rtb_Product;
  }

  // End of Switch: '<S254>/Switch'

  // Quantizer: '<S256>/Quantizer' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn1'

  rtb_Product = rt_roundd_snf(rtb_Add3[0] *
    sim_model_lib0_P->epson_accel_filt_num /
    sim_model_lib0_P->epson_accel_filt_den /
    sim_model_lib0_P->epson_accel_resolution) *
    sim_model_lib0_P->epson_accel_resolution;

  // Saturate: '<S256>/Saturation'
  if (rtb_Product > sim_model_lib0_P->epson_accel_upper_sat) {
    y = sim_model_lib0_P->epson_accel_upper_sat;
  } else if (rtb_Product < sim_model_lib0_P->epson_accel_lower_sat) {
    y = sim_model_lib0_P->epson_accel_lower_sat;
  } else {
    y = rtb_Product;
  }

  // Quantizer: '<S256>/Quantizer' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn1'

  rtb_Product = rt_roundd_snf(rtb_Add3[1] *
    sim_model_lib0_P->epson_accel_filt_num /
    sim_model_lib0_P->epson_accel_filt_den /
    sim_model_lib0_P->epson_accel_resolution) *
    sim_model_lib0_P->epson_accel_resolution;

  // Saturate: '<S256>/Saturation'
  if (rtb_Product > sim_model_lib0_P->epson_accel_upper_sat) {
    y_0 = sim_model_lib0_P->epson_accel_upper_sat;
  } else if (rtb_Product < sim_model_lib0_P->epson_accel_lower_sat) {
    y_0 = sim_model_lib0_P->epson_accel_lower_sat;
  } else {
    y_0 = rtb_Product;
  }

  // Quantizer: '<S256>/Quantizer' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn1'

  rtb_Product = rt_roundd_snf(rtb_Add3[2] *
    sim_model_lib0_P->epson_accel_filt_num /
    sim_model_lib0_P->epson_accel_filt_den /
    sim_model_lib0_P->epson_accel_resolution) *
    sim_model_lib0_P->epson_accel_resolution;

  // Assignment: '<S266>/Assignment' incorporates:
  //   Constant: '<S257>/Constant2'
  //   Constant: '<S266>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'

  memcpy(&rtb_Assignment_jy[0], &sim_model_lib0_P->Constant2_Value_n[0],
         (uint32_T)(9U * sizeof(real_T)));
  rtb_Assignment_jy[0] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];
  rtb_Assignment_jy[4] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];
  rtb_Assignment_jy[8] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];

  // Sum: '<S266>/Sum2' incorporates:
  //   Constant: '<S257>/Constant2'
  //   Constant: '<S268>/Constant3'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'
  //   Gain: '<S268>/Gain'
  //   Gain: '<S268>/Gain1'
  //   Gain: '<S268>/Gain2'

  tmp_6[0] = sim_model_lib0_P->Constant3_Value_i;
  tmp_6[1] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[2];
  tmp_6[2] = sim_model_lib0_P->Gain_Gain_d * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[1];
  tmp_6[3] = sim_model_lib0_P->Gain1_Gain_j * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[2];
  tmp_6[4] = sim_model_lib0_P->Constant3_Value_i;
  tmp_6[5] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[0];
  tmp_6[6] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[1];
  tmp_6[7] = sim_model_lib0_P->Gain2_Gain_k * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[0];
  tmp_6[8] = sim_model_lib0_P->Constant3_Value_i;

  // Concatenate: '<S266>/Matrix Concatenate' incorporates:
  //   Constant: '<S257>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'
  //   Gain: '<S266>/Gain1'
  //   Sum: '<S266>/Sum2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_MatrixConcatenate[(int32_T)(i_0 << 2)] = rtb_Assignment_jy[(int32_T)(3 *
      i_0)] + tmp_6[(int32_T)(3 * i_0)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(i_0 << 2))] =
      rtb_Assignment_jy[(int32_T)((int32_T)(3 * i_0) + 1)] + tmp_6[(int32_T)
      ((int32_T)(3 * i_0) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(i_0 << 2))] =
      rtb_Assignment_jy[(int32_T)((int32_T)(3 * i_0) + 2)] + tmp_6[(int32_T)
      ((int32_T)(3 * i_0) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_h * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_h * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_h * (real_T)
    sim_model_lib0_P->tun_abp_quat_body2imu[2];

  // End of Concatenate: '<S266>/Matrix Concatenate'

  // Switch: '<S257>/Switch4' incorporates:
  //   Constant: '<S257>/Constant11'

  tmp_0 = ((int32_T)sim_model_lib0_P->tun_epson_report_truth != 0);

  // Reshape: '<S261>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'

  rtb_MatrixConcatenate[12] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[0];

  // Product: '<S261>/Product' incorporates:
  //   Constant: '<S257>/Constant10'
  //   Constant: '<S257>/Constant9'
  //   Switch: '<S257>/Switch4'

  if (tmp_0) {
    rtb_Sum = sim_model_lib0_P->Constant10_Value[0];
    rtb_DataTypeConversion5_idx_0 = sim_model_lib0_P->Constant10_Value[1];
  } else {
    rtb_Sum = sim_model_lib0_P->epson_Q_B2gyro_error[0];
    rtb_DataTypeConversion5_idx_0 = sim_model_lib0_P->epson_Q_B2gyro_error[1];
  }

  // Reshape: '<S261>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'

  rtb_MatrixConcatenate[13] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[1];
  rtb_MatrixConcatenate[14] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[2];

  // Product: '<S261>/Product' incorporates:
  //   Constant: '<S257>/Constant10'
  //   Constant: '<S257>/Constant9'
  //   Switch: '<S257>/Switch4'

  if (tmp_0) {
    rtb_DataTypeConversion5_idx_1 = sim_model_lib0_P->Constant10_Value[2];
    rtb_DataTypeConversion5_idx_2 = sim_model_lib0_P->Constant10_Value[3];
  } else {
    rtb_DataTypeConversion5_idx_1 = sim_model_lib0_P->epson_Q_B2gyro_error[2];
    rtb_DataTypeConversion5_idx_2 = sim_model_lib0_P->epson_Q_B2gyro_error[3];
  }

  // Reshape: '<S261>/Reshape1' incorporates:
  //   Constant: '<S257>/Constant2'
  //   DataTypeConversion: '<S257>/Data Type Conversion2'

  rtb_MatrixConcatenate[15] = (real_T)sim_model_lib0_P->tun_abp_quat_body2imu[3];

  // Product: '<S261>/Product'
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_linearaccelbias = rtb_MatrixConcatenate[(int32_T)(i_0 + 12)] *
      rtb_DataTypeConversion5_idx_2 + (rtb_MatrixConcatenate[(int32_T)(i_0 + 8)]
      * rtb_DataTypeConversion5_idx_1 + (rtb_MatrixConcatenate[(int32_T)(i_0 + 4)]
      * rtb_DataTypeConversion5_idx_0 + rtb_MatrixConcatenate[i_0] * rtb_Sum));
    rtb_Product_l[i_0] = rtb_linearaccelbias;
  }

  // Sum: '<S274>/Sum' incorporates:
  //   Constant: '<S274>/Constant1'
  //   Gain: '<S274>/Gain'
  //   Math: '<S274>/Math Function'

  rtb_Sum = rtb_Product_l[3] * rtb_Product_l[3] * sim_model_lib0_P->Gain_Gain_ee
    - sim_model_lib0_P->Constant1_Value_k;

  // Assignment: '<S274>/Assignment' incorporates:
  //   Constant: '<S274>/Constant2'

  memcpy(&rtb_Assignment_jy[0], &sim_model_lib0_P->Constant2_Value_dr[0],
         (uint32_T)(9U * sizeof(real_T)));
  rtb_Assignment_jy[0] = rtb_Sum;
  rtb_Assignment_jy[4] = rtb_Sum;
  rtb_Assignment_jy[8] = rtb_Sum;

  // Gain: '<S274>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_dy * rtb_Product_l[3];

  // Product: '<S274>/Product' incorporates:
  //   Constant: '<S277>/Constant3'
  //   Gain: '<S277>/Gain'
  //   Gain: '<S277>/Gain1'
  //   Gain: '<S277>/Gain2'

  tmp_7[0] = sim_model_lib0_P->Constant3_Value_iy;
  tmp_7[1] = rtb_Product_l[2];
  tmp_7[2] = sim_model_lib0_P->Gain_Gain_k * rtb_Product_l[1];
  tmp_7[3] = sim_model_lib0_P->Gain1_Gain_p * rtb_Product_l[2];
  tmp_7[4] = sim_model_lib0_P->Constant3_Value_iy;
  tmp_7[5] = rtb_Product_l[0];
  tmp_7[6] = rtb_Product_l[1];
  tmp_7[7] = sim_model_lib0_P->Gain2_Gain_dz * rtb_Product_l[0];
  tmp_7[8] = sim_model_lib0_P->Constant3_Value_iy;

  // Product: '<S274>/Product1' incorporates:
  //   Gain: '<S274>/Gain2'
  //   Math: '<S274>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_p_0[i_0] = rtb_Product_l[i_0] * rtb_Product_l[0];
    rtb_Product_p_0[(int32_T)(i_0 + 3)] = rtb_Product_l[i_0] * rtb_Product_l[1];
    rtb_Product_p_0[(int32_T)(i_0 + 6)] = rtb_Product_l[i_0] * rtb_Product_l[2];
  }

  // End of Product: '<S274>/Product1'

  // Sum: '<S274>/Sum1' incorporates:
  //   Gain: '<S274>/Gain2'
  //   Product: '<S263>/Product'
  //   Product: '<S274>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_c[(int32_T)(3 * i_0)] = (rtb_Assignment_jy[(int32_T)(3 * i_0)]
      - tmp_7[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_p_0[(int32_T)(3 * i_0)]
      * sim_model_lib0_P->Gain2_Gain_g;
    rtb_Assignment_c[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment_jy
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_7[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_g;
    rtb_Assignment_c[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment_jy
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_7[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum) + rtb_Product_p_0[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_g;
  }

  // End of Sum: '<S274>/Sum1'

  // Sum: '<S255>/Subtract1' incorporates:
  //   Constant: '<S186>/Constant'
  //   Constant: '<S255>/Constant4'
  //   DataTypeConversion: '<S186>/Data Type Conversion7'

  rtb_Sum = (real_T)sim_model_lib0_P->Constant_Value_g -
    sim_model_lib0_P->Constant4_Value;
  for (i = 0; i < 3; i++) {
    // RandomNumber: '<S255>/random_noise'
    rtb_random_noise[i] = sim_model_lib0_DW->NextOutput_e[i];

    // Product: '<S263>/Product' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   DataTypeConversion: '<S186>/Data Type Conversion5'

    rtb_Add3[i] = rtb_Assignment_c[(int32_T)(i + 6)] * (real_T)rtb_bpm_force_B[2]
      + (rtb_Assignment_c[(int32_T)(i + 3)] * (real_T)rtb_bpm_force_B[1] +
         rtb_Assignment_c[i] * (real_T)rtb_bpm_force_B[0]);
  }

  // Sum: '<S255>/Sum2' incorporates:
  //   Constant: '<S255>/Constant8'
  //   DiscreteIntegrator: '<S255>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S255>/Discrete-Time Integrator2'
  //   Product: '<S255>/Divide2'

  rtb_DataTypeConversion5_idx_0 = (rtb_Sum *
    sim_model_lib0_P->epson_gyro_temp_bias_coeff[0] +
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[0]) +
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[0];
  rtb_DataTypeConversion5_idx_1 = (rtb_Sum *
    sim_model_lib0_P->epson_gyro_temp_bias_coeff[1] +
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[1]) +
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[1];
  rtb_DataTypeConversion5_idx_2 = (rtb_Sum *
    sim_model_lib0_P->epson_gyro_temp_bias_coeff[2] +
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[2]) +
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[2];

  // Switch: '<S255>/Switch' incorporates:
  //   Constant: '<S255>/Constant1'
  //   Constant: '<S255>/Constant5'
  //   Constant: '<S255>/Constant6'
  //   Constant: '<S255>/Constant9'
  //   Gain: '<S255>/Gain3'
  //   Gain: '<S255>/scale_factor'
  //   Product: '<S255>/Divide1'
  //   Product: '<S255>/Product2'
  //   Sum: '<S255>/Sum1'

  if ((int32_T)sim_model_lib0_P->tun_epson_report_truth == 0) {
    // Gain: '<S255>/Gain3'
    rtb_Sum = 1.0 / sqrt(sim_model_lib0_P->astrobee_time_step_size);
    rtb_Add3[0] *= sim_model_lib0_P->epson_gyro_sf_coef;
    rtb_Add3[1] *= sim_model_lib0_P->epson_gyro_sf_coef;
    rtb_Add3[2] *= sim_model_lib0_P->epson_gyro_sf_coef;

    // Product: '<S255>/Divide6' incorporates:
    //   Constant: '<S255>/Constant3'
    //   DotProduct: '<S257>/Dot Product1'
    //   Gain: '<S255>/scale_factor'
    //   Sqrt: '<S257>/Sqrt'

    rtb_linearaccelbias = sqrt((rtb_P_sensor_CG_B[0] * rtb_P_sensor_CG_B[0] +
      rtb_P_sensor_CG_B[1] * rtb_P_sensor_CG_B[1]) + rtb_P_sensor_CG_B[2] *
      rtb_P_sensor_CG_B[2]) *
      sim_model_lib0_P->epson_gyro_linear_accel_bias_coeff;
    rtb_Add3[0] = (((sin(rtb_Add3[0] * sim_model_lib0_P->Constant5_Value_a /
                         sim_model_lib0_P->epson_gyro_upper_sat) *
                     sim_model_lib0_P->epson_gyro_nonlinearity_coeff + rtb_Add3
                     [0]) + rtb_linearaccelbias) + rtb_Sum * rtb_random_noise[0])
      + rtb_DataTypeConversion5_idx_0;
    rtb_Add3[1] = (((sin(rtb_Add3[1] * sim_model_lib0_P->Constant5_Value_a /
                         sim_model_lib0_P->epson_gyro_upper_sat) *
                     sim_model_lib0_P->epson_gyro_nonlinearity_coeff + rtb_Add3
                     [1]) + rtb_linearaccelbias) + rtb_Sum * rtb_random_noise[1])
      + rtb_DataTypeConversion5_idx_1;
    rtb_Add3[2] = (((sin(rtb_Add3[2] * sim_model_lib0_P->Constant5_Value_a /
                         sim_model_lib0_P->epson_gyro_upper_sat) *
                     sim_model_lib0_P->epson_gyro_nonlinearity_coeff + rtb_Add3
                     [2]) + rtb_linearaccelbias) + rtb_Sum * rtb_random_noise[2])
      + rtb_DataTypeConversion5_idx_2;
  }

  // End of Switch: '<S255>/Switch'

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   Constant: '<S186>/Constant1'
  //   Constant: '<S186>/Constant2'

  sim_model_lib0_Y_imu_msg_o->imu_timestamp_sec = rtb_Switch1;
  sim_model_lib0_Y_imu_msg_o->imu_timestamp_nsec = rtb_Switch_g;
  sim_model_lib0_Y_imu_msg_o->imu_validity_flag =
    sim_model_lib0_P->Constant1_Value_ol;
  sim_model_lib0_Y_imu_msg_o->imu_sat_flag =
    sim_model_lib0_P->Constant2_Value_cc;

  // Quantizer: '<S256>/Quantizer1' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn'

  rtb_Sum = rt_roundd_snf(rtb_Add3[0] * sim_model_lib0_P->epson_gyro_filt_num /
    sim_model_lib0_P->epson_gyro_filt_den /
    sim_model_lib0_P->epson_gyro_resolution) *
    sim_model_lib0_P->epson_gyro_resolution;

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion'
  //   DataTypeConversion: '<S186>/Data Type Conversion8'
  //   Delay: '<S253>/Delay2'
  //   Saturate: '<S256>/Saturation'

  sim_model_lib0_Y_imu_msg_o->imu_A_B_ECI_sensor[0] = (real32_T)y;
  sim_model_lib0_Y_imu_msg_o->imu_accel_bias[0] = (real32_T)rtb_Add3_a[0];

  // Saturate: '<S256>/Saturation1'
  if (rtb_Sum > sim_model_lib0_P->epson_gyro_upper_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[0] = (real32_T)
      sim_model_lib0_P->epson_gyro_upper_sat;
  } else if (rtb_Sum < sim_model_lib0_P->epson_gyro_lower_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[0] = (real32_T)
      sim_model_lib0_P->epson_gyro_lower_sat;
  } else {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[0] = (real32_T)rtb_Sum;
  }

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion10'
  //   Delay: '<S253>/Delay1'

  sim_model_lib0_Y_imu_msg_o->imu_gyro_bias[0] = (real32_T)
    rtb_DataTypeConversion5_idx_0;

  // Outport: '<Root>/env_msg' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  sim_model_lib0_Y_env_msg_i->P_B_ISS_ISS[0] =
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];
  sim_model_lib0_Y_env_msg_i->V_B_ISS_ISS[0] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
  sim_model_lib0_Y_env_msg_i->A_B_ISS_ISS[0] = rtb_P_B_ISS_SS[0];
  sim_model_lib0_Y_env_msg_i->A_B_ISS_B[0] = rtb_Merge_o[0];
  sim_model_lib0_Y_env_msg_i->A_B_ECI_B[0] = rtb_imu_gyro_bias[0];

  // Quantizer: '<S256>/Quantizer1' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn'

  rtb_Sum = rt_roundd_snf(rtb_Add3[1] * sim_model_lib0_P->epson_gyro_filt_num /
    sim_model_lib0_P->epson_gyro_filt_den /
    sim_model_lib0_P->epson_gyro_resolution) *
    sim_model_lib0_P->epson_gyro_resolution;

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion'
  //   DataTypeConversion: '<S186>/Data Type Conversion8'
  //   Delay: '<S253>/Delay2'
  //   Saturate: '<S256>/Saturation'

  sim_model_lib0_Y_imu_msg_o->imu_A_B_ECI_sensor[1] = (real32_T)y_0;
  sim_model_lib0_Y_imu_msg_o->imu_accel_bias[1] = (real32_T)rtb_Add3_a[1];

  // Saturate: '<S256>/Saturation1'
  if (rtb_Sum > sim_model_lib0_P->epson_gyro_upper_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[1] = (real32_T)
      sim_model_lib0_P->epson_gyro_upper_sat;
  } else if (rtb_Sum < sim_model_lib0_P->epson_gyro_lower_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[1] = (real32_T)
      sim_model_lib0_P->epson_gyro_lower_sat;
  } else {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[1] = (real32_T)rtb_Sum;
  }

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion10'
  //   Delay: '<S253>/Delay1'

  sim_model_lib0_Y_imu_msg_o->imu_gyro_bias[1] = (real32_T)
    rtb_DataTypeConversion5_idx_1;

  // Outport: '<Root>/env_msg' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  sim_model_lib0_Y_env_msg_i->P_B_ISS_ISS[1] =
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
  sim_model_lib0_Y_env_msg_i->V_B_ISS_ISS[1] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
  sim_model_lib0_Y_env_msg_i->A_B_ISS_ISS[1] = rtb_P_B_ISS_SS[1];
  sim_model_lib0_Y_env_msg_i->A_B_ISS_B[1] = rtb_Merge_o[1];
  sim_model_lib0_Y_env_msg_i->A_B_ECI_B[1] = rtb_imu_gyro_bias[1];

  // Quantizer: '<S256>/Quantizer1' incorporates:
  //   DiscreteTransferFcn: '<S256>/Discrete Transfer Fcn'

  rtb_Sum = rt_roundd_snf(rtb_Add3[2] * sim_model_lib0_P->epson_gyro_filt_num /
    sim_model_lib0_P->epson_gyro_filt_den /
    sim_model_lib0_P->epson_gyro_resolution) *
    sim_model_lib0_P->epson_gyro_resolution;

  // Saturate: '<S256>/Saturation'
  if (rtb_Product > sim_model_lib0_P->epson_accel_upper_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion'

    sim_model_lib0_Y_imu_msg_o->imu_A_B_ECI_sensor[2] = (real32_T)
      sim_model_lib0_P->epson_accel_upper_sat;
  } else if (rtb_Product < sim_model_lib0_P->epson_accel_lower_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion'

    sim_model_lib0_Y_imu_msg_o->imu_A_B_ECI_sensor[2] = (real32_T)
      sim_model_lib0_P->epson_accel_lower_sat;
  } else {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion'

    sim_model_lib0_Y_imu_msg_o->imu_A_B_ECI_sensor[2] = (real32_T)rtb_Product;
  }

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion8'
  //   Delay: '<S253>/Delay2'

  sim_model_lib0_Y_imu_msg_o->imu_accel_bias[2] = (real32_T)rtb_Add3_a[2];

  // Saturate: '<S256>/Saturation1'
  if (rtb_Sum > sim_model_lib0_P->epson_gyro_upper_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[2] = (real32_T)
      sim_model_lib0_P->epson_gyro_upper_sat;
  } else if (rtb_Sum < sim_model_lib0_P->epson_gyro_lower_sat) {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[2] = (real32_T)
      sim_model_lib0_P->epson_gyro_lower_sat;
  } else {
    // BusCreator: '<S186>/Bus Creator' incorporates:
    //   DataTypeConversion: '<S186>/Data Type Conversion1'

    sim_model_lib0_Y_imu_msg_o->imu_omega_B_ECI_sensor[2] = (real32_T)rtb_Sum;
  }

  // BusCreator: '<S186>/Bus Creator' incorporates:
  //   DataTypeConversion: '<S186>/Data Type Conversion10'
  //   Delay: '<S253>/Delay1'

  sim_model_lib0_Y_imu_msg_o->imu_gyro_bias[2] = (real32_T)
    rtb_DataTypeConversion5_idx_2;

  // Outport: '<Root>/env_msg' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  sim_model_lib0_Y_env_msg_i->P_B_ISS_ISS[2] =
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
  sim_model_lib0_Y_env_msg_i->V_B_ISS_ISS[2] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
  sim_model_lib0_Y_env_msg_i->A_B_ISS_ISS[2] = rtb_Merge_c;
  sim_model_lib0_Y_env_msg_i->A_B_ISS_B[2] = rtb_Merge_o[2];
  sim_model_lib0_Y_env_msg_i->A_B_ECI_B[2] = rtb_imu_gyro_bias[2];
  sim_model_lib0_Y_env_msg_i->Q_ISS2B[0] = rtb_Merge_m[0];
  sim_model_lib0_Y_env_msg_i->Q_ISS2B[1] = rtb_Merge_m[1];
  sim_model_lib0_Y_env_msg_i->Q_ISS2B[2] = rtb_Merge_m[2];
  sim_model_lib0_Y_env_msg_i->Q_ISS2B[3] = rtb_Merge_m[3];
  sim_model_lib0_Y_env_msg_i->omega_B_ISS_B[0] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
  sim_model_lib0_Y_env_msg_i->omega_B_ECI_B[0] = rtb_bpm_force_B[0];
  sim_model_lib0_Y_env_msg_i->alpha_B_ISS_B[0] = rtb_alpha_B_ECI_B[0];
  sim_model_lib0_Y_env_msg_i->fan_torques_B[0] = rtb_Delay1_bpm_torque_B[0];
  sim_model_lib0_Y_env_msg_i->fan_forces_B[0] = rtb_TrigonometricFunction1_c;
  sim_model_lib0_Y_env_msg_i->omega_B_ISS_B[1] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];
  sim_model_lib0_Y_env_msg_i->omega_B_ECI_B[1] = rtb_bpm_force_B[1];
  sim_model_lib0_Y_env_msg_i->alpha_B_ISS_B[1] = rtb_alpha_B_ECI_B[1];
  sim_model_lib0_Y_env_msg_i->fan_torques_B[1] = rtb_Delay1_bpm_torque_B[1];
  sim_model_lib0_Y_env_msg_i->fan_forces_B[1] = rtb_Delay1_bpm_force_B_idx_1;
  sim_model_lib0_Y_env_msg_i->omega_B_ISS_B[2] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
  sim_model_lib0_Y_env_msg_i->omega_B_ECI_B[2] = rtb_bpm_force_B[2];
  sim_model_lib0_Y_env_msg_i->alpha_B_ISS_B[2] = rtb_alpha_B_ECI_B[2];
  sim_model_lib0_Y_env_msg_i->fan_torques_B[2] = rtb_Delay1_bpm_torque_B[2];
  sim_model_lib0_Y_env_msg_i->fan_forces_B[2] = rtb_Delay1_bpm_force_B_idx_2;

  // Outputs for Atomic SubSystem: '<S190>/speed_controller'

  // Constant: '<S4>/Constant1' incorporates:
  //   DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  //   Inport: '<Root>/act_msg'

  sim_model__speed_controller(sim_model_lib0_P->Constant1_Value_fq,
    sim_model_lib0_U_act_msg_l->act_impeller_speed_cmd[0],
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp,
    &sim_model_lib0_B->speed_controller, &sim_model_lib0_DW->speed_controller,
    (P_speed_controller_sim_model__T *)&sim_model_lib0_P->speed_controller,
    sim_model_lib0_P->bpm_blower_1_propulsion_module_,
    sim_model_lib0_P->bpm_imp_cmd_filt_num,
    sim_model_lib0_P->bpm_imp_cmd_filt_den,
    sim_model_lib0_P->bpm_imp_speed_filt_num,
    sim_model_lib0_P->bpm_imp_speed_filt_den, sim_model_lib0_P->bpm_imp_ctl_kp,
    sim_model_lib0_P->bpm_imp_ctl_kd, sim_model_lib0_P->bpm_imp_ctl_filt_n,
    sim_model_lib0_P->bpm_imp_max_voltage, sim_model_lib0_P->bpm_imp_ctl_ki);

  // End of Outputs for SubSystem: '<S190>/speed_controller'

  // Outputs for Atomic SubSystem: '<S190>/dc_motor_model'

  // DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  sim_model_li_dc_motor_model(sim_model_lib0_B->speed_controller.Switch2,
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp,
    &sim_model_lib0_B->dc_motor_model, sim_model_lib0_P->bpm_imp_motor_speed_k,
    sim_model_lib0_P->bpm_imp_motor_r, sim_model_lib0_P->bpm_imp_motor_torque_k,
    sim_model_lib0_P->bpm_imp_motor_friction_coeff);

  // End of Outputs for SubSystem: '<S190>/dc_motor_model'

  // SignalConversion: '<S4>/ConcatBufferAtVector ConcatenateIn1'
  rtb_bpm_motor_curr_idx_0 = sim_model_lib0_B->dc_motor_model.current;

  // Outputs for Atomic SubSystem: '<S223>/speed_controller'

  // Constant: '<S4>/Constant2' incorporates:
  //   DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  //   Inport: '<Root>/act_msg'

  sim_model__speed_controller(sim_model_lib0_P->Constant2_Value_cw,
    sim_model_lib0_U_act_msg_l->act_impeller_speed_cmd[1],
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e,
    &sim_model_lib0_B->speed_controller_c,
    &sim_model_lib0_DW->speed_controller_c, (P_speed_controller_sim_model__T *)
    &sim_model_lib0_P->speed_controller_c,
    sim_model_lib0_P->bpm_blower_2_propulsion_module_,
    sim_model_lib0_P->bpm_imp_cmd_filt_num,
    sim_model_lib0_P->bpm_imp_cmd_filt_den,
    sim_model_lib0_P->bpm_imp_speed_filt_num,
    sim_model_lib0_P->bpm_imp_speed_filt_den, sim_model_lib0_P->bpm_imp_ctl_kp,
    sim_model_lib0_P->bpm_imp_ctl_kd, sim_model_lib0_P->bpm_imp_ctl_filt_n,
    sim_model_lib0_P->bpm_imp_max_voltage, sim_model_lib0_P->bpm_imp_ctl_ki);

  // End of Outputs for SubSystem: '<S223>/speed_controller'

  // Outputs for Atomic SubSystem: '<S223>/dc_motor_model'

  // DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  sim_model_li_dc_motor_model(sim_model_lib0_B->speed_controller_c.Switch2,
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e,
    &sim_model_lib0_B->dc_motor_model_g, sim_model_lib0_P->bpm_imp_motor_speed_k,
    sim_model_lib0_P->bpm_imp_motor_r, sim_model_lib0_P->bpm_imp_motor_torque_k,
    sim_model_lib0_P->bpm_imp_motor_friction_coeff);

  // End of Outputs for SubSystem: '<S223>/dc_motor_model'

  // Outputs for Atomic SubSystem: '<S184>/servo_model'

  // Inport: '<Root>/act_msg'
  sim_model_lib0_servo_model((real32_T *)
    &sim_model_lib0_U_act_msg_l->act_servo_pwm_cmd[0],
    &sim_model_lib0_B->servo_model, &sim_model_lib0_DW->servo_model,
    (P_servo_model_sim_model_lib0_T *)&sim_model_lib0_P->servo_model,
    sim_model_lib0_P->bpm_servo_pwm2angle_bias,
    sim_model_lib0_P->bpm_servo_pwm2angle, (real32_T)
    sim_model_lib0_P->bpm_servo_min_theta,
    sim_model_lib0_P->bpm_servo_motor_gear_ratio,
    sim_model_lib0_P->bpm_servo_max_theta,
    sim_model_lib0_P->bpm_servo_motor_backlash_deadband,
    sim_model_lib0_P->bpm_servo_ctl_kp, sim_model_lib0_P->bpm_servo_ctl_kd,
    sim_model_lib0_P->bpm_servo_ctl_filt_n,
    sim_model_lib0_P->bpm_servo_max_voltage, sim_model_lib0_P->bpm_servo_motor_k,
    sim_model_lib0_P->bpm_servo_motor_r,
    sim_model_lib0_P->bpm_servo_motor_friction_coeff,
    sim_model_lib0_P->bpm_servo_motor_gear_box_inertia,
    sim_model_lib0_P->bpm_servo_ctl_ki);

  // End of Outputs for SubSystem: '<S184>/servo_model'

  // SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In1'
  for (i = 0; i < 6; i++) {
    rtb_bpm_servo_curr[i] = sim_model_lib0_B->servo_model.current[i];
  }

  // End of SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In1'

  // Outputs for Atomic SubSystem: '<S185>/servo_model'

  // Inport: '<Root>/act_msg'
  sim_model_lib0_servo_model((real32_T *)
    &sim_model_lib0_U_act_msg_l->act_servo_pwm_cmd[6],
    &sim_model_lib0_B->servo_model_f, &sim_model_lib0_DW->servo_model_f,
    (P_servo_model_sim_model_lib0_T *)&sim_model_lib0_P->servo_model_f,
    sim_model_lib0_P->bpm_servo_pwm2angle_bias,
    sim_model_lib0_P->bpm_servo_pwm2angle, (real32_T)
    sim_model_lib0_P->bpm_servo_min_theta,
    sim_model_lib0_P->bpm_servo_motor_gear_ratio,
    sim_model_lib0_P->bpm_servo_max_theta,
    sim_model_lib0_P->bpm_servo_motor_backlash_deadband,
    sim_model_lib0_P->bpm_servo_ctl_kp, sim_model_lib0_P->bpm_servo_ctl_kd,
    sim_model_lib0_P->bpm_servo_ctl_filt_n,
    sim_model_lib0_P->bpm_servo_max_voltage, sim_model_lib0_P->bpm_servo_motor_k,
    sim_model_lib0_P->bpm_servo_motor_r,
    sim_model_lib0_P->bpm_servo_motor_friction_coeff,
    sim_model_lib0_P->bpm_servo_motor_gear_box_inertia,
    sim_model_lib0_P->bpm_servo_ctl_ki);

  // End of Outputs for SubSystem: '<S185>/servo_model'

  // SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In2'
  for (i = 0; i < 6; i++) {
    rtb_bpm_servo_curr[(int32_T)(i + 6)] =
      sim_model_lib0_B->servo_model_f.current[i];
  }

  // End of SignalConversion: '<S4>/ConcatBufferAtVector Concatenate1In2'

  // Sum: '<S188>/Add1' incorporates:
  //   Constant: '<S188>/Constant1'
  //   Constant: '<S188>/Constant3'
  //   Gain: '<S188>/Gain'

  rtb_bpm_torque_B_c = sim_model_lib0_P->tun_bpm_noise_on_flag *
    sim_model_lib0_P->bmp_PM1_impeller_orientation_error[0] +
    sim_model_lib0_P->abp_pm1_impeller_orientation[0];

  // DotProduct: '<S210>/Dot Product'
  rtb_Sqrt_m = rtb_bpm_torque_B_c * rtb_bpm_torque_B_c;

  // Sum: '<S188>/Add1' incorporates:
  //   Constant: '<S188>/Constant1'
  //   Constant: '<S188>/Constant3'
  //   Gain: '<S188>/Gain'

  rtb_bpm_torque_B[0] = rtb_bpm_torque_B_c;
  rtb_bpm_torque_B_c = sim_model_lib0_P->tun_bpm_noise_on_flag *
    sim_model_lib0_P->bmp_PM1_impeller_orientation_error[1] +
    sim_model_lib0_P->abp_pm1_impeller_orientation[1];

  // DotProduct: '<S210>/Dot Product'
  rtb_Sqrt_m += rtb_bpm_torque_B_c * rtb_bpm_torque_B_c;

  // Sum: '<S188>/Add1' incorporates:
  //   Constant: '<S188>/Constant1'
  //   Constant: '<S188>/Constant3'
  //   Gain: '<S188>/Gain'

  rtb_bpm_torque_B[1] = rtb_bpm_torque_B_c;
  rtb_bpm_torque_B_c = sim_model_lib0_P->tun_bpm_noise_on_flag *
    sim_model_lib0_P->bmp_PM1_impeller_orientation_error[2] +
    sim_model_lib0_P->abp_pm1_impeller_orientation[2];

  // DotProduct: '<S210>/Dot Product'
  rtb_Sqrt_m += rtb_bpm_torque_B_c * rtb_bpm_torque_B_c;

  // Sum: '<S188>/Add1'
  rtb_bpm_torque_B[2] = rtb_bpm_torque_B_c;

  // Sqrt: '<S210>/Sqrt' incorporates:
  //   DotProduct: '<S210>/Dot Product'

  rtb_Sqrt_m = (real32_T)sqrt((real_T)rtb_Sqrt_m);

  // If: '<S197>/If' incorporates:
  //   DataTypeConversion: '<S197>/Data Type Conversion'
  //   Inport: '<S208>/In1'

  if ((real_T)rtb_Sqrt_m > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S197>/Normalize' incorporates:
    //   ActionPort: '<S209>/Action Port'

    sim_model_lib0_Normalize_e(rtb_bpm_torque_B, rtb_Sqrt_m, rtb_A_B_ISS_ISS);

    // End of Outputs for SubSystem: '<S197>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S197>/No-op' incorporates:
    //   ActionPort: '<S208>/Action Port'

    rtb_A_B_ISS_ISS[0] = rtb_bpm_torque_B[0];
    rtb_A_B_ISS_ISS[1] = rtb_bpm_torque_B[1];
    rtb_A_B_ISS_ISS[2] = rtb_bpm_torque_B_c;

    // End of Outputs for SubSystem: '<S197>/No-op'
  }

  // End of If: '<S197>/If'

  // Gain: '<S191>/Gain1'
  rtb_Product_p = sim_model_lib0_P->Gain1_Gain_k3 *
    sim_model_lib0_B->dc_motor_model.output_torque;

  // Product: '<S188>/Product'
  rtb_bpm_torque_B[0] = rtb_A_B_ISS_ISS[0] * rtb_Product_p;
  rtb_bpm_torque_B[1] = rtb_A_B_ISS_ISS[1] * rtb_Product_p;
  rtb_bpm_torque_B[2] = rtb_A_B_ISS_ISS[2] * rtb_Product_p;

  // Product: '<S188>/Product1' incorporates:
  //   Constant: '<S188>/Constant2'
  //   Constant: '<S188>/Constant4'
  //   DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  //   Gain: '<S188>/Gain1'
  //   Sum: '<S188>/Add3'

  rtb_Product_p = (sim_model_lib0_P->tun_bpm_noise_on_flag *
                   sim_model_lib0_P->bpm_impeller_inertia_error +
                   sim_model_lib0_P->bpm_impeller_inertia) *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp;

  // Outputs for Enabled SubSystem: '<S188>/latch_nozzle_thrust_matricies'

  // UnitDelay: '<S188>/Unit Delay'
  latch_nozzle_thrust_matrici(sim_model_lib0_DW->UnitDelay_DSTATE_m,
    rtb_BusAssignment1.center_of_mass,
    &sim_model_lib0_B->latch_nozzle_thrust_matricies,
    (P_latch_nozzle_thrust_matrici_T *)
    &sim_model_lib0_P->latch_nozzle_thrust_matricies,
    sim_model_lib0_P->bpm_PM1_Q_nozzle2misaligned,
    sim_model_lib0_P->abp_PM1_P_nozzle_B_B,
    sim_model_lib0_P->abp_PM1_nozzle_orientations,
    sim_model_lib0_P->bpm_PM1_P_nozzle_B_B_error,
    sim_model_lib0_P->abp_P_CG_B_B_error,
    sim_model_lib0_P->tun_bpm_noise_on_flag);

  // End of Outputs for SubSystem: '<S188>/latch_nozzle_thrust_matricies'

  // Outputs for Atomic SubSystem: '<S184>/calc_nozzle_area'
  sim_model__calc_nozzle_area(sim_model_lib0_B->servo_model.Gain1,
    &sim_model_lib0_B->calc_nozzle_area, sim_model_lib0_P->abp_nozzle_flap_count,
    sim_model_lib0_P->abp_nozzle_min_open_angle,
    sim_model_lib0_P->abp_nozzle_flap_length,
    sim_model_lib0_P->abp_nozzle_intake_height,
    sim_model_lib0_P->abp_PM1_nozzle_widths,
    sim_model_lib0_P->abp_nozzle_gear_ratio);

  // End of Outputs for SubSystem: '<S184>/calc_nozzle_area'

  // Outputs for Atomic SubSystem: '<S184>/blower_aerodynamics'

  // DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  sim_mod_blower_aerodynamics(sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp,
    sim_model_lib0_B->calc_nozzle_area.Product2,
    &sim_model_lib0_B->blower_aerodynamics,
    &sim_model_lib0_DW->blower_aerodynamics, (P_blower_aerodynamics_sim_mod_T *)
    &sim_model_lib0_P->blower_aerodynamics,
    sim_model_lib0_P->abp_pm1_zero_thrust_area,
    sim_model_lib0_P->bpm_PM1_zero_thrust_area_error,
    sim_model_lib0_P->tun_bpm_noise_on_flag,
    sim_model_lib0_P->abp_PM1_discharge_coeff,
    sim_model_lib0_P->bpm_PM1_nozzle_discharge_coeff_error,
    sim_model_lib0_P->bpm_lookup_Cdp_data,
    sim_model_lib0_P->bpm_lookup_totalarea_breakpoints,
    sim_model_lib0_P->abp_impeller_diameter, sim_model_lib0_P->const_air_density,
    sim_model_lib0_P->bpm_PM1_nozzle_noise_feedback_gain);

  // End of Outputs for SubSystem: '<S184>/blower_aerodynamics'

  // SignalConversion: '<S195>/TmpSignal ConversionAtProductInport1' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   Constant: '<S198>/Constant3'
  //   DataTypeConversion: '<S199>/Conversion'
  //   Gain: '<S198>/Gain'
  //   Gain: '<S198>/Gain1'
  //   Gain: '<S198>/Gain2'
  //   Product: '<S195>/Product'

  tmp_8[0] = (real32_T)sim_model_lib0_P->Constant3_Value_ao;
  tmp_8[1] = rtb_bpm_force_B[2];
  tmp_8[2] = sim_model_lib0_P->Gain_Gain_i * rtb_bpm_force_B[1];
  tmp_8[3] = sim_model_lib0_P->Gain1_Gain_me * rtb_bpm_force_B[2];
  tmp_8[4] = (real32_T)sim_model_lib0_P->Constant3_Value_ao;
  tmp_8[5] = rtb_bpm_force_B[0];
  tmp_8[6] = rtb_bpm_force_B[1];
  tmp_8[7] = sim_model_lib0_P->Gain2_Gain_b * rtb_bpm_force_B[0];
  tmp_8[8] = (real32_T)sim_model_lib0_P->Constant3_Value_ao;

  // DotProduct: '<S243>/Dot Product'
  rtb_Sqrt_m = 0.0F;
  for (i = 0; i < 3; i++) {
    // Product: '<S188>/Product4' incorporates:
    //   Sum: '<S188>/Add2'

    tmp[i] = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      tmp[i] +=
        sim_model_lib0_B->latch_nozzle_thrust_matricies.OutportBufferForthrust2torque_B
        [(int32_T)((int32_T)(3 * i_0) + i)] *
        sim_model_lib0_B->blower_aerodynamics.Add1[i_0];
    }

    // End of Product: '<S188>/Product4'

    // Sum: '<S188>/Add2' incorporates:
    //   Product: '<S188>/Product2'
    //   Product: '<S195>/Product'

    rtb_V_B_ISS_ISS[i] = (((rtb_A_B_ISS_ISS[0] * rtb_Product_p * tmp_8[i] +
      tmp_8[(int32_T)(i + 3)] * (rtb_A_B_ISS_ISS[1] * rtb_Product_p)) + tmp_8
      [(int32_T)(i + 6)] * (rtb_A_B_ISS_ISS[2] * rtb_Product_p)) +
                          rtb_bpm_torque_B[i]) + tmp[i];

    // Sum: '<S221>/Add1' incorporates:
    //   Constant: '<S221>/Constant1'
    //   Constant: '<S221>/Constant3'
    //   Gain: '<S221>/Gain'

    rtb_bpm_torque_B_c = sim_model_lib0_P->tun_bpm_noise_on_flag *
      sim_model_lib0_P->bmp_PM2_impeller_orientation_error[i] +
      sim_model_lib0_P->abp_pm2_impeller_orientation[i];

    // DotProduct: '<S243>/Dot Product'
    rtb_Sqrt_m += rtb_bpm_torque_B_c * rtb_bpm_torque_B_c;

    // Sum: '<S221>/Add1'
    rtb_bpm_torque_B[i] = rtb_bpm_torque_B_c;
  }

  // Sqrt: '<S243>/Sqrt' incorporates:
  //   DotProduct: '<S243>/Dot Product'

  rtb_Sqrt_m = (real32_T)sqrt((real_T)rtb_Sqrt_m);

  // If: '<S230>/If' incorporates:
  //   DataTypeConversion: '<S230>/Data Type Conversion'
  //   Inport: '<S241>/In1'

  if ((real_T)rtb_Sqrt_m > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S230>/Normalize' incorporates:
    //   ActionPort: '<S242>/Action Port'

    sim_model_lib0_Normalize_e(rtb_bpm_torque_B, rtb_Sqrt_m, rtb_A_B_ISS_ISS);

    // End of Outputs for SubSystem: '<S230>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S230>/No-op' incorporates:
    //   ActionPort: '<S241>/Action Port'

    rtb_A_B_ISS_ISS[0] = rtb_bpm_torque_B[0];
    rtb_A_B_ISS_ISS[1] = rtb_bpm_torque_B[1];
    rtb_A_B_ISS_ISS[2] = rtb_bpm_torque_B[2];

    // End of Outputs for SubSystem: '<S230>/No-op'
  }

  // End of If: '<S230>/If'

  // Gain: '<S224>/Gain1'
  rtb_Product_p = sim_model_lib0_P->Gain1_Gain_hf *
    sim_model_lib0_B->dc_motor_model_g.output_torque;

  // Product: '<S221>/Product'
  rtb_bpm_torque_B[0] = rtb_A_B_ISS_ISS[0] * rtb_Product_p;
  rtb_bpm_torque_B[1] = rtb_A_B_ISS_ISS[1] * rtb_Product_p;
  rtb_bpm_torque_B[2] = rtb_A_B_ISS_ISS[2] * rtb_Product_p;

  // Product: '<S221>/Product1' incorporates:
  //   Constant: '<S221>/Constant2'
  //   Constant: '<S221>/Constant4'
  //   DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  //   Gain: '<S221>/Gain1'
  //   Sum: '<S221>/Add3'

  rtb_Product_p = (sim_model_lib0_P->tun_bpm_noise_on_flag *
                   sim_model_lib0_P->bpm_impeller_inertia_error +
                   sim_model_lib0_P->bpm_impeller_inertia) *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e;

  // Outputs for Enabled SubSystem: '<S221>/latch_nozzle_thrust_matricies'

  // UnitDelay: '<S221>/Unit Delay'
  latch_nozzle_thrust_matrici(sim_model_lib0_DW->UnitDelay_DSTATE_c,
    rtb_BusAssignment1.center_of_mass,
    &sim_model_lib0_B->latch_nozzle_thrust_matricies_p,
    (P_latch_nozzle_thrust_matrici_T *)
    &sim_model_lib0_P->latch_nozzle_thrust_matricies_p,
    sim_model_lib0_P->bpm_PM2_Q_nozzle2misaligned,
    sim_model_lib0_P->abp_PM2_P_nozzle_B_B,
    sim_model_lib0_P->abp_PM2_nozzle_orientations,
    sim_model_lib0_P->bpm_PM2_P_nozzle_B_B_error,
    sim_model_lib0_P->abp_P_CG_B_B_error,
    sim_model_lib0_P->tun_bpm_noise_on_flag);

  // End of Outputs for SubSystem: '<S221>/latch_nozzle_thrust_matricies'

  // Outputs for Atomic SubSystem: '<S185>/calc_nozzle_area'
  sim_model__calc_nozzle_area(sim_model_lib0_B->servo_model_f.Gain1,
    &sim_model_lib0_B->calc_nozzle_area_c,
    sim_model_lib0_P->abp_nozzle_flap_count,
    sim_model_lib0_P->abp_nozzle_min_open_angle,
    sim_model_lib0_P->abp_nozzle_flap_length,
    sim_model_lib0_P->abp_nozzle_intake_height,
    sim_model_lib0_P->abp_PM2_nozzle_widths,
    sim_model_lib0_P->abp_nozzle_gear_ratio);

  // End of Outputs for SubSystem: '<S185>/calc_nozzle_area'

  // Outputs for Atomic SubSystem: '<S185>/blower_aerodynamics'

  // DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  sim_m_blower_aerodynamics_j(sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e,
    sim_model_lib0_B->calc_nozzle_area_c.Product2,
    &sim_model_lib0_B->blower_aerodynamics_j,
    &sim_model_lib0_DW->blower_aerodynamics_j, (P_blower_aerodynamics_sim_m_m_T *)
    &sim_model_lib0_P->blower_aerodynamics_j,
    sim_model_lib0_P->abp_pm2_zero_thrust_area,
    sim_model_lib0_P->bpm_PM2_zero_thrust_area_error,
    sim_model_lib0_P->tun_bpm_noise_on_flag,
    sim_model_lib0_P->abp_PM2_discharge_coeff,
    sim_model_lib0_P->bpm_PM2_nozzle_discharge_coeff_error,
    sim_model_lib0_P->bpm_lookup_Cdp_data,
    sim_model_lib0_P->bpm_lookup_totalarea_breakpoints,
    sim_model_lib0_P->abp_impeller_diameter, sim_model_lib0_P->const_air_density,
    sim_model_lib0_P->bpm_PM2_nozzle_noise_feedback_gain);

  // End of Outputs for SubSystem: '<S185>/blower_aerodynamics'

  // SignalConversion: '<S228>/TmpSignal ConversionAtProductInport1' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   Constant: '<S231>/Constant3'
  //   DataTypeConversion: '<S232>/Conversion'
  //   Gain: '<S231>/Gain'
  //   Gain: '<S231>/Gain1'
  //   Gain: '<S231>/Gain2'
  //   Product: '<S228>/Product'

  tmp_9[0] = (real32_T)sim_model_lib0_P->Constant3_Value_c;
  tmp_9[1] = rtb_bpm_force_B[2];
  tmp_9[2] = sim_model_lib0_P->Gain_Gain_ib * rtb_bpm_force_B[1];
  tmp_9[3] = sim_model_lib0_P->Gain1_Gain_ak * rtb_bpm_force_B[2];
  tmp_9[4] = (real32_T)sim_model_lib0_P->Constant3_Value_c;
  tmp_9[5] = rtb_bpm_force_B[0];
  tmp_9[6] = rtb_bpm_force_B[1];
  tmp_9[7] = sim_model_lib0_P->Gain2_Gain_al * rtb_bpm_force_B[0];
  tmp_9[8] = (real32_T)sim_model_lib0_P->Constant3_Value_c;
  for (i_0 = 0; i_0 < 3; i_0++) {
    // Product: '<S221>/Product4' incorporates:
    //   Sum: '<S221>/Add2'

    tmp[i_0] = 0.0F;

    // Product: '<S188>/Product3' incorporates:
    //   Sum: '<S4>/Add1'

    tmp_a[i_0] = 0.0F;

    // Product: '<S221>/Product3' incorporates:
    //   Sum: '<S4>/Add1'

    tmp_b[i_0] = 0.0F;
    for (i = 0; i < 6; i++) {
      // Product: '<S221>/Product4' incorporates:
      //   Sum: '<S221>/Add2'

      tmp[i_0] +=
        sim_model_lib0_B->latch_nozzle_thrust_matricies_p.OutportBufferForthrust2torque_B
        [(int32_T)((int32_T)(3 * i) + i_0)] *
        sim_model_lib0_B->blower_aerodynamics_j.Add1[i];

      // Product: '<S188>/Product3' incorporates:
      //   Sum: '<S4>/Add1'

      tmp_a[i_0] +=
        sim_model_lib0_B->latch_nozzle_thrust_matricies.OutportBufferForthrust2force_B
        [(int32_T)((int32_T)(3 * i) + i_0)] *
        sim_model_lib0_B->blower_aerodynamics.Add1[i];

      // Product: '<S221>/Product3' incorporates:
      //   Sum: '<S4>/Add1'

      tmp_b[i_0] +=
        sim_model_lib0_B->latch_nozzle_thrust_matricies_p.OutportBufferForthrust2force_B
        [(int32_T)((int32_T)(3 * i) + i_0)] *
        sim_model_lib0_B->blower_aerodynamics_j.Add1[i];
    }

    // Sum: '<S4>/Add1'
    rtb_bpm_force_B[i_0] = tmp_a[i_0] + tmp_b[i_0];

    // Sum: '<S4>/Add' incorporates:
    //   Product: '<S221>/Product2'
    //   Product: '<S228>/Product'
    //   Sum: '<S221>/Add2'

    rtb_bpm_torque_B[i_0] = ((((rtb_A_B_ISS_ISS[0] * rtb_Product_p * tmp_9[i_0]
      + tmp_9[(int32_T)(i_0 + 3)] * (rtb_A_B_ISS_ISS[1] * rtb_Product_p)) +
      tmp_9[(int32_T)(i_0 + 6)] * (rtb_A_B_ISS_ISS[2] * rtb_Product_p)) +
      rtb_bpm_torque_B[i_0]) + tmp[i_0]) + rtb_V_B_ISS_ISS[i_0];
  }

  for (i = 0; i < 6; i++) {
    // SignalConversion: '<S4>/ConcatBufferAtVector Concatenate5In1'
    rtb_bpm_nozzle_theta[i] = sim_model_lib0_B->calc_nozzle_area.Gain12[i];

    // SignalConversion: '<S4>/ConcatBufferAtVector Concatenate5In2'
    rtb_bpm_nozzle_theta[(int32_T)(i + 6)] =
      sim_model_lib0_B->calc_nozzle_area_c.Gain12[i];
  }

  // Outport: '<Root>/bpm_msg' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   BusCreator: '<S4>/Bus Creator'
  //   DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  //   SignalConversion: '<S4>/ConcatBufferAtVector ConcatenateIn2'

  sim_model_lib0_Y_bpm_msg_h->bpm_timestamp_sec = rtb_Switch1;
  sim_model_lib0_Y_bpm_msg_h->bpm_timestamp_nsec = rtb_Switch_g;
  sim_model_lib0_Y_bpm_msg_h->bpm_motor_curr[0] = rtb_bpm_motor_curr_idx_0;
  sim_model_lib0_Y_bpm_msg_h->bpm_motor_curr[1] =
    sim_model_lib0_B->dc_motor_model_g.current;
  sim_model_lib0_Y_bpm_msg_h->bpm_torque_B[0] = rtb_bpm_torque_B[0];
  sim_model_lib0_Y_bpm_msg_h->bpm_force_B[0] = rtb_bpm_force_B[0];
  sim_model_lib0_Y_bpm_msg_h->bpm_torque_B[1] = rtb_bpm_torque_B[1];
  sim_model_lib0_Y_bpm_msg_h->bpm_force_B[1] = rtb_bpm_force_B[1];
  sim_model_lib0_Y_bpm_msg_h->bpm_torque_B[2] = rtb_bpm_torque_B[2];
  sim_model_lib0_Y_bpm_msg_h->bpm_force_B[2] = rtb_bpm_force_B[2];
  sim_model_lib0_Y_bpm_msg_h->bpm_motor_speed[0] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp;
  sim_model_lib0_Y_bpm_msg_h->bpm_motor_speed[1] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e;
  for (i = 0; i < 12; i++) {
    sim_model_lib0_Y_bpm_msg_h->bpm_servo_curr[i] = rtb_bpm_servo_curr[i];
    sim_model_lib0_Y_bpm_msg_h->bpm_nozzle_theta[i] = rtb_bpm_nozzle_theta[i];
  }

  // End of Outport: '<Root>/bpm_msg'

  // DiscretePulseGenerator: '<S80>/AR_pulse_gen'
  rtb_Sum = ((real_T)sim_model_lib0_DW->clockTickCounter <
             sim_model_lib0_P->AR_pulse_gen_Duty) &&
    (sim_model_lib0_DW->clockTickCounter >= 0) ?
    sim_model_lib0_P->AR_pulse_gen_Amp : 0.0;
  if ((real_T)sim_model_lib0_DW->clockTickCounter >= floor
      (sim_model_lib0_P->cvs_AR_process_time /
       sim_model_lib0_P->astrobee_time_step_size) - 1.0) {
    sim_model_lib0_DW->clockTickCounter = 0;
  } else {
    sim_model_lib0_DW->clockTickCounter++;
  }

  // End of DiscretePulseGenerator: '<S80>/AR_pulse_gen'

  // DataTypeConversion: '<S80>/Data Type Conversion2'
  rtb_Product = floor(rtb_Sum);
  if (rtIsNaN(rtb_Product) || rtIsInf(rtb_Product)) {
    rtb_Product = 0.0;
  } else {
    rtb_Product = fmod(rtb_Product, 256.0);
  }

  // DiscretePulseGenerator: '<S80>/landmark_pulse_gen'
  rtb_Sum = ((real_T)sim_model_lib0_DW->clockTickCounter_c <
             sim_model_lib0_P->landmark_pulse_gen_Duty) &&
    (sim_model_lib0_DW->clockTickCounter_c >= 0) ?
    sim_model_lib0_P->landmark_pulse_gen_Amp : 0.0;
  if ((real_T)sim_model_lib0_DW->clockTickCounter_c >= floor
      (sim_model_lib0_P->cvs_landmark_process_time /
       sim_model_lib0_P->astrobee_time_step_size) - 1.0) {
    sim_model_lib0_DW->clockTickCounter_c = 0;
  } else {
    sim_model_lib0_DW->clockTickCounter_c++;
  }

  // End of DiscretePulseGenerator: '<S80>/landmark_pulse_gen'

  // DataTypeConversion: '<S80>/Data Type Conversion1'
  y = floor(rtb_Sum);
  if (rtIsNaN(y) || rtIsInf(y)) {
    y = 0.0;
  } else {
    y = fmod(y, 256.0);
  }

  // RateTransition: '<S83>/Rate Transition10'
  if (sim_model_lib0_M->Timing.RateInteraction.TID0_3 == 1) {
    sim_model_lib0_B->RateTransition10 =
      sim_model_lib0_DW->RateTransition10_Buffer0;

    // RateTransition: '<S83>/Rate Transition3'
    sim_model_lib0_B->RateTransition3_j =
      sim_model_lib0_DW->RateTransition3_Buffer0_g;

    // RateTransition: '<S83>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_j =
      sim_model_lib0_DW->RateTransition1_Buffer0_b;

    // RateTransition: '<S83>/Rate Transition6'
    memcpy(&sim_model_lib0_B->RateTransition6[0],
           &sim_model_lib0_DW->RateTransition6_Buffer0[0], (uint32_T)(1600U *
            sizeof(real32_T)));

    // RateTransition: '<S83>/Rate Transition7'
    memcpy(&sim_model_lib0_B->RateTransition7_f[0],
           &sim_model_lib0_DW->RateTransition7_Buffer0_a[0], (uint32_T)(800U *
            sizeof(uint8_T)));

    // RateTransition: '<S83>/Rate Transition8'
    memcpy(&sim_model_lib0_B->RateTransition8[0],
           &sim_model_lib0_DW->RateTransition8_Buffer0[0], (uint32_T)(50U *
            sizeof(real32_T)));
  }

  // End of RateTransition: '<S83>/Rate Transition10'

  // RateTransition: '<S82>/Rate Transition3'
  if (sim_model_lib0_M->Timing.RateInteraction.TID0_4 == 1) {
    sim_model_lib0_B->RateTransition3_o =
      sim_model_lib0_DW->RateTransition3_Buffer0_o;

    // RateTransition: '<S82>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_a =
      sim_model_lib0_DW->RateTransition1_Buffer0_l;

    // RateTransition: '<S82>/Rate Transition5'
    memcpy(&sim_model_lib0_B->RateTransition5_b[0],
           &sim_model_lib0_DW->RateTransition5_Buffer0_i[0], (uint32_T)(150U *
            sizeof(real32_T)));

    // RateTransition: '<S82>/Rate Transition6'
    memcpy(&sim_model_lib0_B->RateTransition6_n[0],
           &sim_model_lib0_DW->RateTransition6_Buffer0_h[0], (uint32_T)(100U *
            sizeof(real32_T)));

    // RateTransition: '<S82>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7_g[i] =
        sim_model_lib0_DW->RateTransition7_Buffer0_an[i];
    }

    // End of RateTransition: '<S82>/Rate Transition7'
  }

  // End of RateTransition: '<S82>/Rate Transition3'

  // BusCreator: '<S69>/bus_creator1' incorporates:
  //   Constant: '<S69>/cvs_3d_knowledge_flag'
  //   Constant: '<S69>/cvs_handrail_local_pos'
  //   Constant: '<S69>/cvs_handrail_local_quat'
  //   Constant: '<S69>/cvs_handrail_update_global_pose_flag'

  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_timestamp_sec =
    sim_model_lib0_B->RateTransition3;
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_timestamp_nsec =
    sim_model_lib0_B->RateTransition1;
  memcpy(&sim_model_lib0_Y_cvs_handrail_msg_h->cvs_landmarks[0],
         &sim_model_lib0_B->RateTransition5[0], (uint32_T)(150U * sizeof
          (real32_T)));
  memcpy(&sim_model_lib0_Y_cvs_handrail_msg_h->cvs_observations[0],
         &sim_model_lib0_B->RateTransition9[0], (uint32_T)(150U * sizeof
          (real32_T)));
  for (i = 0; i < 50; i++) {
    sim_model_lib0_Y_cvs_handrail_msg_h->cvs_valid_flag[i] =
      sim_model_lib0_B->RateTransition7[i];
  }

  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_3d_knowledge_flag =
    sim_model_lib0_P->cvs_3d_knowledge_flag_Value;
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_pos[0] =
    sim_model_lib0_P->cvs_handrail_local_pos_Value[0];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_pos[1] =
    sim_model_lib0_P->cvs_handrail_local_pos_Value[1];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_pos[2] =
    sim_model_lib0_P->cvs_handrail_local_pos_Value[2];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_quat[0] =
    sim_model_lib0_P->cvs_handrail_local_quat_Value[0];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_quat[1] =
    sim_model_lib0_P->cvs_handrail_local_quat_Value[1];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_quat[2] =
    sim_model_lib0_P->cvs_handrail_local_quat_Value[2];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_local_quat[3] =
    sim_model_lib0_P->cvs_handrail_local_quat_Value[3];
  sim_model_lib0_Y_cvs_handrail_msg_h->cvs_handrail_update_global_pose_flag =
    sim_model_lib0_P->cvs_handrail_update_global_pose;

  // End of BusCreator: '<S69>/bus_creator1'

  // Outport: '<Root>/cvs_landmark_msg' incorporates:
  //   BusCreator: '<S69>/Bus Creator6'

  sim_model_lib0_Y_cvs_landmark_msg_n->cvs_timestamp_sec =
    sim_model_lib0_B->RateTransition3_o;
  sim_model_lib0_Y_cvs_landmark_msg_n->cvs_timestamp_nsec =
    sim_model_lib0_B->RateTransition1_a;
  memcpy(&sim_model_lib0_Y_cvs_landmark_msg_n->cvs_landmarks[0],
         &sim_model_lib0_B->RateTransition5_b[0], (uint32_T)(150U * sizeof
          (real32_T)));
  memcpy(&sim_model_lib0_Y_cvs_landmark_msg_n->cvs_observations[0],
         &sim_model_lib0_B->RateTransition6_n[0], (uint32_T)(100U * sizeof
          (real32_T)));
  for (i = 0; i < 50; i++) {
    sim_model_lib0_Y_cvs_landmark_msg_n->cvs_valid_flag[i] =
      sim_model_lib0_B->RateTransition7_g[i];
  }

  // End of Outport: '<Root>/cvs_landmark_msg'

  // RateTransition: '<S79>/Rate Transition3'
  if (sim_model_lib0_M->Timing.RateInteraction.TID0_1 == 1) {
    sim_model_lib0_B->RateTransition3_e =
      sim_model_lib0_DW->RateTransition3_Buffer0_n;

    // RateTransition: '<S79>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_b =
      sim_model_lib0_DW->RateTransition1_Buffer0_i;

    // RateTransition: '<S79>/Rate Transition5'
    memcpy(&sim_model_lib0_B->RateTransition5_p[0],
           &sim_model_lib0_DW->RateTransition5_Buffer0_f[0], (uint32_T)(150U *
            sizeof(real32_T)));

    // RateTransition: '<S79>/Rate Transition6'
    memcpy(&sim_model_lib0_B->RateTransition6_p[0],
           &sim_model_lib0_DW->RateTransition6_Buffer0_e[0], (uint32_T)(100U *
            sizeof(real32_T)));

    // RateTransition: '<S79>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7_k[i] =
        sim_model_lib0_DW->RateTransition7_Buffer0_j[i];
    }

    // End of RateTransition: '<S79>/Rate Transition7'
  }

  // End of RateTransition: '<S79>/Rate Transition3'

  // BusCreator: '<S69>/Bus Creator2'
  sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_sec =
    sim_model_lib0_B->RateTransition3_j;
  sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_nsec =
    sim_model_lib0_B->RateTransition1_j;
  memcpy(&sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_observations[0],
         &sim_model_lib0_B->RateTransition6[0], (uint32_T)(1600U * sizeof
          (real32_T)));
  memcpy(&sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_valid_flag[0],
         &sim_model_lib0_B->RateTransition7_f[0], (uint32_T)(800U * sizeof
          (uint8_T)));
  memcpy(&sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_id_tag[0],
         &sim_model_lib0_B->RateTransition8[0], (uint32_T)(50U * sizeof(real32_T)));

  // DiscretePulseGenerator: '<S80>/handrail_pulse_gen'
  rtb_Sum = ((real_T)sim_model_lib0_DW->clockTickCounter_n <
             sim_model_lib0_P->handrail_pulse_gen_Duty) &&
    (sim_model_lib0_DW->clockTickCounter_n >= 0) ?
    sim_model_lib0_P->handrail_pulse_gen_Amp : 0.0;
  if ((real_T)sim_model_lib0_DW->clockTickCounter_n >= floor
      (sim_model_lib0_P->cvs_handrail_process_time /
       sim_model_lib0_P->astrobee_time_step_size) - 1.0) {
    sim_model_lib0_DW->clockTickCounter_n = 0;
  } else {
    sim_model_lib0_DW->clockTickCounter_n++;
  }

  // End of DiscretePulseGenerator: '<S80>/handrail_pulse_gen'

  // DataTypeConversion: '<S80>/Data Type Conversion3'
  y_0 = floor(rtb_Sum);
  if (rtIsNaN(y_0) || rtIsInf(y_0)) {
    y_0 = 0.0;
  } else {
    y_0 = fmod(y_0, 256.0);
  }

  // Outport: '<Root>/cvs_registration_pulse' incorporates:
  //   DataTypeConversion: '<S80>/Data Type Conversion1'
  //   DataTypeConversion: '<S80>/Data Type Conversion2'

  sim_model_lib0_Y_cvs_registration_pulse_d->cvs_ar_tag_pulse = (uint8_T)
    (rtb_Product < 0.0 ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)
     -rtb_Product : (int32_T)(uint8_T)rtb_Product);
  sim_model_lib0_Y_cvs_registration_pulse_d->cvs_landmark_pulse = (uint8_T)(y <
    0.0 ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)-y : (int32_T)
    (uint8_T)y);

  // Switch: '<S80>/Switch3' incorporates:
  //   Logic: '<S80>/Logical Operator'
  //   RelationalOperator: '<S110>/FixPt Relational Operator'
  //   RelationalOperator: '<S111>/FixPt Relational Operator'
  //   UnitDelay: '<S110>/Delay Input1'
  //   UnitDelay: '<S111>/Delay Input1'

  if ((sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_sec !=
       sim_model_lib0_DW->DelayInput1_DSTATE_p) ||
      (sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_nsec !=
       sim_model_lib0_DW->DelayInput1_DSTATE_l)) {
    // Outport: '<Root>/cvs_registration_pulse'
    sim_model_lib0_Y_cvs_registration_pulse_d->cvs_optical_flow_pulse =
      sim_model_lib0_B->RateTransition10;
  } else {
    // DataTypeConversion: '<S80>/Data Type Conversion' incorporates:
    //   Constant: '<S80>/Constant1'

    rtb_Product = floor(sim_model_lib0_P->Constant1_Value_ot);
    if (rtIsNaN(rtb_Product) || rtIsInf(rtb_Product)) {
      rtb_Product = 0.0;
    } else {
      rtb_Product = fmod(rtb_Product, 256.0);
    }

    // Outport: '<Root>/cvs_registration_pulse' incorporates:
    //   DataTypeConversion: '<S80>/Data Type Conversion'

    sim_model_lib0_Y_cvs_registration_pulse_d->cvs_optical_flow_pulse = (uint8_T)
      (rtb_Product < 0.0 ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)
       -rtb_Product : (int32_T)(uint8_T)rtb_Product);
  }

  // End of Switch: '<S80>/Switch3'

  // Outport: '<Root>/cvs_registration_pulse' incorporates:
  //   DataTypeConversion: '<S80>/Data Type Conversion3'

  sim_model_lib0_Y_cvs_registration_pulse_d->cvs_handrail_pulse = (uint8_T)(y_0 <
    0.0 ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)-y_0 : (int32_T)
    (uint8_T)y_0);

  // Outport: '<Root>/cvs_ar_tag_msg' incorporates:
  //   BusCreator: '<S69>/Bus Creator1'

  sim_model_lib0_Y_cvs_ar_tag_msg->cvs_timestamp_sec =
    sim_model_lib0_B->RateTransition3_e;
  sim_model_lib0_Y_cvs_ar_tag_msg->cvs_timestamp_nsec =
    sim_model_lib0_B->RateTransition1_b;
  memcpy(&sim_model_lib0_Y_cvs_ar_tag_msg->cvs_landmarks[0],
         &sim_model_lib0_B->RateTransition5_p[0], (uint32_T)(150U * sizeof
          (real32_T)));
  memcpy(&sim_model_lib0_Y_cvs_ar_tag_msg->cvs_observations[0],
         &sim_model_lib0_B->RateTransition6_p[0], (uint32_T)(100U * sizeof
          (real32_T)));
  for (i = 0; i < 50; i++) {
    sim_model_lib0_Y_cvs_ar_tag_msg->cvs_valid_flag[i] =
      sim_model_lib0_B->RateTransition7_k[i];
  }

  // End of Outport: '<Root>/cvs_ar_tag_msg'

  // Outport: '<Root>/ex_time_msg' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'

  sim_model_lib0_Y_ex_time_msg_m->timestamp_sec = rtb_Switch1;
  sim_model_lib0_Y_ex_time_msg_m->timestamp_nsec = rtb_Switch_g;

  // Logic: '<S70>/Logical Operator2' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   Constant: '<S68>/Constant2'
  //   Constant: '<S75>/Constant'
  //   Logic: '<S70>/Logical Operator'
  //   Logic: '<S70>/Logical Operator1'
  //   RelationalOperator: '<S70>/Relational Operator'
  //   RelationalOperator: '<S70>/Relational Operator1'
  //   RelationalOperator: '<S70>/Relational Operator2'
  //   RelationalOperator: '<S75>/Compare'
  //   Selector: '<S70>/Select_col_1'
  //   Selector: '<S70>/Select_col_2'

  for (i_0 = 0; i_0 < 12; i_0++) {
    rtb_LogicalOperator2[i_0] = ((sim_model_lib0_P->mlp_command_times[i_0] !=
      sim_model_lib0_P->Constant_Value_n) &&
      ((sim_model_lib0_P->mlp_command_times[i_0] < rtb_Switch1) ||
       ((sim_model_lib0_P->mlp_command_times[i_0] == rtb_Switch1) &&
        (rtb_Switch_g >= sim_model_lib0_P->mlp_command_times[(int32_T)(12 + i_0)]))));
  }

  // End of Logic: '<S70>/Logical Operator2'

  // Sum: '<S70>/Sum1'
  rtb_Switch1_p_idx_0 = (uint32_T)rtb_LogicalOperator2[0];
  for (i_0 = 0; i_0 < 11; i_0++) {
    rtb_Switch1_p_idx_0 += (uint32_T)rtb_LogicalOperator2[(int32_T)(i_0 + 1)];
  }

  rtb_Bias = (uint8_T)rtb_Switch1_p_idx_0;

  // End of Sum: '<S70>/Sum1'

  // Saturate: '<S70>/Saturation'
  if (rtb_Bias > sim_model_lib0_P->Saturation_UpperSat) {
    // Update for UnitDelay: '<S70>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_i =
      sim_model_lib0_P->Saturation_UpperSat;
  } else if (rtb_Bias < sim_model_lib0_P->Saturation_LowerSat) {
    // Update for UnitDelay: '<S70>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_i =
      sim_model_lib0_P->Saturation_LowerSat;
  } else {
    // Update for UnitDelay: '<S70>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_i = rtb_Bias;
  }

  // End of Saturate: '<S70>/Saturation'

  // Sum: '<S71>/Sum1' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   Constant: '<S68>/Constant3'
  //   Constant: '<S76>/Constant'
  //   Logic: '<S71>/Logical Operator'
  //   Logic: '<S71>/Logical Operator1'
  //   Logic: '<S71>/Logical Operator2'
  //   RelationalOperator: '<S71>/Relational Operator'
  //   RelationalOperator: '<S71>/Relational Operator1'
  //   RelationalOperator: '<S71>/Relational Operator2'
  //   RelationalOperator: '<S76>/Compare'
  //   Selector: '<S71>/Select_col_1'
  //   Selector: '<S71>/Select_col_2'

  rtb_Switch1_p_idx_0 = (uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->mlp_mode_cmd_times[0] !=
      sim_model_lib0_P->Constant_Value_j) &&
     ((sim_model_lib0_P->mlp_mode_cmd_times[0] < rtb_Switch1) ||
      ((sim_model_lib0_P->mlp_mode_cmd_times[0] == rtb_Switch1) && (rtb_Switch_g
    >= sim_model_lib0_P->mlp_mode_cmd_times[3])))) + (uint32_T)
    ((sim_model_lib0_P->mlp_mode_cmd_times[1] !=
      sim_model_lib0_P->Constant_Value_j) &&
     ((sim_model_lib0_P->mlp_mode_cmd_times[1] < rtb_Switch1) ||
      ((sim_model_lib0_P->mlp_mode_cmd_times[1] == rtb_Switch1) && (rtb_Switch_g
    >= sim_model_lib0_P->mlp_mode_cmd_times[4]))))) + (uint32_T)
    ((sim_model_lib0_P->mlp_mode_cmd_times[2] !=
      sim_model_lib0_P->Constant_Value_j) &&
     ((sim_model_lib0_P->mlp_mode_cmd_times[2] < rtb_Switch1) ||
      ((sim_model_lib0_P->mlp_mode_cmd_times[2] == rtb_Switch1) && (rtb_Switch_g
    >= sim_model_lib0_P->mlp_mode_cmd_times[5])))));

  // Saturate: '<S71>/Saturation' incorporates:
  //   Sum: '<S71>/Sum1'

  if ((uint8_T)rtb_Switch1_p_idx_0 > sim_model_lib0_P->Saturation_UpperSat_b) {
    // Update for UnitDelay: '<S71>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_if =
      sim_model_lib0_P->Saturation_UpperSat_b;
  } else if ((uint8_T)rtb_Switch1_p_idx_0 <
             sim_model_lib0_P->Saturation_LowerSat_e) {
    // Update for UnitDelay: '<S71>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_if =
      sim_model_lib0_P->Saturation_LowerSat_e;
  } else {
    // Update for UnitDelay: '<S71>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_if = (uint8_T)rtb_Switch1_p_idx_0;
  }

  // End of Saturate: '<S71>/Saturation'

  // Sum: '<S72>/Sum1' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   Constant: '<S68>/Constant5'
  //   Constant: '<S77>/Constant'
  //   Logic: '<S72>/Logical Operator'
  //   Logic: '<S72>/Logical Operator1'
  //   Logic: '<S72>/Logical Operator2'
  //   RelationalOperator: '<S72>/Relational Operator'
  //   RelationalOperator: '<S72>/Relational Operator1'
  //   RelationalOperator: '<S72>/Relational Operator2'
  //   RelationalOperator: '<S77>/Compare'
  //   Selector: '<S72>/Select_col_1'
  //   Selector: '<S72>/Select_col_2'

  rtb_Switch1_p_idx_0 = (uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->mlp_loc_mode_cmd_times[0] !=
      sim_model_lib0_P->Constant_Value_m) &&
     ((sim_model_lib0_P->mlp_loc_mode_cmd_times[0] < rtb_Switch1) ||
      ((sim_model_lib0_P->mlp_loc_mode_cmd_times[0] == rtb_Switch1) &&
       (rtb_Switch_g >= sim_model_lib0_P->mlp_loc_mode_cmd_times[3])))) +
    (uint32_T)((sim_model_lib0_P->mlp_loc_mode_cmd_times[1] !=
                sim_model_lib0_P->Constant_Value_m) &&
               ((sim_model_lib0_P->mlp_loc_mode_cmd_times[1] < rtb_Switch1) ||
                ((sim_model_lib0_P->mlp_loc_mode_cmd_times[1] == rtb_Switch1) &&
                 (rtb_Switch_g >= sim_model_lib0_P->mlp_loc_mode_cmd_times[4])))))
    + (uint32_T)((sim_model_lib0_P->mlp_loc_mode_cmd_times[2] !=
                  sim_model_lib0_P->Constant_Value_m) &&
                 ((sim_model_lib0_P->mlp_loc_mode_cmd_times[2] < rtb_Switch1) ||
                  ((sim_model_lib0_P->mlp_loc_mode_cmd_times[2] == rtb_Switch1) &&
                   (rtb_Switch_g >= sim_model_lib0_P->mlp_loc_mode_cmd_times[5])))));

  // Saturate: '<S72>/Saturation' incorporates:
  //   Sum: '<S72>/Sum1'

  if ((uint8_T)rtb_Switch1_p_idx_0 > sim_model_lib0_P->Saturation_UpperSat_d) {
    // Update for UnitDelay: '<S72>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_ht =
      sim_model_lib0_P->Saturation_UpperSat_d;
  } else if ((uint8_T)rtb_Switch1_p_idx_0 <
             sim_model_lib0_P->Saturation_LowerSat_j) {
    // Update for UnitDelay: '<S72>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_ht =
      sim_model_lib0_P->Saturation_LowerSat_j;
  } else {
    // Update for UnitDelay: '<S72>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_ht = (uint8_T)rtb_Switch1_p_idx_0;
  }

  // End of Saturate: '<S72>/Saturation'

  // Sum: '<S73>/Sum1' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   Constant: '<S68>/Constant7'
  //   Constant: '<S78>/Constant'
  //   Logic: '<S73>/Logical Operator'
  //   Logic: '<S73>/Logical Operator1'
  //   Logic: '<S73>/Logical Operator2'
  //   RelationalOperator: '<S73>/Relational Operator'
  //   RelationalOperator: '<S73>/Relational Operator1'
  //   RelationalOperator: '<S73>/Relational Operator2'
  //   RelationalOperator: '<S78>/Compare'
  //   Selector: '<S73>/Select_col_1'
  //   Selector: '<S73>/Select_col_2'

  rtb_Switch1_p_idx_0 = (uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->mlp_speed_gain_cmd_times[0] !=
      sim_model_lib0_P->Constant_Value_nc) &&
     ((sim_model_lib0_P->mlp_speed_gain_cmd_times[0] < rtb_Switch1) ||
      ((sim_model_lib0_P->mlp_speed_gain_cmd_times[0] == rtb_Switch1) &&
       (rtb_Switch_g >= sim_model_lib0_P->mlp_speed_gain_cmd_times[3])))) +
    (uint32_T)((sim_model_lib0_P->mlp_speed_gain_cmd_times[1] !=
                sim_model_lib0_P->Constant_Value_nc) &&
               ((sim_model_lib0_P->mlp_speed_gain_cmd_times[1] < rtb_Switch1) ||
                ((sim_model_lib0_P->mlp_speed_gain_cmd_times[1] == rtb_Switch1) &&
                 (rtb_Switch_g >= sim_model_lib0_P->mlp_speed_gain_cmd_times[4])))))
    + (uint32_T)((sim_model_lib0_P->mlp_speed_gain_cmd_times[2] !=
                  sim_model_lib0_P->Constant_Value_nc) &&
                 ((sim_model_lib0_P->mlp_speed_gain_cmd_times[2] < rtb_Switch1) ||
                  ((sim_model_lib0_P->mlp_speed_gain_cmd_times[2] == rtb_Switch1)
                   && (rtb_Switch_g >=
                       sim_model_lib0_P->mlp_speed_gain_cmd_times[5])))));

  // RateTransition: '<S79>/Rate Transition2' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  if (sim_model_lib0_M->Timing.RateInteraction.TID0_1 == 1) {
    sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];
    sim_model_lib0_B->RateTransition2.V_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
    sim_model_lib0_B->RateTransition2.A_B_ISS_ISS[0] = rtb_P_B_ISS_SS[0];
    sim_model_lib0_B->RateTransition2.A_B_ISS_B[0] = rtb_Merge_o[0];
    sim_model_lib0_B->RateTransition2.A_B_ECI_B[0] = rtb_imu_gyro_bias[0];
    sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
    sim_model_lib0_B->RateTransition2.V_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
    sim_model_lib0_B->RateTransition2.A_B_ISS_ISS[1] = rtb_P_B_ISS_SS[1];
    sim_model_lib0_B->RateTransition2.A_B_ISS_B[1] = rtb_Merge_o[1];
    sim_model_lib0_B->RateTransition2.A_B_ECI_B[1] = rtb_imu_gyro_bias[1];
    sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
    sim_model_lib0_B->RateTransition2.V_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
    sim_model_lib0_B->RateTransition2.A_B_ISS_ISS[2] = rtb_Merge_c;
    sim_model_lib0_B->RateTransition2.A_B_ISS_B[2] = rtb_Merge_o[2];
    sim_model_lib0_B->RateTransition2.A_B_ECI_B[2] = rtb_imu_gyro_bias[2];
    sim_model_lib0_B->RateTransition2.Q_ISS2B[0] = rtb_Merge_m[0];
    sim_model_lib0_B->RateTransition2.Q_ISS2B[1] = rtb_Merge_m[1];
    sim_model_lib0_B->RateTransition2.Q_ISS2B[2] = rtb_Merge_m[2];
    sim_model_lib0_B->RateTransition2.Q_ISS2B[3] = rtb_Merge_m[3];
    sim_model_lib0_B->RateTransition2.omega_B_ISS_B[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    sim_model_lib0_B->RateTransition2.omega_B_ECI_B[0] = rtb_Sum2_f[0];
    sim_model_lib0_B->RateTransition2.alpha_B_ISS_B[0] = rtb_alpha_B_ECI_B[0];
    sim_model_lib0_B->RateTransition2.fan_torques_B[0] =
      rtb_Delay1_bpm_torque_B[0];
    sim_model_lib0_B->RateTransition2.fan_forces_B[0] =
      rtb_TrigonometricFunction1_c;
    sim_model_lib0_B->RateTransition2.omega_B_ISS_B[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];
    sim_model_lib0_B->RateTransition2.omega_B_ECI_B[1] = rtb_Sum2_f[1];
    sim_model_lib0_B->RateTransition2.alpha_B_ISS_B[1] = rtb_alpha_B_ECI_B[1];
    sim_model_lib0_B->RateTransition2.fan_torques_B[1] =
      rtb_Delay1_bpm_torque_B[1];
    sim_model_lib0_B->RateTransition2.fan_forces_B[1] =
      rtb_Delay1_bpm_force_B_idx_1;
    sim_model_lib0_B->RateTransition2.omega_B_ISS_B[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    sim_model_lib0_B->RateTransition2.omega_B_ECI_B[2] = rtb_Sum2_f[2];
    sim_model_lib0_B->RateTransition2.alpha_B_ISS_B[2] = rtb_alpha_B_ECI_B[2];
    sim_model_lib0_B->RateTransition2.fan_torques_B[2] =
      rtb_Delay1_bpm_torque_B[2];
    sim_model_lib0_B->RateTransition2.fan_forces_B[2] =
      rtb_Delay1_bpm_force_B_idx_2;

    // RateTransition: '<S79>/Rate Transition4' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   BusCreator: '<S2>/Bus Creator1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

    sim_model_lib0_B->RateTransition4.timestamp_sec = rtb_Switch1;
    sim_model_lib0_B->RateTransition4.timestamp_nsec = rtb_Switch_g;
  }

  // End of RateTransition: '<S79>/Rate Transition2'

  // RateTransition: '<S81>/Rate Transition2' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  if (sim_model_lib0_M->Timing.RateInteraction.TID0_2 == 1) {
    sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];
    sim_model_lib0_B->RateTransition2_o.V_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_ISS[0] = rtb_P_B_ISS_SS[0];
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_B[0] = rtb_Merge_o[0];
    sim_model_lib0_B->RateTransition2_o.A_B_ECI_B[0] = rtb_imu_gyro_bias[0];
    sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
    sim_model_lib0_B->RateTransition2_o.V_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_ISS[1] = rtb_P_B_ISS_SS[1];
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_B[1] = rtb_Merge_o[1];
    sim_model_lib0_B->RateTransition2_o.A_B_ECI_B[1] = rtb_imu_gyro_bias[1];
    sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
    sim_model_lib0_B->RateTransition2_o.V_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_ISS[2] = rtb_Merge_c;
    sim_model_lib0_B->RateTransition2_o.A_B_ISS_B[2] = rtb_Merge_o[2];
    sim_model_lib0_B->RateTransition2_o.A_B_ECI_B[2] = rtb_imu_gyro_bias[2];
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[0] = rtb_Merge_m[0];
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[1] = rtb_Merge_m[1];
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[2] = rtb_Merge_m[2];
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[3] = rtb_Merge_m[3];
    sim_model_lib0_B->RateTransition2_o.omega_B_ISS_B[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    sim_model_lib0_B->RateTransition2_o.omega_B_ECI_B[0] = rtb_Sum2_f[0];
    sim_model_lib0_B->RateTransition2_o.alpha_B_ISS_B[0] = rtb_alpha_B_ECI_B[0];
    sim_model_lib0_B->RateTransition2_o.fan_torques_B[0] =
      rtb_Delay1_bpm_torque_B[0];
    sim_model_lib0_B->RateTransition2_o.fan_forces_B[0] =
      rtb_TrigonometricFunction1_c;
    sim_model_lib0_B->RateTransition2_o.omega_B_ISS_B[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];
    sim_model_lib0_B->RateTransition2_o.omega_B_ECI_B[1] = rtb_Sum2_f[1];
    sim_model_lib0_B->RateTransition2_o.alpha_B_ISS_B[1] = rtb_alpha_B_ECI_B[1];
    sim_model_lib0_B->RateTransition2_o.fan_torques_B[1] =
      rtb_Delay1_bpm_torque_B[1];
    sim_model_lib0_B->RateTransition2_o.fan_forces_B[1] =
      rtb_Delay1_bpm_force_B_idx_1;
    sim_model_lib0_B->RateTransition2_o.omega_B_ISS_B[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    sim_model_lib0_B->RateTransition2_o.omega_B_ECI_B[2] = rtb_Sum2_f[2];
    sim_model_lib0_B->RateTransition2_o.alpha_B_ISS_B[2] = rtb_alpha_B_ECI_B[2];
    sim_model_lib0_B->RateTransition2_o.fan_torques_B[2] =
      rtb_Delay1_bpm_torque_B[2];
    sim_model_lib0_B->RateTransition2_o.fan_forces_B[2] =
      rtb_Delay1_bpm_force_B_idx_2;

    // RateTransition: '<S81>/Rate Transition4' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   BusCreator: '<S2>/Bus Creator1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

    sim_model_lib0_B->RateTransition4_m.timestamp_sec = rtb_Switch1;
    sim_model_lib0_B->RateTransition4_m.timestamp_nsec = rtb_Switch_g;
  }

  // End of RateTransition: '<S81>/Rate Transition2'

  // RateTransition: '<S82>/Rate Transition2' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  if (sim_model_lib0_M->Timing.RateInteraction.TID0_4 == 1) {
    sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];
    sim_model_lib0_B->RateTransition2_j.V_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_ISS[0] = rtb_P_B_ISS_SS[0];
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_B[0] = rtb_Merge_o[0];
    sim_model_lib0_B->RateTransition2_j.A_B_ECI_B[0] = rtb_imu_gyro_bias[0];
    sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
    sim_model_lib0_B->RateTransition2_j.V_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_ISS[1] = rtb_P_B_ISS_SS[1];
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_B[1] = rtb_Merge_o[1];
    sim_model_lib0_B->RateTransition2_j.A_B_ECI_B[1] = rtb_imu_gyro_bias[1];
    sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
    sim_model_lib0_B->RateTransition2_j.V_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_ISS[2] = rtb_Merge_c;
    sim_model_lib0_B->RateTransition2_j.A_B_ISS_B[2] = rtb_Merge_o[2];
    sim_model_lib0_B->RateTransition2_j.A_B_ECI_B[2] = rtb_imu_gyro_bias[2];
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[0] = rtb_Merge_m[0];
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[1] = rtb_Merge_m[1];
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[2] = rtb_Merge_m[2];
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[3] = rtb_Merge_m[3];
    sim_model_lib0_B->RateTransition2_j.omega_B_ISS_B[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    sim_model_lib0_B->RateTransition2_j.omega_B_ECI_B[0] = rtb_Sum2_f[0];
    sim_model_lib0_B->RateTransition2_j.alpha_B_ISS_B[0] = rtb_alpha_B_ECI_B[0];
    sim_model_lib0_B->RateTransition2_j.fan_torques_B[0] =
      rtb_Delay1_bpm_torque_B[0];
    sim_model_lib0_B->RateTransition2_j.fan_forces_B[0] =
      rtb_TrigonometricFunction1_c;
    sim_model_lib0_B->RateTransition2_j.omega_B_ISS_B[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];
    sim_model_lib0_B->RateTransition2_j.omega_B_ECI_B[1] = rtb_Sum2_f[1];
    sim_model_lib0_B->RateTransition2_j.alpha_B_ISS_B[1] = rtb_alpha_B_ECI_B[1];
    sim_model_lib0_B->RateTransition2_j.fan_torques_B[1] =
      rtb_Delay1_bpm_torque_B[1];
    sim_model_lib0_B->RateTransition2_j.fan_forces_B[1] =
      rtb_Delay1_bpm_force_B_idx_1;
    sim_model_lib0_B->RateTransition2_j.omega_B_ISS_B[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    sim_model_lib0_B->RateTransition2_j.omega_B_ECI_B[2] = rtb_Sum2_f[2];
    sim_model_lib0_B->RateTransition2_j.alpha_B_ISS_B[2] = rtb_alpha_B_ECI_B[2];
    sim_model_lib0_B->RateTransition2_j.fan_torques_B[2] =
      rtb_Delay1_bpm_torque_B[2];
    sim_model_lib0_B->RateTransition2_j.fan_forces_B[2] =
      rtb_Delay1_bpm_force_B_idx_2;

    // RateTransition: '<S82>/Rate Transition4' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   BusCreator: '<S2>/Bus Creator1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

    sim_model_lib0_B->RateTransition4_e.timestamp_sec = rtb_Switch1;
    sim_model_lib0_B->RateTransition4_e.timestamp_nsec = rtb_Switch_g;
  }

  // End of RateTransition: '<S82>/Rate Transition2'

  // RateTransition: '<S83>/Rate Transition2' incorporates:
  //   BusCreator: '<S2>/Bus Creator'
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

  if (sim_model_lib0_M->Timing.RateInteraction.TID0_3 == 1) {
    sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0];
    sim_model_lib0_B->RateTransition2_b.V_B_ISS_ISS[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_ISS[0] = rtb_P_B_ISS_SS[0];
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_B[0] = rtb_Merge_o[0];
    sim_model_lib0_B->RateTransition2_b.A_B_ECI_B[0] = rtb_imu_gyro_bias[0];
    sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1];
    sim_model_lib0_B->RateTransition2_b.V_B_ISS_ISS[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_ISS[1] = rtb_P_B_ISS_SS[1];
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_B[1] = rtb_Merge_o[1];
    sim_model_lib0_B->RateTransition2_b.A_B_ECI_B[1] = rtb_imu_gyro_bias[1];
    sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2];
    sim_model_lib0_B->RateTransition2_b.V_B_ISS_ISS[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_ISS[2] = rtb_Merge_c;
    sim_model_lib0_B->RateTransition2_b.A_B_ISS_B[2] = rtb_Merge_o[2];
    sim_model_lib0_B->RateTransition2_b.A_B_ECI_B[2] = rtb_imu_gyro_bias[2];
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[0] = rtb_Merge_m[0];
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[1] = rtb_Merge_m[1];
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[2] = rtb_Merge_m[2];
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[3] = rtb_Merge_m[3];
    sim_model_lib0_B->RateTransition2_b.omega_B_ISS_B[0] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0];
    sim_model_lib0_B->RateTransition2_b.omega_B_ECI_B[0] = rtb_Sum2_f[0];
    sim_model_lib0_B->RateTransition2_b.alpha_B_ISS_B[0] = rtb_alpha_B_ECI_B[0];
    sim_model_lib0_B->RateTransition2_b.fan_torques_B[0] =
      rtb_Delay1_bpm_torque_B[0];
    sim_model_lib0_B->RateTransition2_b.fan_forces_B[0] =
      rtb_TrigonometricFunction1_c;
    sim_model_lib0_B->RateTransition2_b.omega_B_ISS_B[1] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1];
    sim_model_lib0_B->RateTransition2_b.omega_B_ECI_B[1] = rtb_Sum2_f[1];
    sim_model_lib0_B->RateTransition2_b.alpha_B_ISS_B[1] = rtb_alpha_B_ECI_B[1];
    sim_model_lib0_B->RateTransition2_b.fan_torques_B[1] =
      rtb_Delay1_bpm_torque_B[1];
    sim_model_lib0_B->RateTransition2_b.fan_forces_B[1] =
      rtb_Delay1_bpm_force_B_idx_1;
    sim_model_lib0_B->RateTransition2_b.omega_B_ISS_B[2] =
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2];
    sim_model_lib0_B->RateTransition2_b.omega_B_ECI_B[2] = rtb_Sum2_f[2];
    sim_model_lib0_B->RateTransition2_b.alpha_B_ISS_B[2] = rtb_alpha_B_ECI_B[2];
    sim_model_lib0_B->RateTransition2_b.fan_torques_B[2] =
      rtb_Delay1_bpm_torque_B[2];
    sim_model_lib0_B->RateTransition2_b.fan_forces_B[2] =
      rtb_Delay1_bpm_force_B_idx_2;

    // RateTransition: '<S83>/Rate Transition4' incorporates:
    //   BusCreator: '<S2>/Bus Creator'
    //   BusCreator: '<S2>/Bus Creator1'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'

    sim_model_lib0_B->RateTransition4_f.timestamp_sec = rtb_Switch1;
    sim_model_lib0_B->RateTransition4_f.timestamp_nsec = rtb_Switch_g;
  }

  // End of RateTransition: '<S83>/Rate Transition2'

  // Sum: '<S191>/Add'
  rtb_Sqrt_m = sim_model_lib0_B->dc_motor_model.output_torque -
    sim_model_lib0_B->blower_aerodynamics.Constant7;

  // Switch: '<S191>/Switch' incorporates:
  //   Constant: '<S191>/Constant'
  //   Gain: '<S191>/Gain'
  //   Gain: '<S191>/Gain2'

  if (sim_model_lib0_P->tun_bpm_noise_on_flag >
      sim_model_lib0_P->Switch_Threshold) {
    rtb_Sqrt_m *= 1.0F / sim_model_lib0_P->bpm_impeller_inertia;
  } else {
    rtb_Sqrt_m *= 1.0F / (sim_model_lib0_P->bpm_impeller_inertia +
                          sim_model_lib0_P->bpm_impeller_inertia_error);
  }

  // End of Switch: '<S191>/Switch'

  // Sum: '<S224>/Add'
  rtb_Product_p = sim_model_lib0_B->dc_motor_model_g.output_torque -
    sim_model_lib0_B->blower_aerodynamics_j.Constant7;

  // Switch: '<S224>/Switch' incorporates:
  //   Constant: '<S224>/Constant'
  //   Gain: '<S224>/Gain'
  //   Gain: '<S224>/Gain2'

  if (sim_model_lib0_P->tun_bpm_noise_on_flag >
      sim_model_lib0_P->Switch_Threshold_o) {
    rtb_Product_p *= 1.0F / sim_model_lib0_P->bpm_impeller_inertia;
  } else {
    rtb_Product_p *= 1.0F / (sim_model_lib0_P->bpm_impeller_inertia +
      sim_model_lib0_P->bpm_impeller_inertia_error);
  }

  // End of Switch: '<S224>/Switch'

  // Gain: '<S254>/Gain1' incorporates:
  //   RandomNumber: '<S254>/random_noise1'

  rtb_Sum = 1.0 / sqrt(sim_model_lib0_P->astrobee_time_step_size);
  rtb_random_noise[0] = rtb_Sum * sim_model_lib0_DW->NextOutput_i[0];
  rtb_random_noise[1] = rtb_Sum * sim_model_lib0_DW->NextOutput_i[1];
  rtb_random_noise[2] = rtb_Sum * sim_model_lib0_DW->NextOutput_i[2];

  // Gain: '<S255>/Gain4'
  rtb_Sum = 1.0 / sqrt(sim_model_lib0_P->astrobee_time_step_size);

  // Saturate: '<S73>/Saturation' incorporates:
  //   Sum: '<S73>/Sum1'

  if ((uint8_T)rtb_Switch1_p_idx_0 > sim_model_lib0_P->Saturation_UpperSat_f) {
    // Update for UnitDelay: '<S73>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_h =
      sim_model_lib0_P->Saturation_UpperSat_f;
  } else if ((uint8_T)rtb_Switch1_p_idx_0 <
             sim_model_lib0_P->Saturation_LowerSat_k) {
    // Update for UnitDelay: '<S73>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_h =
      sim_model_lib0_P->Saturation_LowerSat_k;
  } else {
    // Update for UnitDelay: '<S73>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_h = (uint8_T)rtb_Switch1_p_idx_0;
  }

  // End of Saturate: '<S73>/Saturation'

  // Update for UnitDelay: '<S8>/Unit Delay1'
  sim_model_lib0_DW->UnitDelay1_DSTATE = rtb_Switch1;

  // Constant: '<S8>/Constant'
  rtb_Product = 1.0E+9 * sim_model_lib0_P->astrobee_time_step_size;
  y = fabs(rtb_Product);
  if (y < 4.503599627370496E+15) {
    if (y >= 0.5) {
      rtb_Product = floor(rtb_Product + 0.5);
    } else {
      rtb_Product *= 0.0;
    }
  }

  if (rtb_Product < 4.294967296E+9) {
    if (rtb_Product >= 0.0) {
      rtb_Switch1_p_idx_0 = (uint32_T)rtb_Product;
    } else {
      rtb_Switch1_p_idx_0 = 0U;
    }
  } else {
    rtb_Switch1_p_idx_0 = MAX_uint32_T;
  }

  // End of Constant: '<S8>/Constant'

  // Update for UnitDelay: '<S8>/Unit Delay' incorporates:
  //   Sum: '<S8>/Sum'

  sim_model_lib0_DW->UnitDelay_DSTATE = (uint32_T)(rtb_Switch1_p_idx_0 +
    rtb_Switch_g);

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_i *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0];

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainval * rtb_P_B_ISS_SS[0];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_p * rtb_alpha_B_ECI_B[0];

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_i *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1];

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainval * rtb_P_B_ISS_SS[1];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_p * rtb_alpha_B_ECI_B[1];

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_i *
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2];

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainval * rtb_Merge_c;

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_p * rtb_alpha_B_ECI_B[2];

  // Update for Delay: '<S11>/Delay'
  sim_model_lib0_DW->icLoad = 0U;
  sim_model_lib0_DW->Delay_DSTATE[0] = rtb_Merge_kq[0];
  sim_model_lib0_DW->Delay_DSTATE[1] = rtb_Merge_kq[1];
  sim_model_lib0_DW->Delay_DSTATE[2] = rtb_Merge_kq[2];
  sim_model_lib0_DW->Delay_DSTATE[3] = rtb_Merge_kq[3];

  // Update for Delay: '<S1>/Delay1' incorporates:
  //   BusCreator: '<S2>/Bus Creator1'
  //   BusCreator: '<S4>/Bus Creator'
  //   DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  //   SignalConversion: '<S4>/ConcatBufferAtVector ConcatenateIn2'

  sim_model_lib0_DW->Delay1_DSTATE.bpm_timestamp_sec = rtb_Switch1;
  sim_model_lib0_DW->Delay1_DSTATE.bpm_timestamp_nsec = rtb_Switch_g;
  sim_model_lib0_DW->Delay1_DSTATE.bpm_motor_curr[0] = rtb_bpm_motor_curr_idx_0;
  sim_model_lib0_DW->Delay1_DSTATE.bpm_motor_curr[1] =
    sim_model_lib0_B->dc_motor_model_g.current;
  sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[0] = rtb_bpm_torque_B[0];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[0] = rtb_bpm_force_B[0];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[1] = rtb_bpm_torque_B[1];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[1] = rtb_bpm_force_B[1];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_torque_B[2] = rtb_bpm_torque_B[2];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_force_B[2] = rtb_bpm_force_B[2];
  sim_model_lib0_DW->Delay1_DSTATE.bpm_motor_speed[0] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp;
  sim_model_lib0_DW->Delay1_DSTATE.bpm_motor_speed[1] =
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e;
  for (i = 0; i < 12; i++) {
    sim_model_lib0_DW->Delay1_DSTATE.bpm_servo_curr[i] = rtb_bpm_servo_curr[i];
    sim_model_lib0_DW->Delay1_DSTATE.bpm_nozzle_theta[i] =
      rtb_bpm_nozzle_theta[i];
  }

  // End of Update for Delay: '<S1>/Delay1'

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'
  //   RandomNumber: '<S6>/Random Number'

  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_j * (real32_T)
    sim_model_lib0_DW->NextOutput_l[0];
  if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] >=
      sim_model_lib0_P->env_max_ext_air_vel) {
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] =
      sim_model_lib0_P->env_max_ext_air_vel;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] <=
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] =
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F);
    }
  }

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   RandomNumber: '<S6>/Random Number1'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_a * (real32_T)
    sim_model_lib0_DW->NextOutput_f[0];

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'
  //   RandomNumber: '<S6>/Random Number'

  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_j * (real32_T)
    sim_model_lib0_DW->NextOutput_l[1];
  if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] >=
      sim_model_lib0_P->env_max_ext_air_vel) {
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] =
      sim_model_lib0_P->env_max_ext_air_vel;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] <=
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] =
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F);
    }
  }

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   RandomNumber: '<S6>/Random Number1'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_a * (real32_T)
    sim_model_lib0_DW->NextOutput_f[1];

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion'
  //   RandomNumber: '<S6>/Random Number'

  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_j * (real32_T)
    sim_model_lib0_DW->NextOutput_l[2];
  if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] >=
      sim_model_lib0_P->env_max_ext_air_vel) {
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] =
      sim_model_lib0_P->env_max_ext_air_vel;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] <=
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] =
        -(sim_model_lib0_P->env_max_ext_air_vel + 2.22044605E-16F);
    }
  }

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   RandomNumber: '<S6>/Random Number1'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_a * (real32_T)
    sim_model_lib0_DW->NextOutput_f[2];

  // Update for DiscreteIntegrator: '<S191>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainv_j1 * rtb_Sqrt_m;

  // Update for DiscreteIntegrator: '<S224>/Discrete-Time Integrator'
  sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e +=
    sim_model_lib0_P->DiscreteTimeIntegrator_gainva_o * rtb_Product_p;

  // Update for UnitDelay: '<S188>/Unit Delay' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'
  //   Logic: '<S188>/Logical Operator'
  //   RelationalOperator: '<S194>/FixPt Relational Operator'
  //   UnitDelay: '<S194>/Delay Input1'

  sim_model_lib0_DW->UnitDelay_DSTATE_m =
    ((sim_model_lib0_P->tun_default_center_of_mass[0] !=
      sim_model_lib0_DW->DelayInput1_DSTATE[0]) ||
     (sim_model_lib0_P->tun_default_center_of_mass[1] !=
      sim_model_lib0_DW->DelayInput1_DSTATE[1]) ||
     (sim_model_lib0_P->tun_default_center_of_mass[2] !=
      sim_model_lib0_DW->DelayInput1_DSTATE[2]));

  // Update for UnitDelay: '<S221>/Unit Delay' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'
  //   Logic: '<S221>/Logical Operator'
  //   RelationalOperator: '<S227>/FixPt Relational Operator'
  //   UnitDelay: '<S227>/Delay Input1'

  sim_model_lib0_DW->UnitDelay_DSTATE_c =
    ((sim_model_lib0_P->tun_default_center_of_mass[0] !=
      sim_model_lib0_DW->DelayInput1_DSTATE_m[0]) ||
     (sim_model_lib0_P->tun_default_center_of_mass[1] !=
      sim_model_lib0_DW->DelayInput1_DSTATE_m[1]) ||
     (sim_model_lib0_P->tun_default_center_of_mass[2] !=
      sim_model_lib0_DW->DelayInput1_DSTATE_m[2]));

  // Update for UnitDelay: '<S110>/Delay Input1'
  sim_model_lib0_DW->DelayInput1_DSTATE_p =
    sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_sec;

  // Update for UnitDelay: '<S111>/Delay Input1'
  sim_model_lib0_DW->DelayInput1_DSTATE_l =
    sim_model_lib0_Y_cvs_optical_flow_msg_n->cvs_timestamp_nsec;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] >=
      sim_model_lib0_P->env_max_ext_air_omega) {
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] =
      sim_model_lib0_P->env_max_ext_air_omega;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] <=
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] =
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F);
    }
  }

  // Update for RandomNumber: '<S254>/random_noise'
  sim_model_lib0_DW->NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed[0]) * sim_model_lib0_P->random_noise_StdDev +
    sim_model_lib0_P->random_noise_Mean;

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2'
  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[0] +=
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainval * rtb_random_noise[0];

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S254>/Discrete-Time Integrator1'
  //   Gain: '<S254>/Gain2'
  //   RandomNumber: '<S254>/random_noise2'
  //   Sum: '<S254>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[0] +=
    (sim_model_lib0_DW->NextOutput_j[0] -
     sim_model_lib0_P->epson_accel_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[0]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainval;

  // Update for RandomNumber: '<S255>/random_noise'
  sim_model_lib0_DW->NextOutput_e[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_k[0]) *
    sim_model_lib0_P->random_noise_StdDev_c +
    sim_model_lib0_P->random_noise_Mean_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' incorporates:
  //   Gain: '<S255>/Gain4'
  //   RandomNumber: '<S255>/random_noise1'

  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[0] += rtb_Sum *
    sim_model_lib0_DW->NextOutput_h[0] *
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainv_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S255>/Discrete-Time Integrator1'
  //   Gain: '<S255>/Gain2'
  //   RandomNumber: '<S255>/random_noise2'
  //   Sum: '<S255>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[0] +=
    (sim_model_lib0_DW->NextOutput_p[0] -
     sim_model_lib0_P->epson_gyro_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[0]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_l;

  // Update for RandomNumber: '<S6>/Random Number'
  sim_model_lib0_DW->NextOutput_l[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_i[0]) * sqrt
    (sim_model_lib0_P->env_ext_air_vel_variance[0]) +
    sim_model_lib0_P->RandomNumber_Mean;

  // Update for RandomNumber: '<S6>/Random Number1'
  sim_model_lib0_DW->NextOutput_f[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_m[0]) * sqrt
    (sim_model_lib0_P->env_ext_air_omega_variance[0]) +
    sim_model_lib0_P->RandomNumber1_Mean;

  // Update for UnitDelay: '<S194>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE[0] =
    sim_model_lib0_P->tun_default_center_of_mass[0];

  // Update for UnitDelay: '<S227>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE_m[0] =
    sim_model_lib0_P->tun_default_center_of_mass[0];

  // Update for RandomNumber: '<S254>/random_noise1'
  sim_model_lib0_DW->NextOutput_i[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_ip[0]) *
    sim_model_lib0_P->random_noise1_StdDev +
    sim_model_lib0_P->random_noise1_Mean;

  // Update for RandomNumber: '<S254>/random_noise2'
  sim_model_lib0_DW->NextOutput_j[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_e[0]) * sim_model_lib0_P->random_noise2_StdDev
    + sim_model_lib0_P->random_noise2_Mean;

  // Update for RandomNumber: '<S255>/random_noise1'
  sim_model_lib0_DW->NextOutput_h[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_p[0]) *
    sim_model_lib0_P->random_noise1_StdDev_l +
    sim_model_lib0_P->random_noise1_Mean_d;

  // Update for RandomNumber: '<S255>/random_noise2'
  sim_model_lib0_DW->NextOutput_p[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_n[0]) *
    sim_model_lib0_P->random_noise2_StdDev_k +
    sim_model_lib0_P->random_noise2_Mean_f;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] >=
      sim_model_lib0_P->env_max_ext_air_omega) {
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] =
      sim_model_lib0_P->env_max_ext_air_omega;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] <=
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] =
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F);
    }
  }

  // Update for RandomNumber: '<S254>/random_noise'
  sim_model_lib0_DW->NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed[1]) * sim_model_lib0_P->random_noise_StdDev +
    sim_model_lib0_P->random_noise_Mean;

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2'
  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[1] +=
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainval * rtb_random_noise[1];

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S254>/Discrete-Time Integrator1'
  //   Gain: '<S254>/Gain2'
  //   RandomNumber: '<S254>/random_noise2'
  //   Sum: '<S254>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[1] +=
    (sim_model_lib0_DW->NextOutput_j[1] -
     sim_model_lib0_P->epson_accel_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[1]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainval;

  // Update for RandomNumber: '<S255>/random_noise'
  sim_model_lib0_DW->NextOutput_e[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_k[1]) *
    sim_model_lib0_P->random_noise_StdDev_c +
    sim_model_lib0_P->random_noise_Mean_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' incorporates:
  //   Gain: '<S255>/Gain4'
  //   RandomNumber: '<S255>/random_noise1'

  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[1] += rtb_Sum *
    sim_model_lib0_DW->NextOutput_h[1] *
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainv_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S255>/Discrete-Time Integrator1'
  //   Gain: '<S255>/Gain2'
  //   RandomNumber: '<S255>/random_noise2'
  //   Sum: '<S255>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[1] +=
    (sim_model_lib0_DW->NextOutput_p[1] -
     sim_model_lib0_P->epson_gyro_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[1]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_l;

  // Update for RandomNumber: '<S6>/Random Number'
  sim_model_lib0_DW->NextOutput_l[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_i[1]) * sqrt
    (sim_model_lib0_P->env_ext_air_vel_variance[1]) +
    sim_model_lib0_P->RandomNumber_Mean;

  // Update for RandomNumber: '<S6>/Random Number1'
  sim_model_lib0_DW->NextOutput_f[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_m[1]) * sqrt
    (sim_model_lib0_P->env_ext_air_omega_variance[1]) +
    sim_model_lib0_P->RandomNumber1_Mean;

  // Update for UnitDelay: '<S194>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE[1] =
    sim_model_lib0_P->tun_default_center_of_mass[1];

  // Update for UnitDelay: '<S227>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE_m[1] =
    sim_model_lib0_P->tun_default_center_of_mass[1];

  // Update for RandomNumber: '<S254>/random_noise1'
  sim_model_lib0_DW->NextOutput_i[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_ip[1]) *
    sim_model_lib0_P->random_noise1_StdDev +
    sim_model_lib0_P->random_noise1_Mean;

  // Update for RandomNumber: '<S254>/random_noise2'
  sim_model_lib0_DW->NextOutput_j[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_e[1]) * sim_model_lib0_P->random_noise2_StdDev
    + sim_model_lib0_P->random_noise2_Mean;

  // Update for RandomNumber: '<S255>/random_noise1'
  sim_model_lib0_DW->NextOutput_h[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_p[1]) *
    sim_model_lib0_P->random_noise1_StdDev_l +
    sim_model_lib0_P->random_noise1_Mean_d;

  // Update for RandomNumber: '<S255>/random_noise2'
  sim_model_lib0_DW->NextOutput_p[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_n[1]) *
    sim_model_lib0_P->random_noise2_StdDev_k +
    sim_model_lib0_P->random_noise2_Mean_f;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] >=
      sim_model_lib0_P->env_max_ext_air_omega) {
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] =
      sim_model_lib0_P->env_max_ext_air_omega;
  } else {
    if (sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] <=
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F)) {
      sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] =
        -(sim_model_lib0_P->env_max_ext_air_omega + 2.22044605E-16F);
    }
  }

  // Update for RandomNumber: '<S254>/random_noise'
  sim_model_lib0_DW->NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed[2]) * sim_model_lib0_P->random_noise_StdDev +
    sim_model_lib0_P->random_noise_Mean;

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2'
  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[2] +=
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainval * rtb_random_noise[2];

  // Update for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S254>/Discrete-Time Integrator1'
  //   Gain: '<S254>/Gain2'
  //   RandomNumber: '<S254>/random_noise2'
  //   Sum: '<S254>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[2] +=
    (sim_model_lib0_DW->NextOutput_j[2] -
     sim_model_lib0_P->epson_accel_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[2]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainval;

  // Update for RandomNumber: '<S255>/random_noise'
  sim_model_lib0_DW->NextOutput_e[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_k[2]) *
    sim_model_lib0_P->random_noise_StdDev_c +
    sim_model_lib0_P->random_noise_Mean_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' incorporates:
  //   Gain: '<S255>/Gain4'
  //   RandomNumber: '<S255>/random_noise1'

  sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[2] += rtb_Sum *
    sim_model_lib0_DW->NextOutput_h[2] *
    sim_model_lib0_P->DiscreteTimeIntegrator2_gainv_k;

  // Update for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S255>/Discrete-Time Integrator1'
  //   Gain: '<S255>/Gain2'
  //   RandomNumber: '<S255>/random_noise2'
  //   Sum: '<S255>/Sum'

  sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[2] +=
    (sim_model_lib0_DW->NextOutput_p[2] -
     sim_model_lib0_P->epson_gyro_markov_tau *
     sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[2]) *
    sim_model_lib0_P->DiscreteTimeIntegrator1_gainv_l;

  // Update for RandomNumber: '<S6>/Random Number'
  sim_model_lib0_DW->NextOutput_l[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_i[2]) * sqrt
    (sim_model_lib0_P->env_ext_air_vel_variance[2]) +
    sim_model_lib0_P->RandomNumber_Mean;

  // Update for RandomNumber: '<S6>/Random Number1'
  sim_model_lib0_DW->NextOutput_f[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_m[2]) * sqrt
    (sim_model_lib0_P->env_ext_air_omega_variance[2]) +
    sim_model_lib0_P->RandomNumber1_Mean;

  // Update for UnitDelay: '<S194>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE[2] =
    sim_model_lib0_P->tun_default_center_of_mass[2];

  // Update for UnitDelay: '<S227>/Delay Input1' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   Constant: '<S1>/Constant9'

  sim_model_lib0_DW->DelayInput1_DSTATE_m[2] =
    sim_model_lib0_P->tun_default_center_of_mass[2];

  // Update for RandomNumber: '<S254>/random_noise1'
  sim_model_lib0_DW->NextOutput_i[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_ip[2]) *
    sim_model_lib0_P->random_noise1_StdDev +
    sim_model_lib0_P->random_noise1_Mean;

  // Update for RandomNumber: '<S254>/random_noise2'
  sim_model_lib0_DW->NextOutput_j[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_e[2]) * sim_model_lib0_P->random_noise2_StdDev
    + sim_model_lib0_P->random_noise2_Mean;

  // Update for RandomNumber: '<S255>/random_noise1'
  sim_model_lib0_DW->NextOutput_h[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_p[2]) *
    sim_model_lib0_P->random_noise1_StdDev_l +
    sim_model_lib0_P->random_noise1_Mean_d;

  // Update for RandomNumber: '<S255>/random_noise2'
  sim_model_lib0_DW->NextOutput_p[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_n[2]) *
    sim_model_lib0_P->random_noise2_StdDev_k +
    sim_model_lib0_P->random_noise2_Mean_f;
}

// Model step function for TID1
void sim_model_lib0_step1(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M) // Sample time: [0.16s, 0.0s] 
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);

  // local block i/o variables
  real32_T rtb_ImpAsg_InsertedFor_P_points[72];
  boolean_T rtb_ImpAsg_InsertedFor_points_i[36];
  real32_T rtb_Conversion_my[9];
  real32_T rtb_Subtract1[3];
  real32_T rtb_MatrixConcatenate[16];
  real32_T rtb_Product[4];
  real32_T rtb_Sum;
  real32_T rtb_Assignment[9];
  real32_T rtb_Sum_od;
  real32_T rtb_P_point_B_iss[108];
  real32_T rtb_valid_out[50];
  int8_T rtb_DataTypeConversion4[50];
  real32_T rtb_Add_b[108];
  real32_T rtb_points_out_iss[150];
  real32_T rtb_points_out_2D_cam[100];
  int32_T i;
  boolean_T tmp[36];
  int32_T i_0;
  boolean_T tmp_0;
  real32_T tmp_1[9];
  real32_T rtb_Assignment_h4[9];
  real32_T tmp_2[9];
  real32_T rtb_Product_o3[9];
  real32_T rtb_Add_c[108];
  real32_T tmp_3;
  real32_T tmp_4;
  real32_T rtb_Product_b;

  // DataTypeConversion: '<S94>/Conversion' incorporates:
  //   Constant: '<S93>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Conversion_my[i] = (real32_T)sim_model_lib0_P->Constant2_Value_n1[i];
  }

  // End of DataTypeConversion: '<S94>/Conversion'

  // Assignment: '<S93>/Assignment' incorporates:
  //   Constant: '<S88>/Constant3'

  rtb_Conversion_my[0] = sim_model_lib0_P->tun_abp_q_body2dockcam[3];
  rtb_Conversion_my[4] = sim_model_lib0_P->tun_abp_q_body2dockcam[3];
  rtb_Conversion_my[8] = sim_model_lib0_P->tun_abp_q_body2dockcam[3];

  // Sum: '<S93>/Sum2' incorporates:
  //   Constant: '<S88>/Constant3'
  //   Constant: '<S95>/Constant3'
  //   DataTypeConversion: '<S96>/Conversion'
  //   Gain: '<S95>/Gain'
  //   Gain: '<S95>/Gain1'
  //   Gain: '<S95>/Gain2'

  rtb_Product_o3[0] = (real32_T)sim_model_lib0_P->Constant3_Value_eu;
  rtb_Product_o3[1] = sim_model_lib0_P->tun_abp_q_body2dockcam[2];
  rtb_Product_o3[2] = sim_model_lib0_P->Gain_Gain_o *
    sim_model_lib0_P->tun_abp_q_body2dockcam[1];
  rtb_Product_o3[3] = sim_model_lib0_P->Gain1_Gain_mr *
    sim_model_lib0_P->tun_abp_q_body2dockcam[2];
  rtb_Product_o3[4] = (real32_T)sim_model_lib0_P->Constant3_Value_eu;
  rtb_Product_o3[5] = sim_model_lib0_P->tun_abp_q_body2dockcam[0];
  rtb_Product_o3[6] = sim_model_lib0_P->tun_abp_q_body2dockcam[1];
  rtb_Product_o3[7] = sim_model_lib0_P->Gain2_Gain_og *
    sim_model_lib0_P->tun_abp_q_body2dockcam[0];
  rtb_Product_o3[8] = (real32_T)sim_model_lib0_P->Constant3_Value_eu;

  // Concatenate: '<S93>/Matrix Concatenate' incorporates:
  //   Constant: '<S88>/Constant3'
  //   Gain: '<S93>/Gain1'
  //   Sum: '<S93>/Sum2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_MatrixConcatenate[(int32_T)(i_0 << 2)] = rtb_Conversion_my[(int32_T)(3 *
      i_0)] + rtb_Product_o3[(int32_T)(3 * i_0)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_my[(int32_T)((int32_T)(3 * i_0) + 1)] + rtb_Product_o3
      [(int32_T)((int32_T)(3 * i_0) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_my[(int32_T)((int32_T)(3 * i_0) + 2)] + rtb_Product_o3
      [(int32_T)((int32_T)(3 * i_0) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_cz *
    sim_model_lib0_P->tun_abp_q_body2dockcam[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_cz *
    sim_model_lib0_P->tun_abp_q_body2dockcam[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_cz *
    sim_model_lib0_P->tun_abp_q_body2dockcam[2];

  // End of Concatenate: '<S93>/Matrix Concatenate'

  // Switch: '<S88>/Switch' incorporates:
  //   Constant: '<S88>/Constant'

  tmp_0 = ((int32_T)sim_model_lib0_P->tun_cvs_noise_on != 0);

  // Reshape: '<S90>/Reshape1' incorporates:
  //   Constant: '<S88>/Constant3'

  rtb_MatrixConcatenate[12] = sim_model_lib0_P->tun_abp_q_body2dockcam[0];

  // Product: '<S90>/Product' incorporates:
  //   Constant: '<S88>/Constant6'
  //   Constant: '<S88>/Constant7'
  //   Switch: '<S88>/Switch'

  if (tmp_0) {
    rtb_Sum = sim_model_lib0_P->cvs_dockcam_Q_B2dockcam_error[0];
    rtb_Sum_od = sim_model_lib0_P->cvs_dockcam_Q_B2dockcam_error[1];
  } else {
    rtb_Sum = sim_model_lib0_P->Constant7_Value_n[0];
    rtb_Sum_od = sim_model_lib0_P->Constant7_Value_n[1];
  }

  // Reshape: '<S90>/Reshape1' incorporates:
  //   Constant: '<S88>/Constant3'

  rtb_MatrixConcatenate[13] = sim_model_lib0_P->tun_abp_q_body2dockcam[1];
  rtb_MatrixConcatenate[14] = sim_model_lib0_P->tun_abp_q_body2dockcam[2];

  // Product: '<S90>/Product' incorporates:
  //   Constant: '<S88>/Constant6'
  //   Constant: '<S88>/Constant7'
  //   Switch: '<S88>/Switch'

  if (tmp_0) {
    tmp_3 = sim_model_lib0_P->cvs_dockcam_Q_B2dockcam_error[2];
    tmp_4 = sim_model_lib0_P->cvs_dockcam_Q_B2dockcam_error[3];
  } else {
    tmp_3 = sim_model_lib0_P->Constant7_Value_n[2];
    tmp_4 = sim_model_lib0_P->Constant7_Value_n[3];
  }

  // Reshape: '<S90>/Reshape1' incorporates:
  //   Constant: '<S88>/Constant3'

  rtb_MatrixConcatenate[15] = sim_model_lib0_P->tun_abp_q_body2dockcam[3];

  // Product: '<S90>/Product'
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_Product_b = rtb_MatrixConcatenate[(int32_T)(i_0 + 12)] * tmp_4 +
      (rtb_MatrixConcatenate[(int32_T)(i_0 + 8)] * tmp_3 +
       (rtb_MatrixConcatenate[(int32_T)(i_0 + 4)] * rtb_Sum_od +
        rtb_MatrixConcatenate[i_0] * rtb_Sum));
    rtb_Product[i_0] = rtb_Product_b;
  }

  // Sum: '<S92>/Sum' incorporates:
  //   Constant: '<S92>/Constant1'
  //   DataTypeConversion: '<S102>/Conversion'
  //   Gain: '<S92>/Gain'
  //   Math: '<S92>/Math Function'

  rtb_Sum = rtb_Product[3] * rtb_Product[3] * sim_model_lib0_P->Gain_Gain_oe -
    (real32_T)sim_model_lib0_P->Constant1_Value_g;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S92>/Assignment' incorporates:
    //   Constant: '<S92>/Constant2'
    //   DataTypeConversion: '<S101>/Conversion'

    rtb_Conversion_my[i] = (real32_T)sim_model_lib0_P->Constant2_Value_ci[i];

    // Assignment: '<S91>/Assignment' incorporates:
    //   Constant: '<S91>/Constant2'
    //   DataTypeConversion: '<S97>/Conversion'

    rtb_Assignment[i] = (real32_T)sim_model_lib0_P->Constant2_Value_a[i];
  }

  // Assignment: '<S92>/Assignment'
  rtb_Conversion_my[0] = rtb_Sum;
  rtb_Conversion_my[4] = rtb_Sum;
  rtb_Conversion_my[8] = rtb_Sum;

  // Gain: '<S92>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_mm * rtb_Product[3];

  // Sum: '<S91>/Sum' incorporates:
  //   Constant: '<S91>/Constant1'
  //   DataTypeConversion: '<S98>/Conversion'
  //   Gain: '<S91>/Gain'
  //   Math: '<S91>/Math Function'

  rtb_Sum_od = sim_model_lib0_B->RateTransition2.Q_ISS2B[3] *
    sim_model_lib0_B->RateTransition2.Q_ISS2B[3] * sim_model_lib0_P->Gain_Gain_a
    - (real32_T)sim_model_lib0_P->Constant1_Value_m;

  // Assignment: '<S91>/Assignment'
  rtb_Assignment[0] = rtb_Sum_od;
  rtb_Assignment[4] = rtb_Sum_od;
  rtb_Assignment[8] = rtb_Sum_od;

  // Gain: '<S91>/Gain1'
  rtb_Sum_od = sim_model_lib0_P->Gain1_Gain_al2 *
    sim_model_lib0_B->RateTransition2.Q_ISS2B[3];

  // Sum: '<S88>/Add' incorporates:
  //   Constant: '<S85>/Constant3'
  //   Constant: '<S88>/Constant4'
  //   Gain: '<S88>/Gain'

  for (i = 0; i < 108; i++) {
    rtb_Add_b[i] = (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_AR_map_error[i] + sim_model_lib0_P->cvs_AR_map_iss[i];
  }

  // End of Sum: '<S88>/Add'
  for (i_0 = 0; i_0 < 36; i_0++) {
    // Sum: '<S88>/Subtract' incorporates:
    //   Selector: '<S88>/Select_X'

    rtb_P_point_B_iss[i_0] = rtb_Add_b[i_0] -
      sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[0];

    // Sum: '<S88>/Subtract2' incorporates:
    //   Selector: '<S88>/Select_Y'

    rtb_P_point_B_iss[(int32_T)(36 + i_0)] = rtb_Add_b[(int32_T)(36 + i_0)] -
      sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[1];

    // Sum: '<S88>/Subtract3' incorporates:
    //   Selector: '<S88>/Select_Z'

    rtb_P_point_B_iss[(int32_T)(72 + i_0)] = rtb_Add_b[(int32_T)(72 + i_0)] -
      sim_model_lib0_B->RateTransition2.P_B_ISS_ISS[2];
  }

  // Product: '<S91>/Product' incorporates:
  //   Constant: '<S99>/Constant3'
  //   DataTypeConversion: '<S100>/Conversion'
  //   Gain: '<S99>/Gain'
  //   Gain: '<S99>/Gain1'
  //   Gain: '<S99>/Gain2'

  tmp_1[0] = (real32_T)sim_model_lib0_P->Constant3_Value_az;
  tmp_1[1] = sim_model_lib0_B->RateTransition2.Q_ISS2B[2];
  tmp_1[2] = sim_model_lib0_P->Gain_Gain_od *
    sim_model_lib0_B->RateTransition2.Q_ISS2B[1];
  tmp_1[3] = sim_model_lib0_P->Gain1_Gain_f *
    sim_model_lib0_B->RateTransition2.Q_ISS2B[2];
  tmp_1[4] = (real32_T)sim_model_lib0_P->Constant3_Value_az;
  tmp_1[5] = sim_model_lib0_B->RateTransition2.Q_ISS2B[0];
  tmp_1[6] = sim_model_lib0_B->RateTransition2.Q_ISS2B[1];
  tmp_1[7] = sim_model_lib0_P->Gain2_Gain_hy *
    sim_model_lib0_B->RateTransition2.Q_ISS2B[0];
  tmp_1[8] = (real32_T)sim_model_lib0_P->Constant3_Value_az;

  // Product: '<S91>/Product1' incorporates:
  //   Gain: '<S91>/Gain2'
  //   Math: '<S91>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_o3[i_0] = sim_model_lib0_B->RateTransition2.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2.Q_ISS2B[0];
    rtb_Product_o3[(int32_T)(i_0 + 3)] =
      sim_model_lib0_B->RateTransition2.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2.Q_ISS2B[1];
    rtb_Product_o3[(int32_T)(i_0 + 6)] =
      sim_model_lib0_B->RateTransition2.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2.Q_ISS2B[2];
  }

  // End of Product: '<S91>/Product1'

  // Sum: '<S91>/Sum1' incorporates:
  //   Gain: '<S91>/Gain2'
  //   Product: '<S88>/Product'
  //   Product: '<S91>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_h4[(int32_T)(3 * i_0)] = (rtb_Assignment[(int32_T)(3 * i_0)]
      - tmp_1[(int32_T)(3 * i_0)] * rtb_Sum_od) + rtb_Product_o3[(int32_T)(3 *
      i_0)] * sim_model_lib0_P->Gain2_Gain_lw;
    rtb_Assignment_h4[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum_od) + rtb_Product_o3[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_lw;
    rtb_Assignment_h4[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum_od) + rtb_Product_o3[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_lw;
  }

  // End of Sum: '<S91>/Sum1'

  // Product: '<S92>/Product' incorporates:
  //   Constant: '<S103>/Constant3'
  //   DataTypeConversion: '<S104>/Conversion'
  //   Gain: '<S103>/Gain'
  //   Gain: '<S103>/Gain1'
  //   Gain: '<S103>/Gain2'

  tmp_2[0] = (real32_T)sim_model_lib0_P->Constant3_Value_p;
  tmp_2[1] = rtb_Product[2];
  tmp_2[2] = sim_model_lib0_P->Gain_Gain_ee4 * rtb_Product[1];
  tmp_2[3] = sim_model_lib0_P->Gain1_Gain_g * rtb_Product[2];
  tmp_2[4] = (real32_T)sim_model_lib0_P->Constant3_Value_p;
  tmp_2[5] = rtb_Product[0];
  tmp_2[6] = rtb_Product[1];
  tmp_2[7] = sim_model_lib0_P->Gain2_Gain_l0 * rtb_Product[0];
  tmp_2[8] = (real32_T)sim_model_lib0_P->Constant3_Value_p;
  for (i = 0; i < 3; i++) {
    // Product: '<S88>/Product' incorporates:
    //   Math: '<S88>/Math Function'

    for (i_0 = 0; i_0 < 36; i_0++) {
      rtb_Add_b[(int32_T)(i + (int32_T)(3 * i_0))] = 0.0F;
      rtb_Add_b[(int32_T)(i + (int32_T)(3 * i_0))] += rtb_Assignment_h4[i] *
        rtb_P_point_B_iss[i_0];
      rtb_Add_b[(int32_T)(i + (int32_T)(3 * i_0))] += rtb_Assignment_h4[(int32_T)
        (i + 3)] * rtb_P_point_B_iss[(int32_T)(i_0 + 36)];
      rtb_Add_b[(int32_T)(i + (int32_T)(3 * i_0))] += rtb_Assignment_h4[(int32_T)
        (i + 6)] * rtb_P_point_B_iss[(int32_T)(i_0 + 72)];
    }

    // Sum: '<S88>/Subtract1' incorporates:
    //   Constant: '<S88>/Constant1'
    //   Constant: '<S88>/Constant2'
    //   Gain: '<S88>/Gain1'

    rtb_Subtract1[i] = sim_model_lib0_P->tun_abp_p_dockcam_body_body_sim[i] -
      (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_dockcam_P_B_B_error[i];

    // Product: '<S92>/Product1' incorporates:
    //   Gain: '<S92>/Gain2'
    //   Math: '<S92>/Math Function1'

    rtb_Product_o3[i] = rtb_Product[i] * rtb_Product[0];
    rtb_Product_o3[(int32_T)(i + 3)] = rtb_Product[i] * rtb_Product[1];
    rtb_Product_o3[(int32_T)(i + 6)] = rtb_Product[i] * rtb_Product[2];
  }

  // Sum: '<S92>/Sum1' incorporates:
  //   Gain: '<S92>/Gain2'
  //   Product: '<S88>/Product1'
  //   Product: '<S92>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_h4[(int32_T)(3 * i_0)] = (rtb_Conversion_my[(int32_T)(3 * i_0)]
      - tmp_2[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_o3[(int32_T)(3 * i_0)]
      * sim_model_lib0_P->Gain2_Gain_fv;
    rtb_Assignment_h4[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Conversion_my
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_2[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum) + rtb_Product_o3[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_fv;
    rtb_Assignment_h4[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Conversion_my
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_2[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum) + rtb_Product_o3[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_fv;
  }

  // End of Sum: '<S92>/Sum1'
  for (i_0 = 0; i_0 < 36; i_0++) {
    // Concatenate: '<S88>/Matrix Concatenate1' incorporates:
    //   Product: '<S88>/Product1'
    //   Selector: '<S88>/Select_X1'
    //   Selector: '<S88>/Select_Y1'
    //   Selector: '<S88>/Select_Z1'
    //   Sum: '<S88>/Subtract4'
    //   Sum: '<S88>/Subtract5'
    //   Sum: '<S88>/Subtract6'

    rtb_Add_c[(int32_T)(3 * i_0)] = rtb_Add_b[(int32_T)(3 * i_0)] -
      rtb_Subtract1[0];
    rtb_Add_c[(int32_T)(1 + (int32_T)(3 * i_0))] = rtb_Add_b[(int32_T)((int32_T)
      (3 * i_0) + 1)] - rtb_Subtract1[1];
    rtb_Add_c[(int32_T)(2 + (int32_T)(3 * i_0))] = rtb_Add_b[(int32_T)((int32_T)
      (3 * i_0) + 2)] - rtb_Subtract1[2];

    // Math: '<S88>/Math Function1' incorporates:
    //   Product: '<S88>/Product1'

    for (i = 0; i < 3; i++) {
      rtb_P_point_B_iss[(int32_T)(i_0 + (int32_T)(36 * i))] = 0.0F;
      rtb_P_point_B_iss[(int32_T)(i_0 + (int32_T)(36 * i))] += rtb_Add_c
        [(int32_T)(3 * i_0)] * rtb_Assignment_h4[i];
      rtb_P_point_B_iss[(int32_T)(i_0 + (int32_T)(36 * i))] += rtb_Add_c
        [(int32_T)((int32_T)(3 * i_0) + 1)] * rtb_Assignment_h4[(int32_T)(i + 3)];
      rtb_P_point_B_iss[(int32_T)(i_0 + (int32_T)(36 * i))] += rtb_Add_c
        [(int32_T)((int32_T)(3 * i_0) + 2)] * rtb_Assignment_h4[(int32_T)(i + 6)];
    }

    // End of Math: '<S88>/Math Function1'
  }

  // Outputs for Iterator SubSystem: '<S85>/pinhole_projection_model'
  si_pinhole_projection_model(36, rtb_P_point_B_iss,
    rtb_ImpAsg_InsertedFor_P_points, rtb_ImpAsg_InsertedFor_points_i,
    sim_model_lib0_DW->pinhole_projection_model,
    (P_pinhole_projection_model_si_T *)
    &sim_model_lib0_P->pinhole_projection_model,
    sim_model_lib0_P->tun_cvs_noise_on,
    sim_model_lib0_P->tun_cvs_dockcam_focal_length_Y,
    sim_model_lib0_P->tun_cvs_dockcam_focal_length_X,
    sim_model_lib0_P->cvs_AR_pixel_noise,
    sim_model_lib0_P->tun_cvs_dockcam_num_pixels_X,
    sim_model_lib0_P->tun_cvs_dockcam_num_pixels_Y,
    sim_model_lib0_P->cvs_dockcam_min_dist,
    sim_model_lib0_P->cvs_dockcam_max_dist,
    sim_model_lib0_P->cvs_dockcam_pointing);

  // End of Outputs for SubSystem: '<S85>/pinhole_projection_model'

  // MATLAB Function: '<S79>/generate_output' incorporates:
  //   Constant: '<S85>/Constant3'

  // MATLAB Function 'image_processing/generate_output': '<S86>:1'
  //  Inputs:
  //    Process type:                                                                [Parameter] 
  //        1=Landmarks, 2=AR Tags, 3=Optical Flow, 4=Handrail
  //    num_pts_out         = Required number of points to be reported               [Parameter] 
  //    P_point_iss_iss     = ALL [x,y,z] position of points in the ISS frame        **Landmark only** 
  //    P_point_cam_cam     = ALL [x,y,z] position of points in the CAM frame        **Handrail only** 
  //    P_points_2D_cam     = ALL [u,v] pixel location of points in the camera       **Landmark and OF only** 
  //    valid_flag          = indicates which points are valid
  //    num_hist_kept       = Required number of histories kept for Optical Flow     [Parameter] **OF only** 
  //    flow_ids            = ID tag for each optical flow point                     [Parameter] **OF only** 
  //
  //  Outputs:
  //    points_out_iss      = output [x,y,z] position of points in the ISS frame     **This output unused by optical flow** 
  //    points_out_2D_cam   = output [u,v] pixel location of points in the camera    **This output unused by handrail** 
  //    points_out_cam      = output [x,y,z] position of points in the CAM frame     **This output unused by OF and landmarks** 
  //    valid_out           = indicates which values of the output are valid!        **This output unused by landmarks** 
  //    ids_out             = optical flow IDs
  //    registration_id     = visual odometery registration pulse ID                 **ONLY used by OF** 
  // AR tags
  // '<S86>:1:34'
  for (i_0 = 0; i_0 < 36; i_0++) {
    tmp[i_0] = rtb_ImpAsg_InsertedFor_points_i[i_0];
  }

  sim_model__format_AR_tag_output(sim_model_lib0_P->cvs_AR_map_iss,
    rtb_ImpAsg_InsertedFor_P_points, tmp, rtb_points_out_iss,
    rtb_points_out_2D_cam, rtb_valid_out);

  // End of MATLAB Function: '<S79>/generate_output'

  // DataTypeConversion: '<S84>/Data Type Conversion4' incorporates:
  //   Constant: '<S84>/Constant3'
  //   Logic: '<S84>/Logical Operator'
  //   Selector: '<S84>/select_current_command'
  //   UnitDelay: '<S84>/Unit Delay'

  // '<S86>:1:34'
  // '<S86>:1:35'
  // '<S86>:1:36'
  // '<S86>:1:37'
  // end of function
  for (i = 0; i < 50; i++) {
    rtb_DataTypeConversion4[i] = (int8_T)((rtb_valid_out[i] != 0.0F) &&
      sim_model_lib0_P->cvs_AR_valid_mask[(int32_T)((int32_T)
      sim_model_lib0_DW->UnitDelay_DSTATE_hj - 1)]);
  }

  // End of DataTypeConversion: '<S84>/Data Type Conversion4'

  // Sum: '<S84>/Sum1' incorporates:
  //   Constant: '<S84>/Constant1'
  //   Constant: '<S87>/Constant'
  //   Logic: '<S84>/Logical Operator1'
  //   Logic: '<S84>/Logical Operator2'
  //   Logic: '<S84>/Logical Operator3'
  //   RelationalOperator: '<S84>/Relational Operator'
  //   RelationalOperator: '<S84>/Relational Operator1'
  //   RelationalOperator: '<S84>/Relational Operator2'
  //   RelationalOperator: '<S87>/Compare'
  //   Selector: '<S84>/Select_col_1'
  //   Selector: '<S84>/Select_col_2'

  i_0 = (int32_T)(uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->cvs_AR_valid_times[0] !=
      sim_model_lib0_P->Constant_Value_e) &&
     ((sim_model_lib0_P->cvs_AR_valid_times[0] <
       sim_model_lib0_B->RateTransition4.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_AR_valid_times[0] ==
        sim_model_lib0_B->RateTransition4.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4.timestamp_nsec >=
        sim_model_lib0_P->cvs_AR_valid_times[3])))) + (uint32_T)
    ((sim_model_lib0_P->cvs_AR_valid_times[1] !=
      sim_model_lib0_P->Constant_Value_e) &&
     ((sim_model_lib0_P->cvs_AR_valid_times[1] <
       sim_model_lib0_B->RateTransition4.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_AR_valid_times[1] ==
        sim_model_lib0_B->RateTransition4.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4.timestamp_nsec >=
        sim_model_lib0_P->cvs_AR_valid_times[4]))))) + (uint32_T)
    ((sim_model_lib0_P->cvs_AR_valid_times[2] !=
      sim_model_lib0_P->Constant_Value_e) &&
     ((sim_model_lib0_P->cvs_AR_valid_times[2] <
       sim_model_lib0_B->RateTransition4.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_AR_valid_times[2] ==
        sim_model_lib0_B->RateTransition4.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4.timestamp_nsec >=
        sim_model_lib0_P->cvs_AR_valid_times[5])))));

  // Update for RateTransition: '<S79>/Rate Transition3'
  sim_model_lib0_DW->RateTransition3_Buffer0_n =
    sim_model_lib0_B->RateTransition4.timestamp_sec;

  // Update for RateTransition: '<S79>/Rate Transition1'
  sim_model_lib0_DW->RateTransition1_Buffer0_i =
    sim_model_lib0_B->RateTransition4.timestamp_nsec;

  // Update for RateTransition: '<S79>/Rate Transition5'
  memcpy(&sim_model_lib0_DW->RateTransition5_Buffer0_f[0], &rtb_points_out_iss[0],
         (uint32_T)(150U * sizeof(real32_T)));

  // Update for RateTransition: '<S79>/Rate Transition6'
  memcpy(&sim_model_lib0_DW->RateTransition6_Buffer0_e[0],
         &rtb_points_out_2D_cam[0], (uint32_T)(100U * sizeof(real32_T)));

  // Update for RateTransition: '<S79>/Rate Transition7'
  for (i = 0; i < 50; i++) {
    sim_model_lib0_DW->RateTransition7_Buffer0_j[i] = (uint8_T)
      rtb_DataTypeConversion4[i];
  }

  // End of Update for RateTransition: '<S79>/Rate Transition7'

  // Update for RandomNumber: '<S85>/pixel_noise'
  rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_mb);

  // Saturate: '<S84>/Saturation' incorporates:
  //   Sum: '<S84>/Sum1'

  if ((uint8_T)i_0 > sim_model_lib0_P->Saturation_UpperSat_i) {
    // Update for UnitDelay: '<S84>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_hj =
      sim_model_lib0_P->Saturation_UpperSat_i;
  } else if ((uint8_T)i_0 < sim_model_lib0_P->Saturation_LowerSat_b) {
    // Update for UnitDelay: '<S84>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_hj =
      sim_model_lib0_P->Saturation_LowerSat_b;
  } else {
    // Update for UnitDelay: '<S84>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_hj = (uint8_T)i_0;
  }

  // End of Saturate: '<S84>/Saturation'
}

// Model step function for TID2
void sim_model_lib0_step2(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M) // Sample time: [0.20800000000000002s, 0.0s] 
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);
  real32_T rtb_Conversion_i[9];
  real32_T rtb_Subtract1[3];
  real32_T rtb_MatrixConcatenate[16];
  real32_T rtb_Product[4];
  real32_T rtb_Sum;
  real32_T rtb_Assignment[9];
  real32_T rtb_Sum_as;
  int8_T rtb_DataTypeConversion4[50];
  real32_T valid_out_i[50];
  real32_T ImpAsg_InsertedFor_P_points_2_g[1008];
  int32_T i;
  int32_T i_0;
  boolean_T tmp;
  real32_T tmp_0[9];
  real32_T rtb_Assignment_in[9];
  real32_T tmp_1[9];
  real32_T rtb_Product_h[9];
  real32_T tmp_2;
  real32_T tmp_3;
  real32_T rtb_Product_b;

  // DataTypeConversion: '<S122>/Conversion' incorporates:
  //   Constant: '<S121>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Conversion_i[i] = (real32_T)sim_model_lib0_P->Constant2_Value_nv[i];
  }

  // End of DataTypeConversion: '<S122>/Conversion'

  // Assignment: '<S121>/Assignment' incorporates:
  //   Constant: '<S116>/Constant3'

  rtb_Conversion_i[0] = sim_model_lib0_P->tun_abp_q_body2perchcam[3];
  rtb_Conversion_i[4] = sim_model_lib0_P->tun_abp_q_body2perchcam[3];
  rtb_Conversion_i[8] = sim_model_lib0_P->tun_abp_q_body2perchcam[3];

  // Sum: '<S121>/Sum2' incorporates:
  //   Constant: '<S116>/Constant3'
  //   Constant: '<S123>/Constant3'
  //   DataTypeConversion: '<S124>/Conversion'
  //   Gain: '<S123>/Gain'
  //   Gain: '<S123>/Gain1'
  //   Gain: '<S123>/Gain2'

  rtb_Product_h[0] = (real32_T)sim_model_lib0_P->Constant3_Value_h;
  rtb_Product_h[1] = sim_model_lib0_P->tun_abp_q_body2perchcam[2];
  rtb_Product_h[2] = sim_model_lib0_P->Gain_Gain_op *
    sim_model_lib0_P->tun_abp_q_body2perchcam[1];
  rtb_Product_h[3] = sim_model_lib0_P->Gain1_Gain_da *
    sim_model_lib0_P->tun_abp_q_body2perchcam[2];
  rtb_Product_h[4] = (real32_T)sim_model_lib0_P->Constant3_Value_h;
  rtb_Product_h[5] = sim_model_lib0_P->tun_abp_q_body2perchcam[0];
  rtb_Product_h[6] = sim_model_lib0_P->tun_abp_q_body2perchcam[1];
  rtb_Product_h[7] = sim_model_lib0_P->Gain2_Gain_fs *
    sim_model_lib0_P->tun_abp_q_body2perchcam[0];
  rtb_Product_h[8] = (real32_T)sim_model_lib0_P->Constant3_Value_h;

  // Concatenate: '<S121>/Matrix Concatenate' incorporates:
  //   Constant: '<S116>/Constant3'
  //   Gain: '<S121>/Gain1'
  //   Sum: '<S121>/Sum2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_MatrixConcatenate[(int32_T)(i_0 << 2)] = rtb_Conversion_i[(int32_T)(3 *
      i_0)] + rtb_Product_h[(int32_T)(3 * i_0)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_i[(int32_T)((int32_T)(3 * i_0) + 1)] + rtb_Product_h
      [(int32_T)((int32_T)(3 * i_0) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_i[(int32_T)((int32_T)(3 * i_0) + 2)] + rtb_Product_h
      [(int32_T)((int32_T)(3 * i_0) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_jq *
    sim_model_lib0_P->tun_abp_q_body2perchcam[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_jq *
    sim_model_lib0_P->tun_abp_q_body2perchcam[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_jq *
    sim_model_lib0_P->tun_abp_q_body2perchcam[2];

  // End of Concatenate: '<S121>/Matrix Concatenate'

  // Switch: '<S116>/Switch' incorporates:
  //   Constant: '<S116>/Constant'

  tmp = ((int32_T)sim_model_lib0_P->tun_cvs_noise_on != 0);

  // Reshape: '<S118>/Reshape1' incorporates:
  //   Constant: '<S116>/Constant3'

  rtb_MatrixConcatenate[12] = sim_model_lib0_P->tun_abp_q_body2perchcam[0];

  // Product: '<S118>/Product' incorporates:
  //   Constant: '<S116>/Constant6'
  //   Constant: '<S116>/Constant7'
  //   Switch: '<S116>/Switch'

  if (tmp) {
    rtb_Sum = sim_model_lib0_P->cvs_perchcam_Q_B2perchcam_error[0];
    rtb_Sum_as = sim_model_lib0_P->cvs_perchcam_Q_B2perchcam_error[1];
  } else {
    rtb_Sum = sim_model_lib0_P->Constant7_Value_h[0];
    rtb_Sum_as = sim_model_lib0_P->Constant7_Value_h[1];
  }

  // Reshape: '<S118>/Reshape1' incorporates:
  //   Constant: '<S116>/Constant3'

  rtb_MatrixConcatenate[13] = sim_model_lib0_P->tun_abp_q_body2perchcam[1];
  rtb_MatrixConcatenate[14] = sim_model_lib0_P->tun_abp_q_body2perchcam[2];

  // Product: '<S118>/Product' incorporates:
  //   Constant: '<S116>/Constant6'
  //   Constant: '<S116>/Constant7'
  //   Switch: '<S116>/Switch'

  if (tmp) {
    tmp_2 = sim_model_lib0_P->cvs_perchcam_Q_B2perchcam_error[2];
    tmp_3 = sim_model_lib0_P->cvs_perchcam_Q_B2perchcam_error[3];
  } else {
    tmp_2 = sim_model_lib0_P->Constant7_Value_h[2];
    tmp_3 = sim_model_lib0_P->Constant7_Value_h[3];
  }

  // Reshape: '<S118>/Reshape1' incorporates:
  //   Constant: '<S116>/Constant3'

  rtb_MatrixConcatenate[15] = sim_model_lib0_P->tun_abp_q_body2perchcam[3];

  // Product: '<S118>/Product'
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_Product_b = rtb_MatrixConcatenate[(int32_T)(i_0 + 12)] * tmp_3 +
      (rtb_MatrixConcatenate[(int32_T)(i_0 + 8)] * tmp_2 +
       (rtb_MatrixConcatenate[(int32_T)(i_0 + 4)] * rtb_Sum_as +
        rtb_MatrixConcatenate[i_0] * rtb_Sum));
    rtb_Product[i_0] = rtb_Product_b;
  }

  // Sum: '<S120>/Sum' incorporates:
  //   Constant: '<S120>/Constant1'
  //   DataTypeConversion: '<S130>/Conversion'
  //   Gain: '<S120>/Gain'
  //   Math: '<S120>/Math Function'

  rtb_Sum = rtb_Product[3] * rtb_Product[3] * sim_model_lib0_P->Gain_Gain_pb -
    (real32_T)sim_model_lib0_P->Constant1_Value_oc;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S120>/Assignment' incorporates:
    //   Constant: '<S120>/Constant2'
    //   DataTypeConversion: '<S129>/Conversion'

    rtb_Conversion_i[i] = (real32_T)sim_model_lib0_P->Constant2_Value_ab[i];

    // Assignment: '<S119>/Assignment' incorporates:
    //   Constant: '<S119>/Constant2'
    //   DataTypeConversion: '<S125>/Conversion'

    rtb_Assignment[i] = (real32_T)sim_model_lib0_P->Constant2_Value_o[i];
  }

  // Assignment: '<S120>/Assignment'
  rtb_Conversion_i[0] = rtb_Sum;
  rtb_Conversion_i[4] = rtb_Sum;
  rtb_Conversion_i[8] = rtb_Sum;

  // Gain: '<S120>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_i * rtb_Product[3];

  // Sum: '<S119>/Sum' incorporates:
  //   Constant: '<S119>/Constant1'
  //   DataTypeConversion: '<S126>/Conversion'
  //   Gain: '<S119>/Gain'
  //   Math: '<S119>/Math Function'

  rtb_Sum_as = sim_model_lib0_B->RateTransition2_o.Q_ISS2B[3] *
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[3] *
    sim_model_lib0_P->Gain_Gain_cl - (real32_T)
    sim_model_lib0_P->Constant1_Value_e;

  // Assignment: '<S119>/Assignment'
  rtb_Assignment[0] = rtb_Sum_as;
  rtb_Assignment[4] = rtb_Sum_as;
  rtb_Assignment[8] = rtb_Sum_as;

  // Gain: '<S119>/Gain1'
  rtb_Sum_as = sim_model_lib0_P->Gain1_Gain_ap *
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[3];

  // Sum: '<S116>/Add' incorporates:
  //   Constant: '<S113>/Constant3'
  //   Constant: '<S116>/Constant4'
  //   Gain: '<S116>/Gain'

  for (i = 0; i < 1512; i++) {
    sim_model_lib0_B->Add_l[i] = (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_handrail_map_error[i] +
      sim_model_lib0_P->cvs_handrail_map_iss[i];
  }

  // End of Sum: '<S116>/Add'
  for (i_0 = 0; i_0 < 504; i_0++) {
    // Sum: '<S116>/Subtract' incorporates:
    //   Selector: '<S116>/Select_X'

    sim_model_lib0_B->P_point_B_iss_p[i_0] = sim_model_lib0_B->Add_l[i_0] -
      sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[0];

    // Sum: '<S116>/Subtract2' incorporates:
    //   Selector: '<S116>/Select_Y'

    sim_model_lib0_B->P_point_B_iss_p[(int32_T)(504 + i_0)] =
      sim_model_lib0_B->Add_l[(int32_T)(504 + i_0)] -
      sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[1];

    // Sum: '<S116>/Subtract3' incorporates:
    //   Selector: '<S116>/Select_Z'

    sim_model_lib0_B->P_point_B_iss_p[(int32_T)(1008 + i_0)] =
      sim_model_lib0_B->Add_l[(int32_T)(1008 + i_0)] -
      sim_model_lib0_B->RateTransition2_o.P_B_ISS_ISS[2];
  }

  // Product: '<S119>/Product' incorporates:
  //   Constant: '<S127>/Constant3'
  //   DataTypeConversion: '<S128>/Conversion'
  //   Gain: '<S127>/Gain'
  //   Gain: '<S127>/Gain1'
  //   Gain: '<S127>/Gain2'

  tmp_0[0] = (real32_T)sim_model_lib0_P->Constant3_Value_j0;
  tmp_0[1] = sim_model_lib0_B->RateTransition2_o.Q_ISS2B[2];
  tmp_0[2] = sim_model_lib0_P->Gain_Gain_mu *
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[1];
  tmp_0[3] = sim_model_lib0_P->Gain1_Gain_d3 *
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[2];
  tmp_0[4] = (real32_T)sim_model_lib0_P->Constant3_Value_j0;
  tmp_0[5] = sim_model_lib0_B->RateTransition2_o.Q_ISS2B[0];
  tmp_0[6] = sim_model_lib0_B->RateTransition2_o.Q_ISS2B[1];
  tmp_0[7] = sim_model_lib0_P->Gain2_Gain_e *
    sim_model_lib0_B->RateTransition2_o.Q_ISS2B[0];
  tmp_0[8] = (real32_T)sim_model_lib0_P->Constant3_Value_j0;

  // Product: '<S119>/Product1' incorporates:
  //   Gain: '<S119>/Gain2'
  //   Math: '<S119>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_h[i_0] = sim_model_lib0_B->RateTransition2_o.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_o.Q_ISS2B[0];
    rtb_Product_h[(int32_T)(i_0 + 3)] =
      sim_model_lib0_B->RateTransition2_o.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_o.Q_ISS2B[1];
    rtb_Product_h[(int32_T)(i_0 + 6)] =
      sim_model_lib0_B->RateTransition2_o.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_o.Q_ISS2B[2];
  }

  // End of Product: '<S119>/Product1'

  // Sum: '<S119>/Sum1' incorporates:
  //   Gain: '<S119>/Gain2'
  //   Product: '<S116>/Product'
  //   Product: '<S119>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_in[(int32_T)(3 * i_0)] = (rtb_Assignment[(int32_T)(3 * i_0)]
      - tmp_0[(int32_T)(3 * i_0)] * rtb_Sum_as) + rtb_Product_h[(int32_T)(3 *
      i_0)] * sim_model_lib0_P->Gain2_Gain_hf;
    rtb_Assignment_in[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_0[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum_as) + rtb_Product_h[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_hf;
    rtb_Assignment_in[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_0[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum_as) + rtb_Product_h[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_hf;
  }

  // End of Sum: '<S119>/Sum1'

  // Product: '<S120>/Product' incorporates:
  //   Constant: '<S131>/Constant3'
  //   DataTypeConversion: '<S132>/Conversion'
  //   Gain: '<S131>/Gain'
  //   Gain: '<S131>/Gain1'
  //   Gain: '<S131>/Gain2'

  tmp_1[0] = (real32_T)sim_model_lib0_P->Constant3_Value_f;
  tmp_1[1] = rtb_Product[2];
  tmp_1[2] = sim_model_lib0_P->Gain_Gain_hw * rtb_Product[1];
  tmp_1[3] = sim_model_lib0_P->Gain1_Gain_dx * rtb_Product[2];
  tmp_1[4] = (real32_T)sim_model_lib0_P->Constant3_Value_f;
  tmp_1[5] = rtb_Product[0];
  tmp_1[6] = rtb_Product[1];
  tmp_1[7] = sim_model_lib0_P->Gain2_Gain_j * rtb_Product[0];
  tmp_1[8] = (real32_T)sim_model_lib0_P->Constant3_Value_f;
  for (i = 0; i < 3; i++) {
    // Product: '<S116>/Product' incorporates:
    //   Math: '<S116>/Math Function'

    for (i_0 = 0; i_0 < 504; i_0++) {
      sim_model_lib0_B->Add_l[(int32_T)(i + (int32_T)(3 * i_0))] = 0.0F;
      sim_model_lib0_B->Add_l[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_in[i] * sim_model_lib0_B->P_point_B_iss_p[i_0];
      sim_model_lib0_B->Add_l[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_in[(int32_T)(i + 3)] * sim_model_lib0_B->P_point_B_iss_p
        [(int32_T)(i_0 + 504)];
      sim_model_lib0_B->Add_l[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_in[(int32_T)(i + 6)] * sim_model_lib0_B->P_point_B_iss_p
        [(int32_T)(i_0 + 1008)];
    }

    // Sum: '<S116>/Subtract1' incorporates:
    //   Constant: '<S116>/Constant1'
    //   Constant: '<S116>/Constant2'
    //   Gain: '<S116>/Gain1'

    rtb_Subtract1[i] = sim_model_lib0_P->tun_abp_p_perchcam_body_body_sim[i] -
      (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_perchcam_P_B_B_error[i];

    // Product: '<S120>/Product1' incorporates:
    //   Gain: '<S120>/Gain2'
    //   Math: '<S120>/Math Function1'

    rtb_Product_h[i] = rtb_Product[i] * rtb_Product[0];
    rtb_Product_h[(int32_T)(i + 3)] = rtb_Product[i] * rtb_Product[1];
    rtb_Product_h[(int32_T)(i + 6)] = rtb_Product[i] * rtb_Product[2];
  }

  // Sum: '<S120>/Sum1' incorporates:
  //   Gain: '<S120>/Gain2'
  //   Product: '<S116>/Product1'
  //   Product: '<S120>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_in[(int32_T)(3 * i_0)] = (rtb_Conversion_i[(int32_T)(3 * i_0)]
      - tmp_1[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_h[(int32_T)(3 * i_0)]
      * sim_model_lib0_P->Gain2_Gain_dy;
    rtb_Assignment_in[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Conversion_i
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum) + rtb_Product_h[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_dy;
    rtb_Assignment_in[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Conversion_i
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum) + rtb_Product_h[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_dy;
  }

  // End of Sum: '<S120>/Sum1'
  for (i_0 = 0; i_0 < 504; i_0++) {
    // Concatenate: '<S116>/Matrix Concatenate1' incorporates:
    //   Product: '<S116>/Product1'
    //   Selector: '<S116>/Select_X1'
    //   Selector: '<S116>/Select_Y1'
    //   Selector: '<S116>/Select_Z1'
    //   Sum: '<S116>/Subtract4'
    //   Sum: '<S116>/Subtract5'
    //   Sum: '<S116>/Subtract6'

    sim_model_lib0_B->rtb_Add_l_g[(int32_T)(3 * i_0)] = sim_model_lib0_B->Add_l
      [(int32_T)(3 * i_0)] - rtb_Subtract1[0];
    sim_model_lib0_B->rtb_Add_l_g[(int32_T)(1 + (int32_T)(3 * i_0))] =
      sim_model_lib0_B->Add_l[(int32_T)((int32_T)(3 * i_0) + 1)] -
      rtb_Subtract1[1];
    sim_model_lib0_B->rtb_Add_l_g[(int32_T)(2 + (int32_T)(3 * i_0))] =
      sim_model_lib0_B->Add_l[(int32_T)((int32_T)(3 * i_0) + 2)] -
      rtb_Subtract1[2];

    // Math: '<S116>/Math Function1' incorporates:
    //   Product: '<S116>/Product1'

    for (i = 0; i < 3; i++) {
      sim_model_lib0_B->P_point_B_iss_p[(int32_T)(i_0 + (int32_T)(504 * i))] =
        0.0F;
      sim_model_lib0_B->P_point_B_iss_p[(int32_T)(i_0 + (int32_T)(504 * i))] +=
        sim_model_lib0_B->rtb_Add_l_g[(int32_T)(3 * i_0)] * rtb_Assignment_in[i];
      sim_model_lib0_B->P_point_B_iss_p[(int32_T)(i_0 + (int32_T)(504 * i))] +=
        sim_model_lib0_B->rtb_Add_l_g[(int32_T)((int32_T)(3 * i_0) + 1)] *
        rtb_Assignment_in[(int32_T)(i + 3)];
      sim_model_lib0_B->P_point_B_iss_p[(int32_T)(i_0 + (int32_T)(504 * i))] +=
        sim_model_lib0_B->rtb_Add_l_g[(int32_T)((int32_T)(3 * i_0) + 2)] *
        rtb_Assignment_in[(int32_T)(i + 6)];
    }

    // End of Math: '<S116>/Math Function1'
  }

  // Gain: '<S113>/Gain' incorporates:
  //   DataTypeConversion: '<S113>/Data Type Conversion2'
  //   RandomNumber: '<S113>/pixel_noise'

  rtb_Sum = (real32_T)sim_model_lib0_P->tun_cvs_noise_on * (real32_T)
    sim_model_lib0_DW->NextOutput_o;

  // Outputs for Iterator SubSystem: '<S113>/pinhole_projection_model'
  si_pinhole_projection_model(504, sim_model_lib0_B->P_point_B_iss_p,
    ImpAsg_InsertedFor_P_points_2_g,
    sim_model_lib0_B->ImpAsg_InsertedFor_points_in__n,
    sim_model_lib0_DW->pinhole_projection_model_i,
    (P_pinhole_projection_model_si_T *)
    &sim_model_lib0_P->pinhole_projection_model_i,
    sim_model_lib0_P->tun_cvs_noise_on,
    sim_model_lib0_P->tun_cvs_perchcam_focal_length_Y,
    sim_model_lib0_P->tun_cvs_perchcam_focal_length_X,
    sim_model_lib0_P->handrail_image_processing_pixel,
    sim_model_lib0_P->tun_cvs_perchcam_num_pixels_X,
    sim_model_lib0_P->tun_cvs_perchcam_num_pixels_Y,
    sim_model_lib0_P->cvs_perchcam_min_dist,
    sim_model_lib0_P->cvs_perchcam_max_dist,
    sim_model_lib0_P->cvs_perchcam_pointing);

  // End of Outputs for SubSystem: '<S113>/pinhole_projection_model'

  // Sum: '<S113>/Sum1'
  // MATLAB Function 'image_processing/generate_output': '<S114>:1'
  //  Inputs:
  //    Process type:                                                                [Parameter] 
  //        1=Landmarks, 2=AR Tags, 3=Optical Flow, 4=Handrail
  //    num_pts_out         = Required number of points to be reported               [Parameter] 
  //    P_point_iss_iss     = ALL [x,y,z] position of points in the ISS frame        **Landmark only** 
  //    P_point_cam_cam     = ALL [x,y,z] position of points in the CAM frame        **Handrail only** 
  //    P_points_2D_cam     = ALL [u,v] pixel location of points in the camera       **Landmark and OF only** 
  //    valid_flag          = indicates which points are valid
  //    num_hist_kept       = Required number of histories kept for Optical Flow     [Parameter] **OF only** 
  //    flow_ids            = ID tag for each optical flow point                     [Parameter] **OF only** 
  //
  //  Outputs:
  //    points_out_iss      = output [x,y,z] position of points in the ISS frame     **This output unused by optical flow** 
  //    points_out_2D_cam   = output [u,v] pixel location of points in the camera    **This output unused by handrail** 
  //    points_out_cam      = output [x,y,z] position of points in the CAM frame     **This output unused by OF and landmarks** 
  //    valid_out           = indicates which values of the output are valid!        **This output unused by landmarks** 
  //    ids_out             = optical flow IDs
  //    registration_id     = visual odometery registration pulse ID                 **ONLY used by OF** 
  // handrail
  // '<S114>:1:25'
  for (i_0 = 0; i_0 < 1512; i_0++) {
    sim_model_lib0_B->Add_l[i_0] = sim_model_lib0_B->P_point_B_iss_p[i_0] +
      rtb_Sum;
  }

  // End of Sum: '<S113>/Sum1'

  // MATLAB Function: '<S81>/generate_output' incorporates:
  //   Constant: '<S113>/Constant3'

  sim_mode_format_handrail_output(sim_model_lib0_P->cvs_handrail_map_iss,
    sim_model_lib0_B->Add_l, sim_model_lib0_B->ImpAsg_InsertedFor_points_in__n,
    sim_model_lib0_B->points_out_iss_h, sim_model_lib0_B->points_out_3D_cam_n,
    valid_out_i, sim_model_lib0_B, sim_model_lib0_DW);

  // DataTypeConversion: '<S112>/Data Type Conversion4' incorporates:
  //   Constant: '<S112>/Constant3'
  //   Logic: '<S112>/Logical Operator'
  //   Selector: '<S112>/select_current_command'
  //   UnitDelay: '<S112>/Unit Delay'

  // '<S114>:1:25'
  // '<S114>:1:26'
  // '<S114>:1:27'
  // '<S114>:1:28'
  // end of function
  for (i = 0; i < 50; i++) {
    rtb_DataTypeConversion4[i] = (int8_T)((valid_out_i[i] != 0.0F) &&
      sim_model_lib0_P->cvs_handrail_valid_mask[(int32_T)((int32_T)
      sim_model_lib0_DW->UnitDelay_DSTATE_g - 1)]);
  }

  // End of DataTypeConversion: '<S112>/Data Type Conversion4'

  // Sum: '<S112>/Sum1' incorporates:
  //   Constant: '<S112>/Constant1'
  //   Constant: '<S115>/Constant'
  //   Logic: '<S112>/Logical Operator1'
  //   Logic: '<S112>/Logical Operator2'
  //   Logic: '<S112>/Logical Operator3'
  //   RelationalOperator: '<S112>/Relational Operator'
  //   RelationalOperator: '<S112>/Relational Operator1'
  //   RelationalOperator: '<S112>/Relational Operator2'
  //   RelationalOperator: '<S115>/Compare'
  //   Selector: '<S112>/Select_col_1'
  //   Selector: '<S112>/Select_col_2'

  i_0 = (int32_T)(uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->cvs_handrail_valid_times[0] !=
      sim_model_lib0_P->Constant_Value_c0) &&
     ((sim_model_lib0_P->cvs_handrail_valid_times[0] <
       sim_model_lib0_B->RateTransition4_m.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_handrail_valid_times[0] ==
        sim_model_lib0_B->RateTransition4_m.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_m.timestamp_nsec >=
        sim_model_lib0_P->cvs_handrail_valid_times[3])))) + (uint32_T)
    ((sim_model_lib0_P->cvs_handrail_valid_times[1] !=
      sim_model_lib0_P->Constant_Value_c0) &&
     ((sim_model_lib0_P->cvs_handrail_valid_times[1] <
       sim_model_lib0_B->RateTransition4_m.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_handrail_valid_times[1] ==
        sim_model_lib0_B->RateTransition4_m.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_m.timestamp_nsec >=
        sim_model_lib0_P->cvs_handrail_valid_times[4]))))) + (uint32_T)
    ((sim_model_lib0_P->cvs_handrail_valid_times[2] !=
      sim_model_lib0_P->Constant_Value_c0) &&
     ((sim_model_lib0_P->cvs_handrail_valid_times[2] <
       sim_model_lib0_B->RateTransition4_m.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_handrail_valid_times[2] ==
        sim_model_lib0_B->RateTransition4_m.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_m.timestamp_nsec >=
        sim_model_lib0_P->cvs_handrail_valid_times[5])))));

  // Update for RateTransition: '<S81>/Rate Transition3'
  sim_model_lib0_DW->RateTransition3_Buffer0 =
    sim_model_lib0_B->RateTransition4_m.timestamp_sec;

  // Update for RateTransition: '<S81>/Rate Transition1'
  sim_model_lib0_DW->RateTransition1_Buffer0 =
    sim_model_lib0_B->RateTransition4_m.timestamp_nsec;

  // Update for RateTransition: '<S81>/Rate Transition5'
  memcpy(&sim_model_lib0_DW->RateTransition5_Buffer0[0],
         &sim_model_lib0_B->points_out_iss_h[0], (uint32_T)(150U * sizeof
          (real32_T)));

  // Update for RateTransition: '<S81>/Rate Transition9'
  memcpy(&sim_model_lib0_DW->RateTransition9_Buffer0[0],
         &sim_model_lib0_B->points_out_3D_cam_n[0], (uint32_T)(150U * sizeof
          (real32_T)));

  // Update for RateTransition: '<S81>/Rate Transition7'
  for (i = 0; i < 50; i++) {
    sim_model_lib0_DW->RateTransition7_Buffer0[i] = (uint8_T)
      rtb_DataTypeConversion4[i];
  }

  // End of Update for RateTransition: '<S81>/Rate Transition7'

  // Update for RandomNumber: '<S113>/pixel_noise'
  sim_model_lib0_DW->NextOutput_o = rt_nrand_Upu32_Yd_f_pw_snf
    (&sim_model_lib0_DW->RandSeed_d) * sqrt
    (sim_model_lib0_P->cvs_handrail_noise_var) +
    sim_model_lib0_P->pixel_noise_Mean_a;

  // Saturate: '<S112>/Saturation' incorporates:
  //   Sum: '<S112>/Sum1'

  if ((uint8_T)i_0 > sim_model_lib0_P->Saturation_UpperSat_n) {
    // Update for UnitDelay: '<S112>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_g =
      sim_model_lib0_P->Saturation_UpperSat_n;
  } else if ((uint8_T)i_0 < sim_model_lib0_P->Saturation_LowerSat_f) {
    // Update for UnitDelay: '<S112>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_g =
      sim_model_lib0_P->Saturation_LowerSat_f;
  } else {
    // Update for UnitDelay: '<S112>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_g = (uint8_T)i_0;
  }

  // End of Saturate: '<S112>/Saturation'
}

// Model step function for TID3
void sim_model_lib0_step3(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M) // Sample time: [0.272s, 0.0s] 
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);
  boolean_T replace_these_points[50];
  real_T num_points_needed;
  real_T num_points_to_use;
  int32_T nb;
  int32_T khi;
  static const int8_T l[4] = { 2, 10, 13, 15 };

  real32_T rtb_Conversion_h[9];
  real32_T rtb_Subtract1[3];
  real32_T rtb_MatrixConcatenate[16];
  real32_T rtb_Product[4];
  real32_T rtb_Sum;
  real32_T rtb_Assignment[9];
  real32_T rtb_Sum_g5;
  uint8_T valid_out_h[800];
  int32_T i;
  boolean_T tmp;
  real32_T tmp_0[9];
  real32_T rtb_Assignment_pv[9];
  real32_T tmp_1[9];
  real32_T rtb_Product_f[9];
  int32_T loop_ub;
  int32_T valid_points_in_cam_sizes[2];
  real32_T new_ids_data[50];
  uint8_T new_valid_flags_data[800];
  int32_T random_order_sizes[2];
  int32_T shuffled_ids_sizes;
  int8_T d_data[50];
  int32_T indx_sizes;
  int32_T r_sizes;
  int32_T ia_data[50];
  int32_T ib_data[50];
  uint8_T tmp_data[50];
  uint8_T tmp_data_0[750];
  int32_T tmp_sizes[2];
  real32_T valid_points_in_cam_data[100];
  int32_T valid_points_in_cam_sizes_0[2];
  int32_T tmp_sizes_0[3];
  uint8_T tmp_data_1[450];
  uint8_T tmp_data_2[300];
  int32_T tmp_sizes_1[2];
  int32_T tmp_sizes_2[2];
  real32_T tmp_data_3[900];
  int32_T tmp_sizes_3[3];
  real32_T tmp_data_4[600];
  int32_T tmp_sizes_4[3];
  int32_T tmp_sizes_5[3];
  uint8_T tmp_data_5[600];
  uint8_T tmp_data_6[150];
  real32_T tmp_data_7[300];
  int8_T tmp_data_8[50];
  real32_T tmp_2;
  real32_T tmp_3;
  int32_T new_valid_flags_sizes_idx_0;
  real32_T rtb_Product_e;

  // DataTypeConversion: '<S170>/Conversion' incorporates:
  //   Constant: '<S169>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Conversion_h[i] = (real32_T)sim_model_lib0_P->Constant2_Value_d4[i];
  }

  // End of DataTypeConversion: '<S170>/Conversion'

  // Assignment: '<S169>/Assignment' incorporates:
  //   Constant: '<S164>/Constant3'

  rtb_Conversion_h[0] = sim_model_lib0_P->tun_abp_q_body2navcam[3];
  rtb_Conversion_h[4] = sim_model_lib0_P->tun_abp_q_body2navcam[3];
  rtb_Conversion_h[8] = sim_model_lib0_P->tun_abp_q_body2navcam[3];

  // Sum: '<S169>/Sum2' incorporates:
  //   Constant: '<S164>/Constant3'
  //   Constant: '<S171>/Constant3'
  //   DataTypeConversion: '<S172>/Conversion'
  //   Gain: '<S171>/Gain'
  //   Gain: '<S171>/Gain1'
  //   Gain: '<S171>/Gain2'

  rtb_Product_f[0] = (real32_T)sim_model_lib0_P->Constant3_Value_g;
  rtb_Product_f[1] = sim_model_lib0_P->tun_abp_q_body2navcam[2];
  rtb_Product_f[2] = sim_model_lib0_P->Gain_Gain_mw *
    sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_Product_f[3] = sim_model_lib0_P->Gain1_Gain_b *
    sim_model_lib0_P->tun_abp_q_body2navcam[2];
  rtb_Product_f[4] = (real32_T)sim_model_lib0_P->Constant3_Value_g;
  rtb_Product_f[5] = sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_Product_f[6] = sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_Product_f[7] = sim_model_lib0_P->Gain2_Gain_ch *
    sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_Product_f[8] = (real32_T)sim_model_lib0_P->Constant3_Value_g;

  // Concatenate: '<S169>/Matrix Concatenate' incorporates:
  //   Constant: '<S164>/Constant3'
  //   Gain: '<S169>/Gain1'
  //   Sum: '<S169>/Sum2'

  for (khi = 0; khi < 3; khi++) {
    rtb_MatrixConcatenate[(int32_T)(khi << 2)] = rtb_Conversion_h[(int32_T)(3 *
      khi)] + rtb_Product_f[(int32_T)(3 * khi)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(khi << 2))] =
      rtb_Conversion_h[(int32_T)((int32_T)(3 * khi) + 1)] + rtb_Product_f
      [(int32_T)((int32_T)(3 * khi) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(khi << 2))] =
      rtb_Conversion_h[(int32_T)((int32_T)(3 * khi) + 2)] + rtb_Product_f
      [(int32_T)((int32_T)(3 * khi) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_ll *
    sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_ll *
    sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_ll *
    sim_model_lib0_P->tun_abp_q_body2navcam[2];

  // End of Concatenate: '<S169>/Matrix Concatenate'

  // Switch: '<S164>/Switch' incorporates:
  //   Constant: '<S164>/Constant'

  tmp = ((int32_T)sim_model_lib0_P->tun_cvs_noise_on != 0);

  // Reshape: '<S166>/Reshape1' incorporates:
  //   Constant: '<S164>/Constant3'

  rtb_MatrixConcatenate[12] = sim_model_lib0_P->tun_abp_q_body2navcam[0];

  // Product: '<S166>/Product' incorporates:
  //   Constant: '<S164>/Constant6'
  //   Constant: '<S164>/Constant7'
  //   Switch: '<S164>/Switch'

  if (tmp) {
    rtb_Sum = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[0];
    rtb_Sum_g5 = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[1];
  } else {
    rtb_Sum = sim_model_lib0_P->Constant7_Value_m[0];
    rtb_Sum_g5 = sim_model_lib0_P->Constant7_Value_m[1];
  }

  // Reshape: '<S166>/Reshape1' incorporates:
  //   Constant: '<S164>/Constant3'

  rtb_MatrixConcatenate[13] = sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_MatrixConcatenate[14] = sim_model_lib0_P->tun_abp_q_body2navcam[2];

  // Product: '<S166>/Product' incorporates:
  //   Constant: '<S164>/Constant6'
  //   Constant: '<S164>/Constant7'
  //   Switch: '<S164>/Switch'

  if (tmp) {
    tmp_3 = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[2];
    tmp_2 = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[3];
  } else {
    tmp_3 = sim_model_lib0_P->Constant7_Value_m[2];
    tmp_2 = sim_model_lib0_P->Constant7_Value_m[3];
  }

  // Reshape: '<S166>/Reshape1' incorporates:
  //   Constant: '<S164>/Constant3'

  rtb_MatrixConcatenate[15] = sim_model_lib0_P->tun_abp_q_body2navcam[3];

  // Product: '<S166>/Product'
  for (khi = 0; khi < 4; khi++) {
    rtb_Product_e = rtb_MatrixConcatenate[(int32_T)(khi + 12)] * tmp_2 +
      (rtb_MatrixConcatenate[(int32_T)(khi + 8)] * tmp_3 +
       (rtb_MatrixConcatenate[(int32_T)(khi + 4)] * rtb_Sum_g5 +
        rtb_MatrixConcatenate[khi] * rtb_Sum));
    rtb_Product[khi] = rtb_Product_e;
  }

  // Sum: '<S168>/Sum' incorporates:
  //   Constant: '<S168>/Constant1'
  //   DataTypeConversion: '<S178>/Conversion'
  //   Gain: '<S168>/Gain'
  //   Math: '<S168>/Math Function'

  rtb_Sum = rtb_Product[3] * rtb_Product[3] * sim_model_lib0_P->Gain_Gain_ci -
    (real32_T)sim_model_lib0_P->Constant1_Value_h;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S168>/Assignment' incorporates:
    //   Constant: '<S168>/Constant2'
    //   DataTypeConversion: '<S177>/Conversion'

    rtb_Conversion_h[i] = (real32_T)sim_model_lib0_P->Constant2_Value_kp[i];

    // Assignment: '<S167>/Assignment' incorporates:
    //   Constant: '<S167>/Constant2'
    //   DataTypeConversion: '<S173>/Conversion'

    rtb_Assignment[i] = (real32_T)sim_model_lib0_P->Constant2_Value_e[i];
  }

  // Assignment: '<S168>/Assignment'
  rtb_Conversion_h[0] = rtb_Sum;
  rtb_Conversion_h[4] = rtb_Sum;
  rtb_Conversion_h[8] = rtb_Sum;

  // Gain: '<S168>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_fm * rtb_Product[3];

  // Sum: '<S167>/Sum' incorporates:
  //   Constant: '<S167>/Constant1'
  //   DataTypeConversion: '<S174>/Conversion'
  //   Gain: '<S167>/Gain'
  //   Math: '<S167>/Math Function'

  rtb_Sum_g5 = sim_model_lib0_B->RateTransition2_b.Q_ISS2B[3] *
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[3] *
    sim_model_lib0_P->Gain_Gain_fp - (real32_T)
    sim_model_lib0_P->Constant1_Value_es;

  // Assignment: '<S167>/Assignment'
  rtb_Assignment[0] = rtb_Sum_g5;
  rtb_Assignment[4] = rtb_Sum_g5;
  rtb_Assignment[8] = rtb_Sum_g5;

  // Gain: '<S167>/Gain1'
  rtb_Sum_g5 = sim_model_lib0_P->Gain1_Gain_in *
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[3];

  // Sum: '<S164>/Add' incorporates:
  //   Constant: '<S161>/Constant3'
  //   Constant: '<S164>/Constant4'
  //   Gain: '<S164>/Gain'

  for (i = 0; i < 11937; i++) {
    sim_model_lib0_B->Add_f[i] = (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_optflow_map_error[i] +
      sim_model_lib0_P->cvs_optflow_map_iss[i];
  }

  // End of Sum: '<S164>/Add'
  for (khi = 0; khi < 3979; khi++) {
    // Sum: '<S164>/Subtract' incorporates:
    //   Selector: '<S164>/Select_X'

    sim_model_lib0_B->P_point_B_iss_k[khi] = sim_model_lib0_B->Add_f[khi] -
      sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[0];

    // Sum: '<S164>/Subtract2' incorporates:
    //   Selector: '<S164>/Select_Y'

    sim_model_lib0_B->P_point_B_iss_k[(int32_T)(3979 + khi)] =
      sim_model_lib0_B->Add_f[(int32_T)(3979 + khi)] -
      sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[1];

    // Sum: '<S164>/Subtract3' incorporates:
    //   Selector: '<S164>/Select_Z'

    sim_model_lib0_B->P_point_B_iss_k[(int32_T)(7958 + khi)] =
      sim_model_lib0_B->Add_f[(int32_T)(7958 + khi)] -
      sim_model_lib0_B->RateTransition2_b.P_B_ISS_ISS[2];
  }

  // Product: '<S167>/Product' incorporates:
  //   Constant: '<S175>/Constant3'
  //   DataTypeConversion: '<S176>/Conversion'
  //   Gain: '<S175>/Gain'
  //   Gain: '<S175>/Gain1'
  //   Gain: '<S175>/Gain2'

  tmp_0[0] = (real32_T)sim_model_lib0_P->Constant3_Value_kj;
  tmp_0[1] = sim_model_lib0_B->RateTransition2_b.Q_ISS2B[2];
  tmp_0[2] = sim_model_lib0_P->Gain_Gain_lw *
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[1];
  tmp_0[3] = sim_model_lib0_P->Gain1_Gain_jqn *
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[2];
  tmp_0[4] = (real32_T)sim_model_lib0_P->Constant3_Value_kj;
  tmp_0[5] = sim_model_lib0_B->RateTransition2_b.Q_ISS2B[0];
  tmp_0[6] = sim_model_lib0_B->RateTransition2_b.Q_ISS2B[1];
  tmp_0[7] = sim_model_lib0_P->Gain2_Gain_jj *
    sim_model_lib0_B->RateTransition2_b.Q_ISS2B[0];
  tmp_0[8] = (real32_T)sim_model_lib0_P->Constant3_Value_kj;

  // Product: '<S167>/Product1' incorporates:
  //   Gain: '<S167>/Gain2'
  //   Math: '<S167>/Math Function1'

  for (khi = 0; khi < 3; khi++) {
    rtb_Product_f[khi] = sim_model_lib0_B->RateTransition2_b.Q_ISS2B[khi] *
      sim_model_lib0_B->RateTransition2_b.Q_ISS2B[0];
    rtb_Product_f[(int32_T)(khi + 3)] =
      sim_model_lib0_B->RateTransition2_b.Q_ISS2B[khi] *
      sim_model_lib0_B->RateTransition2_b.Q_ISS2B[1];
    rtb_Product_f[(int32_T)(khi + 6)] =
      sim_model_lib0_B->RateTransition2_b.Q_ISS2B[khi] *
      sim_model_lib0_B->RateTransition2_b.Q_ISS2B[2];
  }

  // End of Product: '<S167>/Product1'

  // Sum: '<S167>/Sum1' incorporates:
  //   Gain: '<S167>/Gain2'
  //   Product: '<S164>/Product'
  //   Product: '<S167>/Product'

  for (khi = 0; khi < 3; khi++) {
    rtb_Assignment_pv[(int32_T)(3 * khi)] = (rtb_Assignment[(int32_T)(3 * khi)]
      - tmp_0[(int32_T)(3 * khi)] * rtb_Sum_g5) + rtb_Product_f[(int32_T)(3 *
      khi)] * sim_model_lib0_P->Gain2_Gain_dk;
    rtb_Assignment_pv[(int32_T)(1 + (int32_T)(3 * khi))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * khi) + 1)] - tmp_0[(int32_T)((int32_T)(3 * khi) +
      1)] * rtb_Sum_g5) + rtb_Product_f[(int32_T)((int32_T)(3 * khi) + 1)] *
      sim_model_lib0_P->Gain2_Gain_dk;
    rtb_Assignment_pv[(int32_T)(2 + (int32_T)(3 * khi))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * khi) + 2)] - tmp_0[(int32_T)((int32_T)(3 * khi) +
      2)] * rtb_Sum_g5) + rtb_Product_f[(int32_T)((int32_T)(3 * khi) + 2)] *
      sim_model_lib0_P->Gain2_Gain_dk;
  }

  // End of Sum: '<S167>/Sum1'

  // Product: '<S168>/Product' incorporates:
  //   Constant: '<S179>/Constant3'
  //   DataTypeConversion: '<S180>/Conversion'
  //   Gain: '<S179>/Gain'
  //   Gain: '<S179>/Gain1'
  //   Gain: '<S179>/Gain2'

  tmp_1[0] = (real32_T)sim_model_lib0_P->Constant3_Value_c2;
  tmp_1[1] = rtb_Product[2];
  tmp_1[2] = sim_model_lib0_P->Gain_Gain_d5 * rtb_Product[1];
  tmp_1[3] = sim_model_lib0_P->Gain1_Gain_hfz * rtb_Product[2];
  tmp_1[4] = (real32_T)sim_model_lib0_P->Constant3_Value_c2;
  tmp_1[5] = rtb_Product[0];
  tmp_1[6] = rtb_Product[1];
  tmp_1[7] = sim_model_lib0_P->Gain2_Gain_ee * rtb_Product[0];
  tmp_1[8] = (real32_T)sim_model_lib0_P->Constant3_Value_c2;
  for (i = 0; i < 3; i++) {
    // Product: '<S164>/Product' incorporates:
    //   Math: '<S164>/Math Function'

    for (khi = 0; khi < 3979; khi++) {
      sim_model_lib0_B->Add_f[(int32_T)(i + (int32_T)(3 * khi))] = 0.0F;
      sim_model_lib0_B->Add_f[(int32_T)(i + (int32_T)(3 * khi))] +=
        rtb_Assignment_pv[i] * sim_model_lib0_B->P_point_B_iss_k[khi];
      sim_model_lib0_B->Add_f[(int32_T)(i + (int32_T)(3 * khi))] +=
        rtb_Assignment_pv[(int32_T)(i + 3)] * sim_model_lib0_B->P_point_B_iss_k
        [(int32_T)(khi + 3979)];
      sim_model_lib0_B->Add_f[(int32_T)(i + (int32_T)(3 * khi))] +=
        rtb_Assignment_pv[(int32_T)(i + 6)] * sim_model_lib0_B->P_point_B_iss_k
        [(int32_T)(khi + 7958)];
    }

    // Sum: '<S164>/Subtract1' incorporates:
    //   Constant: '<S164>/Constant1'
    //   Constant: '<S164>/Constant2'
    //   Gain: '<S164>/Gain1'

    rtb_Subtract1[i] = sim_model_lib0_P->tun_abp_p_navcam_body_body_sim[i] -
      (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_navcam_P_B_B_error[i];

    // Product: '<S168>/Product1' incorporates:
    //   Gain: '<S168>/Gain2'
    //   Math: '<S168>/Math Function1'

    rtb_Product_f[i] = rtb_Product[i] * rtb_Product[0];
    rtb_Product_f[(int32_T)(i + 3)] = rtb_Product[i] * rtb_Product[1];
    rtb_Product_f[(int32_T)(i + 6)] = rtb_Product[i] * rtb_Product[2];
  }

  // Sum: '<S168>/Sum1' incorporates:
  //   Gain: '<S168>/Gain2'
  //   Product: '<S164>/Product1'
  //   Product: '<S168>/Product'

  for (khi = 0; khi < 3; khi++) {
    rtb_Assignment_pv[(int32_T)(3 * khi)] = (rtb_Conversion_h[(int32_T)(3 * khi)]
      - tmp_1[(int32_T)(3 * khi)] * rtb_Sum) + rtb_Product_f[(int32_T)(3 * khi)]
      * sim_model_lib0_P->Gain2_Gain_he;
    rtb_Assignment_pv[(int32_T)(1 + (int32_T)(3 * khi))] = (rtb_Conversion_h
      [(int32_T)((int32_T)(3 * khi) + 1)] - tmp_1[(int32_T)((int32_T)(3 * khi) +
      1)] * rtb_Sum) + rtb_Product_f[(int32_T)((int32_T)(3 * khi) + 1)] *
      sim_model_lib0_P->Gain2_Gain_he;
    rtb_Assignment_pv[(int32_T)(2 + (int32_T)(3 * khi))] = (rtb_Conversion_h
      [(int32_T)((int32_T)(3 * khi) + 2)] - tmp_1[(int32_T)((int32_T)(3 * khi) +
      2)] * rtb_Sum) + rtb_Product_f[(int32_T)((int32_T)(3 * khi) + 2)] *
      sim_model_lib0_P->Gain2_Gain_he;
  }

  // End of Sum: '<S168>/Sum1'
  for (khi = 0; khi < 3979; khi++) {
    // Concatenate: '<S164>/Matrix Concatenate1' incorporates:
    //   Product: '<S164>/Product1'
    //   Selector: '<S164>/Select_X1'
    //   Selector: '<S164>/Select_Y1'
    //   Selector: '<S164>/Select_Z1'
    //   Sum: '<S164>/Subtract4'
    //   Sum: '<S164>/Subtract5'
    //   Sum: '<S164>/Subtract6'

    sim_model_lib0_B->rtb_Add_f_b[(int32_T)(3 * khi)] = sim_model_lib0_B->Add_f
      [(int32_T)(3 * khi)] - rtb_Subtract1[0];
    sim_model_lib0_B->rtb_Add_f_b[(int32_T)(1 + (int32_T)(3 * khi))] =
      sim_model_lib0_B->Add_f[(int32_T)((int32_T)(3 * khi) + 1)] -
      rtb_Subtract1[1];
    sim_model_lib0_B->rtb_Add_f_b[(int32_T)(2 + (int32_T)(3 * khi))] =
      sim_model_lib0_B->Add_f[(int32_T)((int32_T)(3 * khi) + 2)] -
      rtb_Subtract1[2];

    // Math: '<S164>/Math Function1' incorporates:
    //   Product: '<S164>/Product1'

    for (indx_sizes = 0; indx_sizes < 3; indx_sizes++) {
      sim_model_lib0_B->P_point_B_iss_k[(int32_T)(khi + (int32_T)(3979 *
        indx_sizes))] = 0.0F;
      sim_model_lib0_B->P_point_B_iss_k[(int32_T)(khi + (int32_T)(3979 *
        indx_sizes))] += sim_model_lib0_B->rtb_Add_f_b[(int32_T)(3 * khi)] *
        rtb_Assignment_pv[indx_sizes];
      sim_model_lib0_B->P_point_B_iss_k[(int32_T)(khi + (int32_T)(3979 *
        indx_sizes))] += sim_model_lib0_B->rtb_Add_f_b[(int32_T)((int32_T)(3 *
        khi) + 1)] * rtb_Assignment_pv[(int32_T)(indx_sizes + 3)];
      sim_model_lib0_B->P_point_B_iss_k[(int32_T)(khi + (int32_T)(3979 *
        indx_sizes))] += sim_model_lib0_B->rtb_Add_f_b[(int32_T)((int32_T)(3 *
        khi) + 2)] * rtb_Assignment_pv[(int32_T)(indx_sizes + 6)];
    }

    // End of Math: '<S164>/Math Function1'
  }

  // Outputs for Iterator SubSystem: '<S161>/pinhole_projection_model'
  si_pinhole_projection_model(3979, sim_model_lib0_B->P_point_B_iss_k,
    sim_model_lib0_B->ImpAsg_InsertedFor_P_points_2D_,
    sim_model_lib0_B->ImpAsg_InsertedFor_points_in_FO,
    sim_model_lib0_DW->pinhole_projection_model_o,
    (P_pinhole_projection_model_si_T *)
    &sim_model_lib0_P->pinhole_projection_model_o,
    sim_model_lib0_P->tun_cvs_noise_on,
    sim_model_lib0_P->tun_cvs_navcam_focal_length_Y,
    sim_model_lib0_P->tun_cvs_navcam_focal_length_X,
    sim_model_lib0_P->optical_flow_image_processing_p,
    sim_model_lib0_P->tun_cvs_navcam_num_pixels_X,
    sim_model_lib0_P->tun_cvs_navcam_num_pixels_Y,
    sim_model_lib0_P->cvs_navcam_min_dist, sim_model_lib0_P->cvs_navcam_max_dist,
    sim_model_lib0_P->cvs_navcam_pointing);

  // End of Outputs for SubSystem: '<S161>/pinhole_projection_model'

  // MATLAB Function: '<S83>/generate_output'
  // MATLAB Function 'image_processing/generate_output': '<S162>:1'
  //  Inputs:
  //    Process type:                                                                [Parameter] 
  //        1=Landmarks, 2=AR Tags, 3=Optical Flow, 4=Handrail
  //    num_pts_out         = Required number of points to be reported               [Parameter] 
  //    P_point_iss_iss     = ALL [x,y,z] position of points in the ISS frame        **Landmark only** 
  //    P_point_cam_cam     = ALL [x,y,z] position of points in the CAM frame        **Handrail only** 
  //    P_points_2D_cam     = ALL [u,v] pixel location of points in the camera       **Landmark and OF only** 
  //    valid_flag          = indicates which points are valid
  //    num_hist_kept       = Required number of histories kept for Optical Flow     [Parameter] **OF only** 
  //    flow_ids            = ID tag for each optical flow point                     [Parameter] **OF only** 
  //
  //  Outputs:
  //    points_out_iss      = output [x,y,z] position of points in the ISS frame     **This output unused by optical flow** 
  //    points_out_2D_cam   = output [u,v] pixel location of points in the camera    **This output unused by handrail** 
  //    points_out_cam      = output [x,y,z] position of points in the CAM frame     **This output unused by OF and landmarks** 
  //    valid_out           = indicates which values of the output are valid!        **This output unused by landmarks** 
  //    ids_out             = optical flow IDs
  //    registration_id     = visual odometery registration pulse ID                 **ONLY used by OF** 
  // optical flow
  // '<S162>:1:30'
  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  // format_visOD_output.m
  //  inputs:
  //    num_pts_out    = Required number of points to be reported
  //    points_in iss  = ALL [x,y,z] position of points in the ISS frame
  //    points_in_cam  = ALL [u,v] pixel location of points in the camera
  //    valid_in       = indicates which points are valid
  //    num_augments    = Required number of histories kept for Optical Flow (ASSUMED to be >=2) 
  //    flow_ids       = ID tag for each optical flow point (OF only)
  //
  //  outputs:
  //    observati_out  = output [u,v] pixel location of points in the camera
  //    valid_out      = indicates which values of the output are valid!
  //    ids_out        = optical flow IDs
  // if number of points out changes, change this number.
  // order of the replacement augmentations
  // note: real FSW has a fancy way to determine the last augmentation to replace that changes between 13, 14 or 15.  Here we simplify to just take 15 every time 
  i = 0;

  // Define Persistance Variables
  // Either Initalize the variables (1st time only), or shift the history over
  if (!sim_model_lib0_DW->int_id_hist_not_empty) {
    sim_model_lib0_DW->int_id_hist_not_empty = true;
    memset(&sim_model_lib0_DW->int_observations[0], 0, (uint32_T)(1600U * sizeof
            (real32_T)));
    memset(&sim_model_lib0_DW->int_valid_flag[0], 0, (uint32_T)(800U * sizeof
            (uint8_T)));
    sim_model_lib0_DW->int_registration_number = 1U;
    sim_model_lib0_DW->int_initalization_complete = 0.0;
  } else {
    // until 16 augmentations have been captured, just keep incrementing number.  Then after that, start rotating through the registration numbers 
    if (sim_model_lib0_DW->int_initalization_complete < 15.0) {
      sim_model_lib0_DW->int_initalization_complete++;
      memset(&valid_points_in_cam_data[0], 0, (uint32_T)(100U * sizeof(real32_T)));
      for (khi = 0; khi < 2; khi++) {
        for (indx_sizes = 0; indx_sizes < 15; indx_sizes++) {
          memcpy(&sim_model_lib0_B->tmp_data_l[(int32_T)((int32_T)(khi * 50) +
                  (int32_T)(indx_sizes * 100))],
                 &sim_model_lib0_DW->int_observations[(int32_T)((int32_T)(khi *
                   50) + (int32_T)(indx_sizes * 100))], (uint32_T)(50U * sizeof
                  (real32_T)));
        }
      }

      cbiephdblnopophl_cat(sim_model_lib0_B->tmp_data_l,
                           sim_model_lib0_DW->int_observations);
      memset(&new_ids_data[0], 0, (uint32_T)(50U * sizeof(real32_T)));
      for (khi = 0; khi < 15; khi++) {
        for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
          tmp_data_0[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
            sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(50 * khi) +
            indx_sizes)];
        }
      }

      mglflfcbcbimecje_cat(tmp_data_0, sim_model_lib0_DW->int_valid_flag);
      sim_model_lib0_DW->int_registration_number = 1U;
    } else {
      switch (sim_model_lib0_DW->int_registration_number) {
       case 2U:
        // the registration pulse should rotated between augmentations [0, 2, 6, 15] 
        sim_model_lib0_DW->int_registration_number = 10U;

        // remove the 3rd augmentation (zero based index)
        tmp_sizes_3[0] = 50;
        tmp_sizes_3[1] = 2;
        tmp_sizes_3[2] = 9;
        for (khi = 0; khi < 9; khi++) {
          for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
            memcpy(&tmp_data_3[(int32_T)((int32_T)(khi * 100) + (int32_T)
                    (indx_sizes * 50))], &sim_model_lib0_DW->int_observations
                   [(int32_T)((int32_T)(khi * 100) + (int32_T)(indx_sizes * 50))],
                   (uint32_T)(50U * sizeof(real32_T)));
          }
        }

        tmp_sizes_4[0] = 50;
        tmp_sizes_4[1] = 2;
        tmp_sizes_4[2] = 6;
        for (khi = 0; khi < 6; khi++) {
          for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
            memcpy(&tmp_data_4[(int32_T)((int32_T)(khi * 100) + (int32_T)
                    (indx_sizes * 50))], &sim_model_lib0_DW->int_observations
                   [(int32_T)((int32_T)((int32_T)(khi * 100) + (int32_T)
                     (indx_sizes * 50)) + 1000)], (uint32_T)(50U * sizeof
                    (real32_T)));
          }
        }

        ophlngdbgdjmgdje_cat(tmp_data_3, tmp_sizes_3, tmp_data_4, tmp_sizes_4,
                             sim_model_lib0_B->tmp_data_d, tmp_sizes_5);
        memcpy(&sim_model_lib0_DW->int_observations[0],
               &sim_model_lib0_B->tmp_data_d[0], (uint32_T)(1600U * sizeof
                (real32_T)));
        random_order_sizes[0] = 50;
        random_order_sizes[1] = 9;
        for (khi = 0; khi < 9; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_1[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(50 * khi) +
              indx_sizes)];
          }
        }

        tmp_sizes_1[0] = 50;
        tmp_sizes_1[1] = 6;
        for (khi = 0; khi < 6; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_2[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)((int32_T)(10
              + khi) * 50) + indx_sizes)];
          }
        }

        dbiehlnglnohaaai_cat(tmp_data_1, random_order_sizes, tmp_data_2,
                             tmp_sizes_1, valid_out_h, tmp_sizes_2);
        memcpy(&sim_model_lib0_DW->int_valid_flag[0], &valid_out_h[0], (uint32_T)
               (800U * sizeof(uint8_T)));
        break;

       case 10U:
        sim_model_lib0_DW->int_registration_number = 13U;

        // remove the 7th augmentation (zero based index)
        tmp_sizes_3[0] = 50;
        tmp_sizes_3[1] = 2;
        tmp_sizes_3[2] = 12;
        for (khi = 0; khi < 12; khi++) {
          for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
            memcpy(&sim_model_lib0_B->tmp_data_dh[(int32_T)((int32_T)(khi * 100)
                    + (int32_T)(indx_sizes * 50))],
                   &sim_model_lib0_DW->int_observations[(int32_T)((int32_T)(khi *
                     100) + (int32_T)(indx_sizes * 50))], (uint32_T)(50U *
                    sizeof(real32_T)));
          }
        }

        tmp_sizes_4[0] = 50;
        tmp_sizes_4[1] = 2;
        tmp_sizes_4[2] = 3;
        for (khi = 0; khi < 3; khi++) {
          for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
            memcpy(&tmp_data_7[(int32_T)((int32_T)(khi * 100) + (int32_T)
                    (indx_sizes * 50))], &sim_model_lib0_DW->int_observations
                   [(int32_T)((int32_T)((int32_T)(khi * 100) + (int32_T)
                     (indx_sizes * 50)) + 1300)], (uint32_T)(50U * sizeof
                    (real32_T)));
          }
        }

        ophlngdbgdjmgdje_cat(sim_model_lib0_B->tmp_data_dh, tmp_sizes_3,
                             tmp_data_7, tmp_sizes_4,
                             sim_model_lib0_B->tmp_data_d, tmp_sizes_5);
        memcpy(&sim_model_lib0_DW->int_observations[0],
               &sim_model_lib0_B->tmp_data_d[0], (uint32_T)(1600U * sizeof
                (real32_T)));
        random_order_sizes[0] = 50;
        random_order_sizes[1] = 12;
        for (khi = 0; khi < 12; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_5[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(50 * khi) +
              indx_sizes)];
          }
        }

        tmp_sizes_1[0] = 50;
        tmp_sizes_1[1] = 3;
        for (khi = 0; khi < 3; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_6[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)((int32_T)(13
              + khi) * 50) + indx_sizes)];
          }
        }

        dbiehlnglnohaaai_cat(tmp_data_5, random_order_sizes, tmp_data_6,
                             tmp_sizes_1, valid_out_h, tmp_sizes_2);
        memcpy(&sim_model_lib0_DW->int_valid_flag[0], &valid_out_h[0], (uint32_T)
               (800U * sizeof(uint8_T)));
        break;

       case 13U:
        sim_model_lib0_DW->int_registration_number = 15U;

        // remove the last augmentation (zero based index)
        memset(&valid_points_in_cam_data[0], 0, (uint32_T)(100U * sizeof
                (real32_T)));
        for (khi = 0; khi < 2; khi++) {
          for (indx_sizes = 0; indx_sizes < 15; indx_sizes++) {
            memcpy(&sim_model_lib0_B->tmp_data_l[(int32_T)((int32_T)(khi * 50) +
                    (int32_T)(indx_sizes * 100))],
                   &sim_model_lib0_DW->int_observations[(int32_T)((int32_T)(khi *
                     50) + (int32_T)(indx_sizes * 100))], (uint32_T)(50U *
                    sizeof(real32_T)));
          }
        }

        cbiephdblnopophl_cat(sim_model_lib0_B->tmp_data_l,
                             sim_model_lib0_DW->int_observations);
        memset(&new_ids_data[0], 0, (uint32_T)(50U * sizeof(real32_T)));
        for (khi = 0; khi < 15; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_0[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(50 * khi) +
              indx_sizes)];
          }
        }

        mglflfcbcbimecje_cat(tmp_data_0, sim_model_lib0_DW->int_valid_flag);
        break;

       default:
        i = 1;

        // after capturing the last image, set flag to report
        sim_model_lib0_DW->int_registration_number = 2U;

        // remove the first augmentation  (zero based index)
        memset(&valid_points_in_cam_data[0], 0, (uint32_T)(100U * sizeof
                (real32_T)));
        for (khi = 0; khi < 2; khi++) {
          for (indx_sizes = 0; indx_sizes < 15; indx_sizes++) {
            memcpy(&sim_model_lib0_B->tmp_data_l[(int32_T)((int32_T)(khi * 50) +
                    (int32_T)(indx_sizes * 100))],
                   &sim_model_lib0_DW->int_observations[(int32_T)((int32_T)
                    ((int32_T)(khi * 50) + (int32_T)(indx_sizes * 100)) + 100)],
                   (uint32_T)(50U * sizeof(real32_T)));
          }
        }

        cbiephdblnopophl_cat(sim_model_lib0_B->tmp_data_l,
                             sim_model_lib0_DW->int_observations);
        memset(&new_ids_data[0], 0, (uint32_T)(50U * sizeof(real32_T)));
        for (khi = 0; khi < 15; khi++) {
          for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
            tmp_data_0[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
              sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)((int32_T)(1
              + khi) * 50) + indx_sizes)];
          }
        }

        mglflfcbcbimecje_cat(tmp_data_0, sim_model_lib0_DW->int_valid_flag);
        break;
      }
    }
  }

  // pull out the currently valid points as marked by the valid flag input
  nb = 0;
  for (khi = 0; khi < 3979; khi++) {
    if (sim_model_lib0_B->ImpAsg_InsertedFor_points_in_FO[khi]) {
      nb++;
    }
  }

  shuffled_ids_sizes = nb;
  nb = 0;
  for (khi = 0; khi < 3979; khi++) {
    if (sim_model_lib0_B->ImpAsg_InsertedFor_points_in_FO[khi]) {
      sim_model_lib0_B->shuffled_ids_data[nb] = (int32_T)(khi + 1);
      nb++;
    }
  }

  nb = 0;
  for (khi = 0; khi < 3979; khi++) {
    if (sim_model_lib0_B->ImpAsg_InsertedFor_points_in_FO[khi]) {
      nb++;
    }
  }

  indx_sizes = nb;
  nb = 0;
  for (khi = 0; khi < 3979; khi++) {
    if (sim_model_lib0_B->ImpAsg_InsertedFor_points_in_FO[khi]) {
      sim_model_lib0_B->indx_data_g[nb] = (int32_T)(khi + 1);
      nb++;
    }
  }

  valid_points_in_cam_sizes[0] = indx_sizes;
  valid_points_in_cam_sizes[1] = 2;
  for (khi = 0; khi <= (int32_T)(indx_sizes - 1); khi++) {
    sim_model_lib0_B->valid_points_in_cam_data[khi] =
      sim_model_lib0_B->ImpAsg_InsertedFor_P_points_2D_[(int32_T)
      (sim_model_lib0_B->indx_data_g[khi] - 1)];
  }

  for (khi = 0; khi <= (int32_T)(indx_sizes - 1); khi++) {
    sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(khi + indx_sizes)] =
      sim_model_lib0_B->ImpAsg_InsertedFor_P_points_2D_[(int32_T)
      (sim_model_lib0_B->indx_data_g[khi] + 3978)];
  }

  // Find unique (and valid) 2D points in the cam frame.  Can be thought of as either: 
  //    a) points blocked by other points are not visible to the camera, despite technically being in the FOV of the camera 
  //    b) Only can have 1 point reported per pixel location
  if (indx_sizes != 0) {
    sim_m_mglflfcbkfkfbiec_sortrows(sim_model_lib0_B->valid_points_in_cam_data,
      valid_points_in_cam_sizes, sim_model_lib0_B->idx_data_c, &khi,
      sim_model_lib0_B);
    nb = 0;
    khi = 1;
    while (khi <= indx_sizes) {
      r_sizes = khi;
      do {
        khi++;
      } while (!((khi > indx_sizes) || fcbadjmgohlnpphl_rows_differ
                 (sim_model_lib0_B->valid_points_in_cam_data,
                  valid_points_in_cam_sizes, r_sizes, khi)));

      nb++;
      sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(nb - 1)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(r_sizes - 1)];
      sim_model_lib0_B->valid_points_in_cam_data[(int32_T)((int32_T)(nb +
        valid_points_in_cam_sizes[0]) - 1)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)((int32_T)(r_sizes +
        valid_points_in_cam_sizes[0]) - 1)];
      sim_model_lib0_B->idx_data_c[(int32_T)(nb - 1)] =
        sim_model_lib0_B->idx_data_c[(int32_T)(r_sizes - 1)];
    }

    if (1 > nb) {
      loop_ub = 0;
    } else {
      loop_ub = nb;
    }

    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data_f[khi] =
        sim_model_lib0_B->valid_points_in_cam_data[khi];
    }

    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi + loop_ub)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(khi +
        valid_points_in_cam_sizes[0])];
    }

    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data[khi] =
        sim_model_lib0_B->valid_points_in_cam_data_f[khi];
    }

    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(khi + loop_ub)] =
        sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi + loop_ub)];
    }

    indx_sizes = nb;
    for (khi = 0; (int32_T)(khi + 1) <= nb; khi++) {
      sim_model_lib0_B->indx_data_g[khi] = (int32_T)sim_model_lib0_B->
        idx_data_c[khi];
    }

    sim_mo_dbaaphlnfkfkphdb_sortIdx(sim_model_lib0_B->indx_data_g, nb,
      sim_model_lib0_B->r_data, &r_sizes, sim_model_lib0_B);
    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data_f[khi] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)
        (sim_model_lib0_B->r_data[khi] - 1)];
    }

    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi + r_sizes)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)((int32_T)
        (sim_model_lib0_B->r_data[khi] + loop_ub) - 1)];
    }

    valid_points_in_cam_sizes[0] = r_sizes;
    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data[khi] =
        sim_model_lib0_B->valid_points_in_cam_data_f[khi];
    }

    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(khi + r_sizes)] =
        sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi + r_sizes)];
    }

    for (khi = 0; (int32_T)(khi + 1) <= nb; khi++) {
      sim_model_lib0_B->indx_data_g[khi] = (int32_T)sim_model_lib0_B->
        idx_data_c[(int32_T)(sim_model_lib0_B->r_data[khi] - 1)];
    }
  }

  // finds unique rows, and 'stable' preserves the current order instead of sorting 
  for (khi = 0; khi <= (int32_T)(shuffled_ids_sizes - 1); khi++) {
    sim_model_lib0_B->tmp_data[khi] = (int16_T)
      sim_model_lib0_B->shuffled_ids_data[khi];
  }

  for (khi = 0; khi <= (int32_T)(indx_sizes - 1); khi++) {
    sim_model_lib0_B->valid_ids_data[khi] = (real32_T)sim_model_lib0_B->
      tmp_data[(int32_T)(sim_model_lib0_B->indx_data_g[khi] - 1)];
  }

  // If any historical valid points are currently valid, capture this info
  hdbiaaaabiecmgdb_do_vectors(sim_model_lib0_DW->int_id_hist,
    sim_model_lib0_B->valid_ids_data, indx_sizes, new_ids_data, &khi, ia_data,
    &shuffled_ids_sizes, ib_data, &r_sizes);
  if (!(shuffled_ids_sizes == 0)) {
    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_DW->int_observations[(int32_T)(ia_data[khi] - 1)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)(ib_data[khi] - 1)];
    }

    for (khi = 0; khi <= (int32_T)(r_sizes - 1); khi++) {
      sim_model_lib0_DW->int_observations[(int32_T)(ia_data[khi] + 49)] =
        sim_model_lib0_B->valid_points_in_cam_data[(int32_T)((int32_T)
        (ib_data[khi] + valid_points_in_cam_sizes[0]) - 1)];
    }

    for (khi = 0; khi <= (int32_T)(shuffled_ids_sizes - 1); khi++) {
      sim_model_lib0_DW->int_valid_flag[(int32_T)(ia_data[khi] - 1)] = 1U;
    }
  }

  // Determine which points need to be replaced.  Keep any points that are still valid this history, 
  for (khi = 0; khi < 50; khi++) {
    replace_these_points[khi] = ((int32_T)sim_model_lib0_DW->int_valid_flag[khi]
      == 0);
  }

  num_points_needed = jecjcjmgfkfkophl_sum(replace_these_points);
  if ((50.0 <= num_points_needed) || rtIsNaN(num_points_needed)) {
    num_points_needed = 50.0;
  }

  // Replace any points needed
  if (num_points_needed > 0.0) {
    // initialize new slots with zeros (since we may not fill it, want the remaining to be zeros already) 
    loop_ub = (int32_T)num_points_needed;
    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      new_ids_data[khi] = 0.0F;
    }

    r_sizes = (int32_T)num_points_needed;
    loop_ub = (int32_T)((int32_T)num_points_needed << 5);
    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      sim_model_lib0_B->new_observations_data[khi] = 0.0F;
    }

    new_valid_flags_sizes_idx_0 = (int32_T)num_points_needed;
    loop_ub = (int32_T)((int32_T)num_points_needed << 4);
    for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
      new_valid_flags_data[khi] = 0U;
    }

    // find the unused valid points (points not already in augmentations)
    sim_fkfchlfkgdbimgdj_do_vectors(sim_model_lib0_DW->int_id_hist,
      sim_model_lib0_B->valid_ids_data, indx_sizes, sim_model_lib0_B->b_c_data,
      &khi, ia_data, &shuffled_ids_sizes, sim_model_lib0_B->indx_data_g,
      &indx_sizes, sim_model_lib0_B);
    for (khi = 0; khi <= (int32_T)(indx_sizes - 1); khi++) {
      sim_model_lib0_B->idx_data_c[khi] = (real_T)sim_model_lib0_B->
        indx_data_g[khi];
    }

    if (indx_sizes > 0) {
      // shuffle the points avaliable
      sim_model_lib0_randperm_i((real_T)indx_sizes,
        sim_model_lib0_B->random_order_data, random_order_sizes,
        sim_model_lib0_B, sim_model_lib0_DW);
      shuffled_ids_sizes = random_order_sizes[1];
      loop_ub = random_order_sizes[1];
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        sim_model_lib0_B->shuffled_ids_data[khi] = (int32_T)
          sim_model_lib0_B->idx_data_c[(int32_T)((int32_T)
          sim_model_lib0_B->random_order_data[(int32_T)(random_order_sizes[0] *
          khi)] - 1)];
      }

      // determine if num_points_avail>num_points needed, then verify its <num_points_out to make simulink happy 
      if (num_points_needed <= (real_T)indx_sizes) {
        num_points_to_use = num_points_needed;
      } else {
        num_points_to_use = (real_T)indx_sizes;
      }

      // capture the points
      indx_sizes = (int32_T)num_points_to_use;
      loop_ub = (int32_T)((int32_T)num_points_to_use - 1);
      for (khi = 0; khi <= loop_ub; khi++) {
        d_data[khi] = (int8_T)khi;
      }

      loop_ub = (int32_T)num_points_to_use;
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        tmp_data_8[khi] = (int8_T)(int32_T)(1 + khi);
      }

      for (khi = 0; khi <= (int32_T)(indx_sizes - 1); khi++) {
        new_ids_data[(int32_T)d_data[khi]] = sim_model_lib0_B->valid_ids_data
          [(int32_T)(sim_model_lib0_B->shuffled_ids_data[(int32_T)((int32_T)
          tmp_data_8[khi] - 1)] - 1)];
      }

      for (khi = 0; khi <= (int32_T)(shuffled_ids_sizes - 1); khi++) {
        sim_model_lib0_B->valid_points_in_cam_data_f[khi] =
          sim_model_lib0_B->valid_points_in_cam_data[(int32_T)
          (sim_model_lib0_B->shuffled_ids_data[khi] - 1)];
      }

      for (khi = 0; khi <= (int32_T)(shuffled_ids_sizes - 1); khi++) {
        sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi +
          shuffled_ids_sizes)] = sim_model_lib0_B->valid_points_in_cam_data
          [(int32_T)((int32_T)(sim_model_lib0_B->shuffled_ids_data[khi] +
          valid_points_in_cam_sizes[0]) - 1)];
      }

      valid_points_in_cam_sizes_0[0] = (int32_T)num_points_to_use;
      valid_points_in_cam_sizes_0[1] = 2;
      loop_ub = (int32_T)num_points_to_use;
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        valid_points_in_cam_data[khi] =
          sim_model_lib0_B->valid_points_in_cam_data_f[khi];
      }

      loop_ub = (int32_T)num_points_to_use;
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        valid_points_in_cam_data[(int32_T)(khi + valid_points_in_cam_sizes_0[0])]
          = sim_model_lib0_B->valid_points_in_cam_data_f[(int32_T)(khi +
          shuffled_ids_sizes)];
      }

      tmp_sizes_0[0] = (int32_T)num_points_to_use;
      tmp_sizes_0[1] = 2;
      tmp_sizes_0[2] = 15;
      loop_ub = (int32_T)((int32_T)((int32_T)num_points_to_use << 1) * 15);
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        sim_model_lib0_B->tmp_data_l[khi] = 0.0F;
      }

      mglncbaaiekflfcb_cat(valid_points_in_cam_data, valid_points_in_cam_sizes_0,
                           tmp_sizes_0, sim_model_lib0_B->tmp_data_d,
                           tmp_sizes_3);
      for (khi = 0; khi < 16; khi++) {
        loop_ub = tmp_sizes_3[0];
        for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++)
        {
          sim_model_lib0_B->new_observations_data[(int32_T)(indx_sizes +
            (int32_T)((int32_T)(r_sizes << 1) * khi))] =
            sim_model_lib0_B->tmp_data_d[(int32_T)((int32_T)((int32_T)
            (tmp_sizes_3[0] * tmp_sizes_3[1]) * khi) + indx_sizes)];
        }

        loop_ub = tmp_sizes_3[0];
        for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++)
        {
          sim_model_lib0_B->new_observations_data[(int32_T)((int32_T)(indx_sizes
            + r_sizes) + (int32_T)((int32_T)(r_sizes << 1) * khi))] =
            sim_model_lib0_B->tmp_data_d[(int32_T)((int32_T)((int32_T)((int32_T)
            (tmp_sizes_3[0] * tmp_sizes_3[1]) * khi) + indx_sizes) +
            tmp_sizes_3[0])];
        }
      }

      loop_ub = (int32_T)num_points_to_use;
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        tmp_data[khi] = 1U;
      }

      tmp_sizes[0] = (int32_T)num_points_to_use;
      tmp_sizes[1] = 15;
      loop_ub = (int32_T)((int32_T)num_points_to_use * 15);
      for (khi = 0; khi <= (int32_T)(loop_ub - 1); khi++) {
        tmp_data_0[khi] = 0U;
      }

      fkfkbiecjecjbaie_cat((int32_T)num_points_to_use, tmp_sizes, valid_out_h,
                           random_order_sizes);
      for (khi = 0; khi < 16; khi++) {
        loop_ub = random_order_sizes[0];
        for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++)
        {
          new_valid_flags_data[(int32_T)(indx_sizes + (int32_T)
            (new_valid_flags_sizes_idx_0 * khi))] = valid_out_h[(int32_T)
            ((int32_T)(random_order_sizes[0] * khi) + indx_sizes)];
        }
      }
    }

    nb = 0;
    for (khi = 0; khi < 50; khi++) {
      if (replace_these_points[khi]) {
        sim_model_lib0_DW->int_id_hist[khi] = new_ids_data[nb];
        nb++;
      }
    }

    nb = 0;
    for (khi = 0; khi < 50; khi++) {
      if (replace_these_points[khi]) {
        ia_data[nb] = (int32_T)(khi + 1);
        nb++;
      }
    }

    for (khi = 0; khi < 16; khi++) {
      loop_ub = (int32_T)num_points_needed;
      for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++) {
        sim_model_lib0_DW->int_observations[(int32_T)((int32_T)
          (ia_data[indx_sizes] + (int32_T)(100 * khi)) - 1)] =
          sim_model_lib0_B->new_observations_data[(int32_T)((int32_T)((int32_T)
          (r_sizes << 1) * khi) + indx_sizes)];
      }

      loop_ub = (int32_T)num_points_needed;
      for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++) {
        sim_model_lib0_DW->int_observations[(int32_T)((int32_T)
          (ia_data[indx_sizes] + (int32_T)(100 * khi)) + 49)] =
          sim_model_lib0_B->new_observations_data[(int32_T)((int32_T)((int32_T)
          ((int32_T)(r_sizes << 1) * khi) + indx_sizes) + r_sizes)];
      }
    }

    nb = 0;
    for (khi = 0; khi < 50; khi++) {
      if (replace_these_points[khi]) {
        ia_data[nb] = (int32_T)(khi + 1);
        nb++;
      }
    }

    for (khi = 0; khi < 16; khi++) {
      loop_ub = (int32_T)num_points_needed;
      for (indx_sizes = 0; indx_sizes <= (int32_T)(loop_ub - 1); indx_sizes++) {
        sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(ia_data[indx_sizes]
          + (int32_T)(50 * khi)) - 1)] = new_valid_flags_data[(int32_T)((int32_T)
          (new_valid_flags_sizes_idx_0 * khi) + indx_sizes)];
      }
    }
  }

  // sort the outputs by ID number
  hdjmknohknopimgl_sort(sim_model_lib0_DW->int_id_hist, ia_data);
  for (khi = 0; khi < 16; khi++) {
    for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
      for (shuffled_ids_sizes = 0; shuffled_ids_sizes < 50; shuffled_ids_sizes++)
      {
        sim_model_lib0_B->tmp_data_d[(int32_T)((int32_T)(shuffled_ids_sizes +
          (int32_T)(50 * indx_sizes)) + (int32_T)(100 * khi))] =
          sim_model_lib0_DW->int_observations[(int32_T)((int32_T)((int32_T)
          ((int32_T)(50 * indx_sizes) + ia_data[shuffled_ids_sizes]) + (int32_T)
          (100 * khi)) - 1)];
      }
    }
  }

  for (khi = 0; khi < 16; khi++) {
    for (indx_sizes = 0; indx_sizes < 2; indx_sizes++) {
      memcpy(&sim_model_lib0_DW->int_observations[(int32_T)((int32_T)(khi * 100)
              + (int32_T)(indx_sizes * 50))], &sim_model_lib0_B->tmp_data_d
             [(int32_T)((int32_T)(khi * 100) + (int32_T)(indx_sizes * 50))],
             (uint32_T)(50U * sizeof(real32_T)));
    }

    for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
      valid_out_h[(int32_T)(indx_sizes + (int32_T)(50 * khi))] =
        sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)((int32_T)(50 * khi)
        + ia_data[indx_sizes]) - 1)];
    }
  }

  for (khi = 0; khi < 16; khi++) {
    for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
      sim_model_lib0_DW->int_valid_flag[(int32_T)(indx_sizes + (int32_T)(50 *
        khi))] = valid_out_h[(int32_T)((int32_T)(50 * khi) + indx_sizes)];
    }
  }

  memset(&valid_out_h[0], 0, (uint32_T)(800U * sizeof(uint8_T)));

  // Determine which points to report
  if (i != 0) {
    // if we just replaced the last augmentation
    for (khi = 0; khi < 4; khi++) {
      for (indx_sizes = 0; indx_sizes < 50; indx_sizes++) {
        valid_out_h[(int32_T)(indx_sizes + (int32_T)(50 * (int32_T)l[khi]))] =
          sim_model_lib0_DW->int_valid_flag[(int32_T)((int32_T)(50 * (int32_T)
          l[khi]) + indx_sizes)];
      }
    }
  }

  // DataTypeConversion: '<S160>/Data Type Conversion4' incorporates:
  //   Constant: '<S160>/Constant3'
  //   Logic: '<S160>/Logical Operator'
  //   Selector: '<S160>/select_current_command'
  //   UnitDelay: '<S160>/Unit Delay'

  // else send all zeros
  // populate outputs
  // '<S162>:1:30'
  // '<S162>:1:31'
  // landmarks NOT populated/used by optical flow
  // '<S162>:1:32'
  // cam points not populated by OF
  // end of function
  for (khi = 0; khi < 800; khi++) {
    valid_out_h[khi] = (uint8_T)(((int32_T)valid_out_h[khi] != 0) &&
      sim_model_lib0_P->cvs_optflow_valid_mask[(int32_T)((int32_T)
      sim_model_lib0_DW->UnitDelay_DSTATE_it - 1)]);
  }

  // End of DataTypeConversion: '<S160>/Data Type Conversion4'

  // Sum: '<S160>/Sum1' incorporates:
  //   Constant: '<S160>/Constant1'
  //   Constant: '<S163>/Constant'
  //   Logic: '<S160>/Logical Operator1'
  //   Logic: '<S160>/Logical Operator2'
  //   Logic: '<S160>/Logical Operator3'
  //   RelationalOperator: '<S160>/Relational Operator'
  //   RelationalOperator: '<S160>/Relational Operator1'
  //   RelationalOperator: '<S160>/Relational Operator2'
  //   RelationalOperator: '<S163>/Compare'
  //   Selector: '<S160>/Select_col_1'
  //   Selector: '<S160>/Select_col_2'

  khi = (int32_T)(uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->cvs_optflow_valid_times[0] !=
      sim_model_lib0_P->Constant_Value_mi) &&
     ((sim_model_lib0_P->cvs_optflow_valid_times[0] <
       sim_model_lib0_B->RateTransition4_f.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_optflow_valid_times[0] ==
        sim_model_lib0_B->RateTransition4_f.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_f.timestamp_nsec >=
        sim_model_lib0_P->cvs_optflow_valid_times[3])))) + (uint32_T)
    ((sim_model_lib0_P->cvs_optflow_valid_times[1] !=
      sim_model_lib0_P->Constant_Value_mi) &&
     ((sim_model_lib0_P->cvs_optflow_valid_times[1] <
       sim_model_lib0_B->RateTransition4_f.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_optflow_valid_times[1] ==
        sim_model_lib0_B->RateTransition4_f.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_f.timestamp_nsec >=
        sim_model_lib0_P->cvs_optflow_valid_times[4]))))) + (uint32_T)
    ((sim_model_lib0_P->cvs_optflow_valid_times[2] !=
      sim_model_lib0_P->Constant_Value_mi) &&
     ((sim_model_lib0_P->cvs_optflow_valid_times[2] <
       sim_model_lib0_B->RateTransition4_f.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_optflow_valid_times[2] ==
        sim_model_lib0_B->RateTransition4_f.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_f.timestamp_nsec >=
        sim_model_lib0_P->cvs_optflow_valid_times[5])))));

  // Update for RateTransition: '<S83>/Rate Transition10' incorporates:
  //   MATLAB Function: '<S83>/generate_output'

  sim_model_lib0_DW->RateTransition10_Buffer0 =
    sim_model_lib0_DW->int_registration_number;

  // Update for RateTransition: '<S83>/Rate Transition3'
  sim_model_lib0_DW->RateTransition3_Buffer0_g =
    sim_model_lib0_B->RateTransition4_f.timestamp_sec;

  // Update for RateTransition: '<S83>/Rate Transition1'
  sim_model_lib0_DW->RateTransition1_Buffer0_b =
    sim_model_lib0_B->RateTransition4_f.timestamp_nsec;

  // Update for RateTransition: '<S83>/Rate Transition6' incorporates:
  //   MATLAB Function: '<S83>/generate_output'

  memcpy(&sim_model_lib0_DW->RateTransition6_Buffer0[0],
         &sim_model_lib0_DW->int_observations[0], (uint32_T)(1600U * sizeof
          (real32_T)));

  // Update for RateTransition: '<S83>/Rate Transition7'
  memcpy(&sim_model_lib0_DW->RateTransition7_Buffer0_a[0], &valid_out_h[0],
         (uint32_T)(800U * sizeof(uint8_T)));

  // Update for RateTransition: '<S83>/Rate Transition8' incorporates:
  //   MATLAB Function: '<S83>/generate_output'

  memcpy(&sim_model_lib0_DW->RateTransition8_Buffer0[0],
         &sim_model_lib0_DW->int_id_hist[0], (uint32_T)(50U * sizeof(real32_T)));

  // Update for RandomNumber: '<S161>/pixel_noise'
  rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_o);

  // Saturate: '<S160>/Saturation' incorporates:
  //   Sum: '<S160>/Sum1'

  if ((uint8_T)khi > sim_model_lib0_P->Saturation_UpperSat_p) {
    // Update for UnitDelay: '<S160>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_it =
      sim_model_lib0_P->Saturation_UpperSat_p;
  } else if ((uint8_T)khi < sim_model_lib0_P->Saturation_LowerSat_h) {
    // Update for UnitDelay: '<S160>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_it =
      sim_model_lib0_P->Saturation_LowerSat_h;
  } else {
    // Update for UnitDelay: '<S160>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_it = (uint8_T)khi;
  }

  // End of Saturate: '<S160>/Saturation'
}

// Model step function for TID4
void sim_model_lib0_step4(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M) // Sample time: [0.336s, 0.0s] 
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);
  real32_T rtb_Conversion_oh[9];
  real32_T rtb_Subtract1[3];
  real32_T rtb_MatrixConcatenate[16];
  real32_T rtb_Product[4];
  real32_T rtb_Sum;
  real32_T rtb_Assignment[9];
  real32_T rtb_Sum_o;
  int8_T rtb_DataTypeConversion4[50];
  real32_T valid_out[50];
  int32_T i;
  int32_T i_0;
  boolean_T tmp;
  real32_T tmp_0[9];
  real32_T rtb_Assignment_p[9];
  real32_T tmp_1[9];
  real32_T rtb_Product_m[9];
  real32_T tmp_2;
  real32_T tmp_3;
  real32_T rtb_Product_p;

  // DataTypeConversion: '<S146>/Conversion' incorporates:
  //   Constant: '<S145>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Conversion_oh[i] = (real32_T)sim_model_lib0_P->Constant2_Value_j[i];
  }

  // End of DataTypeConversion: '<S146>/Conversion'

  // Assignment: '<S145>/Assignment' incorporates:
  //   Constant: '<S140>/Constant3'

  rtb_Conversion_oh[0] = sim_model_lib0_P->tun_abp_q_body2navcam[3];
  rtb_Conversion_oh[4] = sim_model_lib0_P->tun_abp_q_body2navcam[3];
  rtb_Conversion_oh[8] = sim_model_lib0_P->tun_abp_q_body2navcam[3];

  // Sum: '<S145>/Sum2' incorporates:
  //   Constant: '<S140>/Constant3'
  //   Constant: '<S147>/Constant3'
  //   DataTypeConversion: '<S148>/Conversion'
  //   Gain: '<S147>/Gain'
  //   Gain: '<S147>/Gain1'
  //   Gain: '<S147>/Gain2'

  rtb_Product_m[0] = (real32_T)sim_model_lib0_P->Constant3_Value_hp;
  rtb_Product_m[1] = sim_model_lib0_P->tun_abp_q_body2navcam[2];
  rtb_Product_m[2] = sim_model_lib0_P->Gain_Gain_kc *
    sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_Product_m[3] = sim_model_lib0_P->Gain1_Gain_pi *
    sim_model_lib0_P->tun_abp_q_body2navcam[2];
  rtb_Product_m[4] = (real32_T)sim_model_lib0_P->Constant3_Value_hp;
  rtb_Product_m[5] = sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_Product_m[6] = sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_Product_m[7] = sim_model_lib0_P->Gain2_Gain_pj *
    sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_Product_m[8] = (real32_T)sim_model_lib0_P->Constant3_Value_hp;

  // Concatenate: '<S145>/Matrix Concatenate' incorporates:
  //   Constant: '<S140>/Constant3'
  //   Gain: '<S145>/Gain1'
  //   Sum: '<S145>/Sum2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_MatrixConcatenate[(int32_T)(i_0 << 2)] = rtb_Conversion_oh[(int32_T)(3 *
      i_0)] + rtb_Product_m[(int32_T)(3 * i_0)];
    rtb_MatrixConcatenate[(int32_T)(1 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_oh[(int32_T)((int32_T)(3 * i_0) + 1)] + rtb_Product_m
      [(int32_T)((int32_T)(3 * i_0) + 1)];
    rtb_MatrixConcatenate[(int32_T)(2 + (int32_T)(i_0 << 2))] =
      rtb_Conversion_oh[(int32_T)((int32_T)(3 * i_0) + 2)] + rtb_Product_m
      [(int32_T)((int32_T)(3 * i_0) + 2)];
  }

  rtb_MatrixConcatenate[3] = sim_model_lib0_P->Gain1_Gain_hr *
    sim_model_lib0_P->tun_abp_q_body2navcam[0];
  rtb_MatrixConcatenate[7] = sim_model_lib0_P->Gain1_Gain_hr *
    sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_MatrixConcatenate[11] = sim_model_lib0_P->Gain1_Gain_hr *
    sim_model_lib0_P->tun_abp_q_body2navcam[2];

  // End of Concatenate: '<S145>/Matrix Concatenate'

  // Switch: '<S140>/Switch' incorporates:
  //   Constant: '<S140>/Constant'

  tmp = ((int32_T)sim_model_lib0_P->tun_cvs_noise_on != 0);

  // Reshape: '<S142>/Reshape1' incorporates:
  //   Constant: '<S140>/Constant3'

  rtb_MatrixConcatenate[12] = sim_model_lib0_P->tun_abp_q_body2navcam[0];

  // Product: '<S142>/Product' incorporates:
  //   Constant: '<S140>/Constant6'
  //   Constant: '<S140>/Constant7'
  //   Switch: '<S140>/Switch'

  if (tmp) {
    rtb_Sum = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[0];
    rtb_Sum_o = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[1];
  } else {
    rtb_Sum = sim_model_lib0_P->Constant7_Value_b[0];
    rtb_Sum_o = sim_model_lib0_P->Constant7_Value_b[1];
  }

  // Reshape: '<S142>/Reshape1' incorporates:
  //   Constant: '<S140>/Constant3'

  rtb_MatrixConcatenate[13] = sim_model_lib0_P->tun_abp_q_body2navcam[1];
  rtb_MatrixConcatenate[14] = sim_model_lib0_P->tun_abp_q_body2navcam[2];

  // Product: '<S142>/Product' incorporates:
  //   Constant: '<S140>/Constant6'
  //   Constant: '<S140>/Constant7'
  //   Switch: '<S140>/Switch'

  if (tmp) {
    tmp_2 = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[2];
    tmp_3 = sim_model_lib0_P->cvs_navcam_Q_B2navcan_error[3];
  } else {
    tmp_2 = sim_model_lib0_P->Constant7_Value_b[2];
    tmp_3 = sim_model_lib0_P->Constant7_Value_b[3];
  }

  // Reshape: '<S142>/Reshape1' incorporates:
  //   Constant: '<S140>/Constant3'

  rtb_MatrixConcatenate[15] = sim_model_lib0_P->tun_abp_q_body2navcam[3];

  // Product: '<S142>/Product'
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_Product_p = rtb_MatrixConcatenate[(int32_T)(i_0 + 12)] * tmp_3 +
      (rtb_MatrixConcatenate[(int32_T)(i_0 + 8)] * tmp_2 +
       (rtb_MatrixConcatenate[(int32_T)(i_0 + 4)] * rtb_Sum_o +
        rtb_MatrixConcatenate[i_0] * rtb_Sum));
    rtb_Product[i_0] = rtb_Product_p;
  }

  // Sum: '<S144>/Sum' incorporates:
  //   Constant: '<S144>/Constant1'
  //   DataTypeConversion: '<S154>/Conversion'
  //   Gain: '<S144>/Gain'
  //   Math: '<S144>/Math Function'

  rtb_Sum = rtb_Product[3] * rtb_Product[3] * sim_model_lib0_P->Gain_Gain_cy -
    (real32_T)sim_model_lib0_P->Constant1_Value_f;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S144>/Assignment' incorporates:
    //   Constant: '<S144>/Constant2'
    //   DataTypeConversion: '<S153>/Conversion'

    rtb_Conversion_oh[i] = (real32_T)sim_model_lib0_P->Constant2_Value_id[i];

    // Assignment: '<S143>/Assignment' incorporates:
    //   Constant: '<S143>/Constant2'
    //   DataTypeConversion: '<S149>/Conversion'

    rtb_Assignment[i] = (real32_T)sim_model_lib0_P->Constant2_Value_k[i];
  }

  // Assignment: '<S144>/Assignment'
  rtb_Conversion_oh[0] = rtb_Sum;
  rtb_Conversion_oh[4] = rtb_Sum;
  rtb_Conversion_oh[8] = rtb_Sum;

  // Gain: '<S144>/Gain1'
  rtb_Sum = sim_model_lib0_P->Gain1_Gain_l0 * rtb_Product[3];

  // Sum: '<S143>/Sum' incorporates:
  //   Constant: '<S143>/Constant1'
  //   DataTypeConversion: '<S150>/Conversion'
  //   Gain: '<S143>/Gain'
  //   Math: '<S143>/Math Function'

  rtb_Sum_o = sim_model_lib0_B->RateTransition2_j.Q_ISS2B[3] *
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[3] *
    sim_model_lib0_P->Gain_Gain_bvs - (real32_T)
    sim_model_lib0_P->Constant1_Value_l;

  // Assignment: '<S143>/Assignment'
  rtb_Assignment[0] = rtb_Sum_o;
  rtb_Assignment[4] = rtb_Sum_o;
  rtb_Assignment[8] = rtb_Sum_o;

  // Gain: '<S143>/Gain1'
  rtb_Sum_o = sim_model_lib0_P->Gain1_Gain_hd *
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[3];

  // Sum: '<S140>/Add' incorporates:
  //   Constant: '<S137>/Constant3'
  //   Constant: '<S140>/Constant4'
  //   Gain: '<S140>/Gain'

  for (i = 0; i < 25896; i++) {
    sim_model_lib0_B->Add_o[i] = (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_landmark_map_error[i] +
      sim_model_lib0_P->cvs_landmark_map_iss[i];
  }

  // End of Sum: '<S140>/Add'
  for (i_0 = 0; i_0 < 8632; i_0++) {
    // Sum: '<S140>/Subtract' incorporates:
    //   Selector: '<S140>/Select_X'

    sim_model_lib0_B->P_point_B_iss_p4[i_0] = sim_model_lib0_B->Add_o[i_0] -
      sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[0];

    // Sum: '<S140>/Subtract2' incorporates:
    //   Selector: '<S140>/Select_Y'

    sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(8632 + i_0)] =
      sim_model_lib0_B->Add_o[(int32_T)(8632 + i_0)] -
      sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[1];

    // Sum: '<S140>/Subtract3' incorporates:
    //   Selector: '<S140>/Select_Z'

    sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(17264 + i_0)] =
      sim_model_lib0_B->Add_o[(int32_T)(17264 + i_0)] -
      sim_model_lib0_B->RateTransition2_j.P_B_ISS_ISS[2];
  }

  // Product: '<S143>/Product' incorporates:
  //   Constant: '<S151>/Constant3'
  //   DataTypeConversion: '<S152>/Conversion'
  //   Gain: '<S151>/Gain'
  //   Gain: '<S151>/Gain1'
  //   Gain: '<S151>/Gain2'

  tmp_0[0] = (real32_T)sim_model_lib0_P->Constant3_Value_p2;
  tmp_0[1] = sim_model_lib0_B->RateTransition2_j.Q_ISS2B[2];
  tmp_0[2] = sim_model_lib0_P->Gain_Gain_jw *
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[1];
  tmp_0[3] = sim_model_lib0_P->Gain1_Gain_om *
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[2];
  tmp_0[4] = (real32_T)sim_model_lib0_P->Constant3_Value_p2;
  tmp_0[5] = sim_model_lib0_B->RateTransition2_j.Q_ISS2B[0];
  tmp_0[6] = sim_model_lib0_B->RateTransition2_j.Q_ISS2B[1];
  tmp_0[7] = sim_model_lib0_P->Gain2_Gain_pf *
    sim_model_lib0_B->RateTransition2_j.Q_ISS2B[0];
  tmp_0[8] = (real32_T)sim_model_lib0_P->Constant3_Value_p2;

  // Product: '<S143>/Product1' incorporates:
  //   Gain: '<S143>/Gain2'
  //   Math: '<S143>/Math Function1'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_m[i_0] = sim_model_lib0_B->RateTransition2_j.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_j.Q_ISS2B[0];
    rtb_Product_m[(int32_T)(i_0 + 3)] =
      sim_model_lib0_B->RateTransition2_j.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_j.Q_ISS2B[1];
    rtb_Product_m[(int32_T)(i_0 + 6)] =
      sim_model_lib0_B->RateTransition2_j.Q_ISS2B[i_0] *
      sim_model_lib0_B->RateTransition2_j.Q_ISS2B[2];
  }

  // End of Product: '<S143>/Product1'

  // Sum: '<S143>/Sum1' incorporates:
  //   Gain: '<S143>/Gain2'
  //   Product: '<S140>/Product'
  //   Product: '<S143>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_p[(int32_T)(3 * i_0)] = (rtb_Assignment[(int32_T)(3 * i_0)] -
      tmp_0[(int32_T)(3 * i_0)] * rtb_Sum_o) + rtb_Product_m[(int32_T)(3 * i_0)]
      * sim_model_lib0_P->Gain2_Gain_n;
    rtb_Assignment_p[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_0[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum_o) + rtb_Product_m[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_n;
    rtb_Assignment_p[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Assignment
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_0[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum_o) + rtb_Product_m[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_n;
  }

  // End of Sum: '<S143>/Sum1'

  // Product: '<S144>/Product' incorporates:
  //   Constant: '<S155>/Constant3'
  //   DataTypeConversion: '<S156>/Conversion'
  //   Gain: '<S155>/Gain'
  //   Gain: '<S155>/Gain1'
  //   Gain: '<S155>/Gain2'

  tmp_1[0] = (real32_T)sim_model_lib0_P->Constant3_Value_o;
  tmp_1[1] = rtb_Product[2];
  tmp_1[2] = sim_model_lib0_P->Gain_Gain_bv * rtb_Product[1];
  tmp_1[3] = sim_model_lib0_P->Gain1_Gain_m5 * rtb_Product[2];
  tmp_1[4] = (real32_T)sim_model_lib0_P->Constant3_Value_o;
  tmp_1[5] = rtb_Product[0];
  tmp_1[6] = rtb_Product[1];
  tmp_1[7] = sim_model_lib0_P->Gain2_Gain_l1 * rtb_Product[0];
  tmp_1[8] = (real32_T)sim_model_lib0_P->Constant3_Value_o;
  for (i = 0; i < 3; i++) {
    // Product: '<S140>/Product' incorporates:
    //   Math: '<S140>/Math Function'

    for (i_0 = 0; i_0 < 8632; i_0++) {
      sim_model_lib0_B->Add_o[(int32_T)(i + (int32_T)(3 * i_0))] = 0.0F;
      sim_model_lib0_B->Add_o[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_p[i] * sim_model_lib0_B->P_point_B_iss_p4[i_0];
      sim_model_lib0_B->Add_o[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_p[(int32_T)(i + 3)] * sim_model_lib0_B->P_point_B_iss_p4
        [(int32_T)(i_0 + 8632)];
      sim_model_lib0_B->Add_o[(int32_T)(i + (int32_T)(3 * i_0))] +=
        rtb_Assignment_p[(int32_T)(i + 6)] * sim_model_lib0_B->P_point_B_iss_p4
        [(int32_T)(i_0 + 17264)];
    }

    // Sum: '<S140>/Subtract1' incorporates:
    //   Constant: '<S140>/Constant1'
    //   Constant: '<S140>/Constant2'
    //   Gain: '<S140>/Gain1'

    rtb_Subtract1[i] = sim_model_lib0_P->tun_abp_p_navcam_body_body_sim[i] -
      (real32_T)sim_model_lib0_P->tun_cvs_noise_on *
      sim_model_lib0_P->cvs_navcam_P_B_B_error[i];

    // Product: '<S144>/Product1' incorporates:
    //   Gain: '<S144>/Gain2'
    //   Math: '<S144>/Math Function1'

    rtb_Product_m[i] = rtb_Product[i] * rtb_Product[0];
    rtb_Product_m[(int32_T)(i + 3)] = rtb_Product[i] * rtb_Product[1];
    rtb_Product_m[(int32_T)(i + 6)] = rtb_Product[i] * rtb_Product[2];
  }

  // Sum: '<S144>/Sum1' incorporates:
  //   Gain: '<S144>/Gain2'
  //   Product: '<S140>/Product1'
  //   Product: '<S144>/Product'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Assignment_p[(int32_T)(3 * i_0)] = (rtb_Conversion_oh[(int32_T)(3 * i_0)]
      - tmp_1[(int32_T)(3 * i_0)] * rtb_Sum) + rtb_Product_m[(int32_T)(3 * i_0)]
      * sim_model_lib0_P->Gain2_Gain_hs;
    rtb_Assignment_p[(int32_T)(1 + (int32_T)(3 * i_0))] = (rtb_Conversion_oh
      [(int32_T)((int32_T)(3 * i_0) + 1)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      1)] * rtb_Sum) + rtb_Product_m[(int32_T)((int32_T)(3 * i_0) + 1)] *
      sim_model_lib0_P->Gain2_Gain_hs;
    rtb_Assignment_p[(int32_T)(2 + (int32_T)(3 * i_0))] = (rtb_Conversion_oh
      [(int32_T)((int32_T)(3 * i_0) + 2)] - tmp_1[(int32_T)((int32_T)(3 * i_0) +
      2)] * rtb_Sum) + rtb_Product_m[(int32_T)((int32_T)(3 * i_0) + 2)] *
      sim_model_lib0_P->Gain2_Gain_hs;
  }

  // End of Sum: '<S144>/Sum1'
  for (i_0 = 0; i_0 < 8632; i_0++) {
    // Concatenate: '<S140>/Matrix Concatenate1' incorporates:
    //   Product: '<S140>/Product1'
    //   Selector: '<S140>/Select_X1'
    //   Selector: '<S140>/Select_Y1'
    //   Selector: '<S140>/Select_Z1'
    //   Sum: '<S140>/Subtract4'
    //   Sum: '<S140>/Subtract5'
    //   Sum: '<S140>/Subtract6'

    sim_model_lib0_B->rtb_Add_o_m[(int32_T)(3 * i_0)] = sim_model_lib0_B->Add_o
      [(int32_T)(3 * i_0)] - rtb_Subtract1[0];
    sim_model_lib0_B->rtb_Add_o_m[(int32_T)(1 + (int32_T)(3 * i_0))] =
      sim_model_lib0_B->Add_o[(int32_T)((int32_T)(3 * i_0) + 1)] -
      rtb_Subtract1[1];
    sim_model_lib0_B->rtb_Add_o_m[(int32_T)(2 + (int32_T)(3 * i_0))] =
      sim_model_lib0_B->Add_o[(int32_T)((int32_T)(3 * i_0) + 2)] -
      rtb_Subtract1[2];

    // Math: '<S140>/Math Function1' incorporates:
    //   Product: '<S140>/Product1'

    for (i = 0; i < 3; i++) {
      sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(i_0 + (int32_T)(8632 * i))] =
        0.0F;
      sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(i_0 + (int32_T)(8632 * i))] +=
        sim_model_lib0_B->rtb_Add_o_m[(int32_T)(3 * i_0)] * rtb_Assignment_p[i];
      sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(i_0 + (int32_T)(8632 * i))] +=
        sim_model_lib0_B->rtb_Add_o_m[(int32_T)((int32_T)(3 * i_0) + 1)] *
        rtb_Assignment_p[(int32_T)(i + 3)];
      sim_model_lib0_B->P_point_B_iss_p4[(int32_T)(i_0 + (int32_T)(8632 * i))] +=
        sim_model_lib0_B->rtb_Add_o_m[(int32_T)((int32_T)(3 * i_0) + 2)] *
        rtb_Assignment_p[(int32_T)(i + 6)];
    }

    // End of Math: '<S140>/Math Function1'
  }

  // Outputs for Iterator SubSystem: '<S137>/pinhole_projection_model'
  si_pinhole_projection_model(8632, sim_model_lib0_B->P_point_B_iss_p4,
    sim_model_lib0_B->ImpAsg_InsertedFor_P_points_2_f,
    sim_model_lib0_B->ImpAsg_InsertedFor_points_in__p,
    sim_model_lib0_DW->pinhole_projection_model_id,
    (P_pinhole_projection_model_si_T *)
    &sim_model_lib0_P->pinhole_projection_model_id,
    sim_model_lib0_P->tun_cvs_noise_on,
    sim_model_lib0_P->tun_cvs_navcam_focal_length_Y,
    sim_model_lib0_P->tun_cvs_navcam_focal_length_X,
    sim_model_lib0_P->landmark_image_processing_pixel,
    sim_model_lib0_P->tun_cvs_navcam_num_pixels_X,
    sim_model_lib0_P->tun_cvs_navcam_num_pixels_Y,
    sim_model_lib0_P->cvs_navcam_min_dist, sim_model_lib0_P->cvs_navcam_max_dist,
    sim_model_lib0_P->cvs_navcam_pointing);

  // End of Outputs for SubSystem: '<S137>/pinhole_projection_model'

  // MATLAB Function: '<S82>/generate_output' incorporates:
  //   Constant: '<S137>/Constant3'

  // MATLAB Function 'image_processing/generate_output': '<S138>:1'
  //  Inputs:
  //    Process type:                                                                [Parameter] 
  //        1=Landmarks, 2=AR Tags, 3=Optical Flow, 4=Handrail
  //    num_pts_out         = Required number of points to be reported               [Parameter] 
  //    P_point_iss_iss     = ALL [x,y,z] position of points in the ISS frame        **Landmark only** 
  //    P_point_cam_cam     = ALL [x,y,z] position of points in the CAM frame        **Handrail only** 
  //    P_points_2D_cam     = ALL [u,v] pixel location of points in the camera       **Landmark and OF only** 
  //    valid_flag          = indicates which points are valid
  //    num_hist_kept       = Required number of histories kept for Optical Flow     [Parameter] **OF only** 
  //    flow_ids            = ID tag for each optical flow point                     [Parameter] **OF only** 
  //
  //  Outputs:
  //    points_out_iss      = output [x,y,z] position of points in the ISS frame     **This output unused by optical flow** 
  //    points_out_2D_cam   = output [u,v] pixel location of points in the camera    **This output unused by handrail** 
  //    points_out_cam      = output [x,y,z] position of points in the CAM frame     **This output unused by OF and landmarks** 
  //    valid_out           = indicates which values of the output are valid!        **This output unused by landmarks** 
  //    ids_out             = optical flow IDs
  //    registration_id     = visual odometery registration pulse ID                 **ONLY used by OF** 
  // default to landmarks
  // '<S138>:1:39'
  sim_mode_format_landmark_output(sim_model_lib0_P->cvs_landmark_map_iss,
    sim_model_lib0_B->ImpAsg_InsertedFor_P_points_2_f,
    sim_model_lib0_B->ImpAsg_InsertedFor_points_in__p,
    sim_model_lib0_B->points_out_iss_j, sim_model_lib0_B->points_out_2D_cam_g,
    valid_out, sim_model_lib0_B, sim_model_lib0_DW);

  // DataTypeConversion: '<S136>/Data Type Conversion4' incorporates:
  //   Constant: '<S136>/Constant3'
  //   Logic: '<S136>/Logical Operator'
  //   Selector: '<S136>/select_current_command'
  //   UnitDelay: '<S136>/Unit Delay'

  // '<S138>:1:39'
  // '<S138>:1:40'
  // ID tag only populated/used in optical flow
  // '<S138>:1:41'
  // cam points not populated by landmarks
  // '<S138>:1:42'
  // end of function
  for (i = 0; i < 50; i++) {
    rtb_DataTypeConversion4[i] = (int8_T)((valid_out[i] != 0.0F) &&
      sim_model_lib0_P->cvs_landmark_valid_mask[(int32_T)((int32_T)
      sim_model_lib0_DW->UnitDelay_DSTATE_a - 1)]);
  }

  // End of DataTypeConversion: '<S136>/Data Type Conversion4'

  // Sum: '<S136>/Sum1' incorporates:
  //   Constant: '<S136>/Constant1'
  //   Constant: '<S139>/Constant'
  //   Logic: '<S136>/Logical Operator1'
  //   Logic: '<S136>/Logical Operator2'
  //   Logic: '<S136>/Logical Operator3'
  //   RelationalOperator: '<S136>/Relational Operator'
  //   RelationalOperator: '<S136>/Relational Operator1'
  //   RelationalOperator: '<S136>/Relational Operator2'
  //   RelationalOperator: '<S139>/Compare'
  //   Selector: '<S136>/Select_col_1'
  //   Selector: '<S136>/Select_col_2'

  i_0 = (int32_T)(uint32_T)((uint32_T)((uint32_T)
    ((sim_model_lib0_P->cvs_landmark_valid_times[0] !=
      sim_model_lib0_P->Constant_Value_eq) &&
     ((sim_model_lib0_P->cvs_landmark_valid_times[0] <
       sim_model_lib0_B->RateTransition4_e.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_landmark_valid_times[0] ==
        sim_model_lib0_B->RateTransition4_e.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_e.timestamp_nsec >=
        sim_model_lib0_P->cvs_landmark_valid_times[3])))) + (uint32_T)
    ((sim_model_lib0_P->cvs_landmark_valid_times[1] !=
      sim_model_lib0_P->Constant_Value_eq) &&
     ((sim_model_lib0_P->cvs_landmark_valid_times[1] <
       sim_model_lib0_B->RateTransition4_e.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_landmark_valid_times[1] ==
        sim_model_lib0_B->RateTransition4_e.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_e.timestamp_nsec >=
        sim_model_lib0_P->cvs_landmark_valid_times[4]))))) + (uint32_T)
    ((sim_model_lib0_P->cvs_landmark_valid_times[2] !=
      sim_model_lib0_P->Constant_Value_eq) &&
     ((sim_model_lib0_P->cvs_landmark_valid_times[2] <
       sim_model_lib0_B->RateTransition4_e.timestamp_sec) ||
      ((sim_model_lib0_P->cvs_landmark_valid_times[2] ==
        sim_model_lib0_B->RateTransition4_e.timestamp_sec) &&
       (sim_model_lib0_B->RateTransition4_e.timestamp_nsec >=
        sim_model_lib0_P->cvs_landmark_valid_times[5])))));

  // Update for RateTransition: '<S82>/Rate Transition3'
  sim_model_lib0_DW->RateTransition3_Buffer0_o =
    sim_model_lib0_B->RateTransition4_e.timestamp_sec;

  // Update for RateTransition: '<S82>/Rate Transition1'
  sim_model_lib0_DW->RateTransition1_Buffer0_l =
    sim_model_lib0_B->RateTransition4_e.timestamp_nsec;

  // Update for RateTransition: '<S82>/Rate Transition5'
  memcpy(&sim_model_lib0_DW->RateTransition5_Buffer0_i[0],
         &sim_model_lib0_B->points_out_iss_j[0], (uint32_T)(150U * sizeof
          (real32_T)));

  // Update for RateTransition: '<S82>/Rate Transition6'
  memcpy(&sim_model_lib0_DW->RateTransition6_Buffer0_h[0],
         &sim_model_lib0_B->points_out_2D_cam_g[0], (uint32_T)(100U * sizeof
          (real32_T)));

  // Update for RateTransition: '<S82>/Rate Transition7'
  for (i = 0; i < 50; i++) {
    sim_model_lib0_DW->RateTransition7_Buffer0_an[i] = (uint8_T)
      rtb_DataTypeConversion4[i];
  }

  // End of Update for RateTransition: '<S82>/Rate Transition7'

  // Update for RandomNumber: '<S137>/pixel_noise'
  rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_mi);

  // Saturate: '<S136>/Saturation' incorporates:
  //   Sum: '<S136>/Sum1'

  if ((uint8_T)i_0 > sim_model_lib0_P->Saturation_UpperSat_k) {
    // Update for UnitDelay: '<S136>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_a =
      sim_model_lib0_P->Saturation_UpperSat_k;
  } else if ((uint8_T)i_0 < sim_model_lib0_P->Saturation_LowerSat_a) {
    // Update for UnitDelay: '<S136>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_a =
      sim_model_lib0_P->Saturation_LowerSat_a;
  } else {
    // Update for UnitDelay: '<S136>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_a = (uint8_T)i_0;
  }

  // End of Saturate: '<S136>/Saturation'
}

// Model initialize function
void sim_model_lib0_initialize(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M,
  act_msg *sim_model_lib0_U_act_msg_l, cmc_msg *sim_model_lib0_U_cmc_msg_in,
  cvs_optical_flow_msg *sim_model_lib0_Y_cvs_optical_flow_msg_n,
  cvs_handrail_msg *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg
  *sim_model_lib0_Y_cmc_msg_c, imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg
  *sim_model_lib0_Y_env_msg_i, bpm_msg *sim_model_lib0_Y_bpm_msg_h,
  cvs_registration_pulse *sim_model_lib0_Y_cvs_registration_pulse_d,
  cvs_landmark_msg *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
  *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg *sim_model_lib0_Y_ex_time_msg_m)
{
  P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
    sim_model_lib0_M->defaultParam);
  B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
    sim_model_lib0_M->blockIO);
  DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
    sim_model_lib0_M->dwork);

  {
    uint32_T tseed;
    uint32_T r;
    real_T y1;
    int32_T i;
    real32_T Constant3_idx_2;
    real32_T Constant12_idx_2;
    real32_T Constant11_idx_2;
    real32_T Constant3_idx_1;
    real32_T Constant12_idx_1;
    real32_T Constant11_idx_1;
    real32_T Constant3_idx_0;
    real32_T Constant12_idx_0;
    real32_T Constant11_idx_0;

    // Start for RateTransition: '<S81>/Rate Transition3'
    sim_model_lib0_B->RateTransition3 = sim_model_lib0_P->RateTransition3_X0;

    // Start for RateTransition: '<S81>/Rate Transition1'
    sim_model_lib0_B->RateTransition1 = sim_model_lib0_P->RateTransition1_X0;
    for (i = 0; i < 150; i++) {
      // Start for RateTransition: '<S81>/Rate Transition5'
      sim_model_lib0_B->RateTransition5[i] =
        sim_model_lib0_P->RateTransition5_X0;

      // Start for RateTransition: '<S81>/Rate Transition9'
      sim_model_lib0_B->RateTransition9[i] =
        sim_model_lib0_P->RateTransition9_X0;
    }

    // Start for RateTransition: '<S81>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7[i] =
        sim_model_lib0_P->RateTransition7_X0;
    }

    // End of Start for RateTransition: '<S81>/Rate Transition7'

    // Start for Constant: '<S7>/Constant11'
    Constant11_idx_0 = sim_model_lib0_P->tun_ini_P_B_ISS_ISS[0];

    // Start for Constant: '<S7>/Constant12'
    Constant12_idx_0 = sim_model_lib0_P->tun_ini_V_B_ISS_ISS[0];

    // Start for Constant: '<S5>/Constant3'
    Constant3_idx_0 = sim_model_lib0_P->tun_ini_omega_B_ISS_B[0];

    // Start for Constant: '<S7>/Constant11'
    Constant11_idx_1 = sim_model_lib0_P->tun_ini_P_B_ISS_ISS[1];

    // Start for Constant: '<S7>/Constant12'
    Constant12_idx_1 = sim_model_lib0_P->tun_ini_V_B_ISS_ISS[1];

    // Start for Constant: '<S5>/Constant3'
    Constant3_idx_1 = sim_model_lib0_P->tun_ini_omega_B_ISS_B[1];

    // Start for Constant: '<S7>/Constant11'
    Constant11_idx_2 = sim_model_lib0_P->tun_ini_P_B_ISS_ISS[2];

    // Start for Constant: '<S7>/Constant12'
    Constant12_idx_2 = sim_model_lib0_P->tun_ini_V_B_ISS_ISS[2];

    // Start for Constant: '<S5>/Constant3'
    Constant3_idx_2 = sim_model_lib0_P->tun_ini_omega_B_ISS_B[2];

    // Start for RateTransition: '<S83>/Rate Transition10'
    sim_model_lib0_B->RateTransition10 = sim_model_lib0_P->RateTransition10_X0;

    // Start for RateTransition: '<S83>/Rate Transition3'
    sim_model_lib0_B->RateTransition3_j = sim_model_lib0_P->RateTransition3_X0_o;

    // Start for RateTransition: '<S83>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_j = sim_model_lib0_P->RateTransition1_X0_e;

    // Start for RateTransition: '<S83>/Rate Transition6'
    for (i = 0; i < 1600; i++) {
      sim_model_lib0_B->RateTransition6[i] =
        sim_model_lib0_P->RateTransition6_X0;
    }

    // End of Start for RateTransition: '<S83>/Rate Transition6'

    // Start for RateTransition: '<S83>/Rate Transition7'
    for (i = 0; i < 800; i++) {
      sim_model_lib0_B->RateTransition7_f[i] =
        sim_model_lib0_P->RateTransition7_X0_m;
    }

    // End of Start for RateTransition: '<S83>/Rate Transition7'

    // Start for RateTransition: '<S83>/Rate Transition8'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition8[i] =
        sim_model_lib0_P->RateTransition8_X0;
    }

    // End of Start for RateTransition: '<S83>/Rate Transition8'

    // Start for RateTransition: '<S82>/Rate Transition3'
    sim_model_lib0_B->RateTransition3_o = sim_model_lib0_P->RateTransition3_X0_m;

    // Start for RateTransition: '<S82>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_a =
      sim_model_lib0_P->RateTransition1_X0_eo;

    // Start for RateTransition: '<S82>/Rate Transition5'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_B->RateTransition5_b[i] =
        sim_model_lib0_P->RateTransition5_X0_c;
    }

    // End of Start for RateTransition: '<S82>/Rate Transition5'

    // Start for RateTransition: '<S82>/Rate Transition6'
    for (i = 0; i < 100; i++) {
      sim_model_lib0_B->RateTransition6_n[i] =
        sim_model_lib0_P->RateTransition6_X0_j;
    }

    // End of Start for RateTransition: '<S82>/Rate Transition6'

    // Start for RateTransition: '<S82>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7_g[i] =
        sim_model_lib0_P->RateTransition7_X0_k;
    }

    // End of Start for RateTransition: '<S82>/Rate Transition7'

    // Start for RateTransition: '<S79>/Rate Transition3'
    sim_model_lib0_B->RateTransition3_e = sim_model_lib0_P->RateTransition3_X0_c;

    // Start for RateTransition: '<S79>/Rate Transition1'
    sim_model_lib0_B->RateTransition1_b = sim_model_lib0_P->RateTransition1_X0_a;

    // Start for RateTransition: '<S79>/Rate Transition5'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_B->RateTransition5_p[i] =
        sim_model_lib0_P->RateTransition5_X0_o;
    }

    // End of Start for RateTransition: '<S79>/Rate Transition5'

    // Start for RateTransition: '<S79>/Rate Transition6'
    for (i = 0; i < 100; i++) {
      sim_model_lib0_B->RateTransition6_p[i] =
        sim_model_lib0_P->RateTransition6_X0_g;
    }

    // End of Start for RateTransition: '<S79>/Rate Transition6'

    // Start for RateTransition: '<S79>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_B->RateTransition7_k[i] =
        sim_model_lib0_P->RateTransition7_X0_n;
    }

    // End of Start for RateTransition: '<S79>/Rate Transition7'

    // Start for Iterator SubSystem: '<S85>/pinhole_projection_model'
    pinhole_projection_mo_Start(36, sim_model_lib0_DW->pinhole_projection_model);

    // End of Start for SubSystem: '<S85>/pinhole_projection_model'

    // Start for Iterator SubSystem: '<S113>/pinhole_projection_model'
    pinhole_projection_mo_Start(504,
      sim_model_lib0_DW->pinhole_projection_model_i);

    // End of Start for SubSystem: '<S113>/pinhole_projection_model'

    // Start for Iterator SubSystem: '<S137>/pinhole_projection_model'
    pinhole_projection_mo_Start(8632,
      sim_model_lib0_DW->pinhole_projection_model_id);

    // End of Start for SubSystem: '<S137>/pinhole_projection_model'

    // Start for Iterator SubSystem: '<S161>/pinhole_projection_model'
    pinhole_projection_mo_Start(3979,
      sim_model_lib0_DW->pinhole_projection_model_o);

    // End of Start for SubSystem: '<S161>/pinhole_projection_model'

    // InitializeConditions for RateTransition: '<S81>/Rate Transition3'
    sim_model_lib0_DW->RateTransition3_Buffer0 =
      sim_model_lib0_P->RateTransition3_X0;

    // InitializeConditions for RateTransition: '<S81>/Rate Transition1'
    sim_model_lib0_DW->RateTransition1_Buffer0 =
      sim_model_lib0_P->RateTransition1_X0;

    // InitializeConditions for RateTransition: '<S81>/Rate Transition5'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_DW->RateTransition5_Buffer0[i] =
        sim_model_lib0_P->RateTransition5_X0;
    }

    // End of InitializeConditions for RateTransition: '<S81>/Rate Transition5'

    // InitializeConditions for RateTransition: '<S81>/Rate Transition9'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_DW->RateTransition9_Buffer0[i] =
        sim_model_lib0_P->RateTransition9_X0;
    }

    // End of InitializeConditions for RateTransition: '<S81>/Rate Transition9'

    // InitializeConditions for RateTransition: '<S81>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_DW->RateTransition7_Buffer0[i] =
        sim_model_lib0_P->RateTransition7_X0;
    }

    // End of InitializeConditions for RateTransition: '<S81>/Rate Transition7'

    // InitializeConditions for UnitDelay: '<S70>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_i =
      sim_model_lib0_P->UnitDelay_InitialCondition;

    // InitializeConditions for UnitDelay: '<S71>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_if =
      sim_model_lib0_P->UnitDelay_InitialCondition_c;

    // InitializeConditions for UnitDelay: '<S73>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_h =
      sim_model_lib0_P->UnitDelay_InitialCondition_n;

    // InitializeConditions for UnitDelay: '<S72>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_ht =
      sim_model_lib0_P->UnitDelay_InitialCondition_l;

    // InitializeConditions for UnitDelay: '<S8>/Unit Delay1'
    sim_model_lib0_DW->UnitDelay1_DSTATE = sim_model_lib0_P->ini_time_seconds;

    // InitializeConditions for UnitDelay: '<S8>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE = sim_model_lib0_P->ini_time_nanoseconds;

    // InitializeConditions for Delay: '<S11>/Delay'
    sim_model_lib0_DW->icLoad = 1U;

    // InitializeConditions for Delay: '<S1>/Delay1'
    sim_model_lib0_DW->Delay1_DSTATE = sim_model_lib0_P->Delay1_InitialCondition;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[0] = Constant11_idx_0;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[0] = Constant12_idx_0;

    // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[0] = Constant3_idx_0;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[0] =
      sim_model_lib0_P->DiscreteTimeIntegrator_IC;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[0] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_b;

    // InitializeConditions for RandomNumber: '<S254>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[0]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev +
      sim_model_lib0_P->random_noise_Mean;
    sim_model_lib0_DW->NextOutput[0] = y1;
    sim_model_lib0_DW->RandSeed[0] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[0] = (real_T)
      sim_model_lib0_P->epson_accel_bias_ic[0];

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[1] = Constant11_idx_1;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[1] = Constant12_idx_1;

    // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[1] = Constant3_idx_1;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[1] =
      sim_model_lib0_P->DiscreteTimeIntegrator_IC;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[1] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_b;

    // InitializeConditions for RandomNumber: '<S254>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[1]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev +
      sim_model_lib0_P->random_noise_Mean;
    sim_model_lib0_DW->NextOutput[1] = y1;
    sim_model_lib0_DW->RandSeed[1] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[1] = (real_T)
      sim_model_lib0_P->epson_accel_bias_ic[1];

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_p[2] = Constant11_idx_2;

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE[2] = Constant12_idx_2;

    // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_b[2] = Constant3_idx_2;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_d[2] =
      sim_model_lib0_P->DiscreteTimeIntegrator_IC;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_b[2] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_b;

    // InitializeConditions for RandomNumber: '<S254>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[2]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev +
      sim_model_lib0_P->random_noise_Mean;
    sim_model_lib0_DW->NextOutput[2] = y1;
    sim_model_lib0_DW->RandSeed[2] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTATE[2] = (real_T)
      sim_model_lib0_P->epson_accel_bias_ic[2];

    // InitializeConditions for DiscreteIntegrator: '<S254>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC;

    // InitializeConditions for RandomNumber: '<S255>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[0]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev_c +
      sim_model_lib0_P->random_noise_Mean_k;
    sim_model_lib0_DW->NextOutput_e[0] = y1;
    sim_model_lib0_DW->RandSeed_k[0] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[0] =
      sim_model_lib0_P->epson_gyro_bias_ic[0];

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[0] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_k;

    // InitializeConditions for RandomNumber: '<S255>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[1]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev_c +
      sim_model_lib0_P->random_noise_Mean_k;
    sim_model_lib0_DW->NextOutput_e[1] = y1;
    sim_model_lib0_DW->RandSeed_k[1] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[1] =
      sim_model_lib0_P->epson_gyro_bias_ic[1];

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[1] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_k;

    // InitializeConditions for RandomNumber: '<S255>/random_noise'
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[2]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise_StdDev_c +
      sim_model_lib0_P->random_noise_Mean_k;
    sim_model_lib0_DW->NextOutput_e[2] = y1;
    sim_model_lib0_DW->RandSeed_k[2] = tseed;

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator2' 
    sim_model_lib0_DW->DiscreteTimeIntegrator2_DSTAT_j[2] =
      sim_model_lib0_P->epson_gyro_bias_ic[2];

    // InitializeConditions for DiscreteIntegrator: '<S255>/Discrete-Time Integrator1' 
    sim_model_lib0_DW->DiscreteTimeIntegrator1_DSTAT_c[2] =
      sim_model_lib0_P->DiscreteTimeIntegrator1_IC_k;

    // InitializeConditions for DiscreteIntegrator: '<S191>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTAT_dp =
      sim_model_lib0_P->bpm_impeller_init_speed;

    // InitializeConditions for DiscreteIntegrator: '<S224>/Discrete-Time Integrator' 
    sim_model_lib0_DW->DiscreteTimeIntegrator_DSTATE_e =
      sim_model_lib0_P->bpm_impeller_init_speed;

    // InitializeConditions for UnitDelay: '<S188>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_m =
      sim_model_lib0_P->UnitDelay_InitialCondition_o;

    // InitializeConditions for UnitDelay: '<S221>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_c =
      sim_model_lib0_P->UnitDelay_InitialCondition_gb;

    // InitializeConditions for DiscretePulseGenerator: '<S80>/AR_pulse_gen'
    sim_model_lib0_DW->clockTickCounter = 0;

    // InitializeConditions for DiscretePulseGenerator: '<S80>/landmark_pulse_gen' 
    sim_model_lib0_DW->clockTickCounter_c = 0;

    // InitializeConditions for RateTransition: '<S83>/Rate Transition10'
    sim_model_lib0_DW->RateTransition10_Buffer0 =
      sim_model_lib0_P->RateTransition10_X0;

    // InitializeConditions for RateTransition: '<S83>/Rate Transition3'
    sim_model_lib0_DW->RateTransition3_Buffer0_g =
      sim_model_lib0_P->RateTransition3_X0_o;

    // InitializeConditions for RateTransition: '<S83>/Rate Transition1'
    sim_model_lib0_DW->RateTransition1_Buffer0_b =
      sim_model_lib0_P->RateTransition1_X0_e;

    // InitializeConditions for RateTransition: '<S83>/Rate Transition6'
    for (i = 0; i < 1600; i++) {
      sim_model_lib0_DW->RateTransition6_Buffer0[i] =
        sim_model_lib0_P->RateTransition6_X0;
    }

    // End of InitializeConditions for RateTransition: '<S83>/Rate Transition6'

    // InitializeConditions for RateTransition: '<S83>/Rate Transition7'
    for (i = 0; i < 800; i++) {
      sim_model_lib0_DW->RateTransition7_Buffer0_a[i] =
        sim_model_lib0_P->RateTransition7_X0_m;
    }

    // End of InitializeConditions for RateTransition: '<S83>/Rate Transition7'

    // InitializeConditions for RateTransition: '<S83>/Rate Transition8'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_DW->RateTransition8_Buffer0[i] =
        sim_model_lib0_P->RateTransition8_X0;
    }

    // End of InitializeConditions for RateTransition: '<S83>/Rate Transition8'

    // InitializeConditions for UnitDelay: '<S110>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE_p =
      sim_model_lib0_P->DetectChange_vinit_j;

    // InitializeConditions for UnitDelay: '<S111>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE_l =
      sim_model_lib0_P->DetectChange1_vinit;

    // InitializeConditions for RateTransition: '<S82>/Rate Transition3'
    sim_model_lib0_DW->RateTransition3_Buffer0_o =
      sim_model_lib0_P->RateTransition3_X0_m;

    // InitializeConditions for RateTransition: '<S82>/Rate Transition1'
    sim_model_lib0_DW->RateTransition1_Buffer0_l =
      sim_model_lib0_P->RateTransition1_X0_eo;

    // InitializeConditions for RateTransition: '<S82>/Rate Transition5'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_DW->RateTransition5_Buffer0_i[i] =
        sim_model_lib0_P->RateTransition5_X0_c;
    }

    // End of InitializeConditions for RateTransition: '<S82>/Rate Transition5'

    // InitializeConditions for RateTransition: '<S82>/Rate Transition6'
    for (i = 0; i < 100; i++) {
      sim_model_lib0_DW->RateTransition6_Buffer0_h[i] =
        sim_model_lib0_P->RateTransition6_X0_j;
    }

    // End of InitializeConditions for RateTransition: '<S82>/Rate Transition6'

    // InitializeConditions for RateTransition: '<S82>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_DW->RateTransition7_Buffer0_an[i] =
        sim_model_lib0_P->RateTransition7_X0_k;
    }

    // End of InitializeConditions for RateTransition: '<S82>/Rate Transition7'

    // InitializeConditions for RateTransition: '<S79>/Rate Transition3'
    sim_model_lib0_DW->RateTransition3_Buffer0_n =
      sim_model_lib0_P->RateTransition3_X0_c;

    // InitializeConditions for RateTransition: '<S79>/Rate Transition1'
    sim_model_lib0_DW->RateTransition1_Buffer0_i =
      sim_model_lib0_P->RateTransition1_X0_a;

    // InitializeConditions for RateTransition: '<S79>/Rate Transition5'
    for (i = 0; i < 150; i++) {
      sim_model_lib0_DW->RateTransition5_Buffer0_f[i] =
        sim_model_lib0_P->RateTransition5_X0_o;
    }

    // End of InitializeConditions for RateTransition: '<S79>/Rate Transition5'

    // InitializeConditions for RateTransition: '<S79>/Rate Transition6'
    for (i = 0; i < 100; i++) {
      sim_model_lib0_DW->RateTransition6_Buffer0_e[i] =
        sim_model_lib0_P->RateTransition6_X0_g;
    }

    // End of InitializeConditions for RateTransition: '<S79>/Rate Transition6'

    // InitializeConditions for RateTransition: '<S79>/Rate Transition7'
    for (i = 0; i < 50; i++) {
      sim_model_lib0_DW->RateTransition7_Buffer0_j[i] =
        sim_model_lib0_P->RateTransition7_X0_n;
    }

    // End of InitializeConditions for RateTransition: '<S79>/Rate Transition7'

    // InitializeConditions for DiscretePulseGenerator: '<S80>/handrail_pulse_gen' 
    sim_model_lib0_DW->clockTickCounter_n = 0;

    // InitializeConditions for RandomNumber: '<S6>/Random Number'
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[0]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_vel_variance[0]) +
      sim_model_lib0_P->RandomNumber_Mean;
    sim_model_lib0_DW->NextOutput_l[0] = y1;
    sim_model_lib0_DW->RandSeed_i[0] = tseed;
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[1]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_vel_variance[1]) +
      sim_model_lib0_P->RandomNumber_Mean;
    sim_model_lib0_DW->NextOutput_l[1] = y1;
    sim_model_lib0_DW->RandSeed_i[1] = tseed;
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[2]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_vel_variance[2]) +
      sim_model_lib0_P->RandomNumber_Mean;
    sim_model_lib0_DW->NextOutput_l[2] = y1;
    sim_model_lib0_DW->RandSeed_i[2] = tseed;

    // End of InitializeConditions for RandomNumber: '<S6>/Random Number'

    // InitializeConditions for RandomNumber: '<S6>/Random Number1'
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[0]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_omega_variance[0]) +
      sim_model_lib0_P->RandomNumber1_Mean;
    sim_model_lib0_DW->NextOutput_f[0] = y1;
    sim_model_lib0_DW->RandSeed_m[0] = tseed;
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[1]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_omega_variance[1]) +
      sim_model_lib0_P->RandomNumber1_Mean;
    sim_model_lib0_DW->NextOutput_f[1] = y1;
    sim_model_lib0_DW->RandSeed_m[1] = tseed;
    y1 = floor(sim_model_lib0_P->env_ext_air_vel_seed[2]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * sqrt
      (sim_model_lib0_P->env_ext_air_omega_variance[2]) +
      sim_model_lib0_P->RandomNumber1_Mean;
    sim_model_lib0_DW->NextOutput_f[2] = y1;
    sim_model_lib0_DW->RandSeed_m[2] = tseed;

    // End of InitializeConditions for RandomNumber: '<S6>/Random Number1'

    // InitializeConditions for RandomNumber: '<S85>/pixel_noise'
    y1 = floor(sim_model_lib0_P->ARtag_image_processing_handrail);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    sim_model_lib0_DW->RandSeed_mb = tseed;
    rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_mb);

    // End of InitializeConditions for RandomNumber: '<S85>/pixel_noise'

    // InitializeConditions for UnitDelay: '<S84>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_hj =
      sim_model_lib0_P->UnitDelay_InitialCondition_d;

    // InitializeConditions for RandomNumber: '<S113>/pixel_noise'
    y1 = floor(sim_model_lib0_P->cvs_noise_seed * 8.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    sim_model_lib0_DW->RandSeed_d = tseed;
    sim_model_lib0_DW->NextOutput_o = rt_nrand_Upu32_Yd_f_pw_snf
      (&sim_model_lib0_DW->RandSeed_d) * sqrt
      (sim_model_lib0_P->cvs_handrail_noise_var) +
      sim_model_lib0_P->pixel_noise_Mean_a;

    // End of InitializeConditions for RandomNumber: '<S113>/pixel_noise'

    // InitializeConditions for UnitDelay: '<S112>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_g =
      sim_model_lib0_P->UnitDelay_InitialCondition_m;

    // InitializeConditions for RandomNumber: '<S137>/pixel_noise'
    y1 = floor(sim_model_lib0_P->landmark_image_processing_handr);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    sim_model_lib0_DW->RandSeed_mi = tseed;
    rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_mi);

    // End of InitializeConditions for RandomNumber: '<S137>/pixel_noise'

    // InitializeConditions for UnitDelay: '<S136>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_a =
      sim_model_lib0_P->UnitDelay_InitialCondition_g;

    // InitializeConditions for RandomNumber: '<S161>/pixel_noise'
    y1 = floor(sim_model_lib0_P->optical_flow_image_processing_h);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    sim_model_lib0_DW->RandSeed_o = tseed;
    rt_nrand_Upu32_Yd_f_pw_snf(&sim_model_lib0_DW->RandSeed_o);

    // End of InitializeConditions for RandomNumber: '<S161>/pixel_noise'

    // InitializeConditions for UnitDelay: '<S160>/Unit Delay'
    sim_model_lib0_DW->UnitDelay_DSTATE_it =
      sim_model_lib0_P->UnitDelay_InitialCondition_a;

    // InitializeConditions for UnitDelay: '<S194>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE[0] =
      sim_model_lib0_P->DetectChange_vinit;

    // InitializeConditions for UnitDelay: '<S227>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE_m[0] =
      sim_model_lib0_P->DetectChange_vinit_o;

    // InitializeConditions for RandomNumber: '<S254>/random_noise1'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[0] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev +
      sim_model_lib0_P->random_noise1_Mean;
    sim_model_lib0_DW->NextOutput_i[0] = y1;
    sim_model_lib0_DW->RandSeed_ip[0] = tseed;

    // InitializeConditions for UnitDelay: '<S194>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE[1] =
      sim_model_lib0_P->DetectChange_vinit;

    // InitializeConditions for UnitDelay: '<S227>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE_m[1] =
      sim_model_lib0_P->DetectChange_vinit_o;

    // InitializeConditions for RandomNumber: '<S254>/random_noise1'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[1] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev +
      sim_model_lib0_P->random_noise1_Mean;
    sim_model_lib0_DW->NextOutput_i[1] = y1;
    sim_model_lib0_DW->RandSeed_ip[1] = tseed;

    // InitializeConditions for UnitDelay: '<S194>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE[2] =
      sim_model_lib0_P->DetectChange_vinit;

    // InitializeConditions for UnitDelay: '<S227>/Delay Input1'
    sim_model_lib0_DW->DelayInput1_DSTATE_m[2] =
      sim_model_lib0_P->DetectChange_vinit_o;

    // InitializeConditions for RandomNumber: '<S254>/random_noise1'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[2] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev +
      sim_model_lib0_P->random_noise1_Mean;
    sim_model_lib0_DW->NextOutput_i[2] = y1;
    sim_model_lib0_DW->RandSeed_ip[2] = tseed;

    // InitializeConditions for RandomNumber: '<S254>/random_noise2'
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[0] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev +
      sim_model_lib0_P->random_noise2_Mean;
    sim_model_lib0_DW->NextOutput_j[0] = y1;
    sim_model_lib0_DW->RandSeed_e[0] = tseed;
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[1] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev +
      sim_model_lib0_P->random_noise2_Mean;
    sim_model_lib0_DW->NextOutput_j[1] = y1;
    sim_model_lib0_DW->RandSeed_e[1] = tseed;
    y1 = floor(sim_model_lib0_P->epson_accel_noise_seed[2] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev +
      sim_model_lib0_P->random_noise2_Mean;
    sim_model_lib0_DW->NextOutput_j[2] = y1;
    sim_model_lib0_DW->RandSeed_e[2] = tseed;

    // End of InitializeConditions for RandomNumber: '<S254>/random_noise2'

    // InitializeConditions for RandomNumber: '<S255>/random_noise1'
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[0] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev_l +
      sim_model_lib0_P->random_noise1_Mean_d;
    sim_model_lib0_DW->NextOutput_h[0] = y1;
    sim_model_lib0_DW->RandSeed_p[0] = tseed;
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[1] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev_l +
      sim_model_lib0_P->random_noise1_Mean_d;
    sim_model_lib0_DW->NextOutput_h[1] = y1;
    sim_model_lib0_DW->RandSeed_p[1] = tseed;
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[2] + 1.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise1_StdDev_l +
      sim_model_lib0_P->random_noise1_Mean_d;
    sim_model_lib0_DW->NextOutput_h[2] = y1;
    sim_model_lib0_DW->RandSeed_p[2] = tseed;

    // End of InitializeConditions for RandomNumber: '<S255>/random_noise1'

    // InitializeConditions for RandomNumber: '<S255>/random_noise2'
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[0] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev_k +
      sim_model_lib0_P->random_noise2_Mean_f;
    sim_model_lib0_DW->NextOutput_p[0] = y1;
    sim_model_lib0_DW->RandSeed_n[0] = tseed;
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[1] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev_k +
      sim_model_lib0_P->random_noise2_Mean_f;
    sim_model_lib0_DW->NextOutput_p[1] = y1;
    sim_model_lib0_DW->RandSeed_n[1] = tseed;
    y1 = floor(sim_model_lib0_P->epson_gyro_noise_seed[2] + 2.0);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (uint32_T)(tseed >> 16U);
    i = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)(r << 16U)) + (uint32_T)i) << 16U) + (uint32_T)i) + r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) *
      sim_model_lib0_P->random_noise2_StdDev_k +
      sim_model_lib0_P->random_noise2_Mean_f;
    sim_model_lib0_DW->NextOutput_p[2] = y1;
    sim_model_lib0_DW->RandSeed_n[2] = tseed;

    // End of InitializeConditions for RandomNumber: '<S255>/random_noise2'

    // SystemInitialize for Atomic SubSystem: '<S190>/speed_controller'
    sim_m_speed_controller_Init(&sim_model_lib0_DW->speed_controller,
      (P_speed_controller_sim_model__T *)&sim_model_lib0_P->speed_controller);

    // End of SystemInitialize for SubSystem: '<S190>/speed_controller'

    // SystemInitialize for Atomic SubSystem: '<S223>/speed_controller'
    sim_m_speed_controller_Init(&sim_model_lib0_DW->speed_controller_c,
      (P_speed_controller_sim_model__T *)&sim_model_lib0_P->speed_controller_c);

    // End of SystemInitialize for SubSystem: '<S223>/speed_controller'

    // SystemInitialize for Atomic SubSystem: '<S184>/servo_model'
    sim_model__servo_model_Init(&sim_model_lib0_DW->servo_model,
      (P_servo_model_sim_model_lib0_T *)&sim_model_lib0_P->servo_model,
      (real32_T)sim_model_lib0_P->bpm_servo_min_theta,
      sim_model_lib0_P->bpm_servo_motor_gear_ratio);

    // End of SystemInitialize for SubSystem: '<S184>/servo_model'

    // SystemInitialize for Atomic SubSystem: '<S185>/servo_model'
    sim_model__servo_model_Init(&sim_model_lib0_DW->servo_model_f,
      (P_servo_model_sim_model_lib0_T *)&sim_model_lib0_P->servo_model_f,
      (real32_T)sim_model_lib0_P->bpm_servo_min_theta,
      sim_model_lib0_P->bpm_servo_motor_gear_ratio);

    // End of SystemInitialize for SubSystem: '<S185>/servo_model'

    // SystemInitialize for Enabled SubSystem: '<S188>/latch_nozzle_thrust_matricies' 
    latch_nozzle_thrust_ma_Init(&sim_model_lib0_B->latch_nozzle_thrust_matricies,
      (P_latch_nozzle_thrust_matrici_T *)
      &sim_model_lib0_P->latch_nozzle_thrust_matricies);

    // End of SystemInitialize for SubSystem: '<S188>/latch_nozzle_thrust_matricies' 

    // SystemInitialize for Atomic SubSystem: '<S184>/blower_aerodynamics'
    si_blower_aerodynamics_Init(&sim_model_lib0_DW->blower_aerodynamics,
      (P_blower_aerodynamics_sim_mod_T *)&sim_model_lib0_P->blower_aerodynamics,
      sim_model_lib0_P->bpm_PM1_randn_noise_seed);

    // End of SystemInitialize for SubSystem: '<S184>/blower_aerodynamics'

    // SystemInitialize for Enabled SubSystem: '<S221>/latch_nozzle_thrust_matricies' 
    latch_nozzle_thrust_ma_Init
      (&sim_model_lib0_B->latch_nozzle_thrust_matricies_p,
       (P_latch_nozzle_thrust_matrici_T *)
       &sim_model_lib0_P->latch_nozzle_thrust_matricies_p);

    // End of SystemInitialize for SubSystem: '<S221>/latch_nozzle_thrust_matricies' 

    // SystemInitialize for Atomic SubSystem: '<S185>/blower_aerodynamics'
    blower_aerodynamics_d_Init(&sim_model_lib0_DW->blower_aerodynamics_j,
      (P_blower_aerodynamics_sim_m_m_T *)
      &sim_model_lib0_P->blower_aerodynamics_j,
      sim_model_lib0_P->bpm_PM2_randn_noise_seed);

    // End of SystemInitialize for SubSystem: '<S185>/blower_aerodynamics'

    // SystemInitialize for Iterator SubSystem: '<S85>/pinhole_projection_model' 
    pinhole_projection_mod_Init(36, sim_model_lib0_DW->pinhole_projection_model,
      (P_pinhole_projection_model_si_T *)
      &sim_model_lib0_P->pinhole_projection_model,
      sim_model_lib0_P->cvs_AR_pixel_noise, sim_model_lib0_P->cvs_noise_seed *
      4.0);

    // End of SystemInitialize for SubSystem: '<S85>/pinhole_projection_model'

    // SystemInitialize for Iterator SubSystem: '<S113>/pinhole_projection_model' 
    pinhole_projection_mod_Init(504,
      sim_model_lib0_DW->pinhole_projection_model_i,
      (P_pinhole_projection_model_si_T *)
      &sim_model_lib0_P->pinhole_projection_model_i,
      sim_model_lib0_P->handrail_image_processing_pixel,
      sim_model_lib0_P->cvs_noise_seed * 7.0);

    // End of SystemInitialize for SubSystem: '<S113>/pinhole_projection_model'

    // SystemInitialize for MATLAB Function: '<S81>/generate_output'
    memset(&sim_model_lib0_DW->state_m[0], 0, (uint32_T)(625U * sizeof(uint32_T)));
    r = 5489U;
    sim_model_lib0_DW->state_m[0] = 5489U;
    for (i = 0; i < 623; i++) {
      r = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(r >> 30U) ^ r) *
        1812433253U) + (uint32_T)i) + 1U);
      sim_model_lib0_DW->state_m[(int32_T)(i + 1)] = r;
    }

    sim_model_lib0_DW->state_m[624] = 624U;

    // End of SystemInitialize for MATLAB Function: '<S81>/generate_output'

    // SystemInitialize for Iterator SubSystem: '<S137>/pinhole_projection_model' 
    pinhole_projection_mod_Init(8632,
      sim_model_lib0_DW->pinhole_projection_model_id,
      (P_pinhole_projection_model_si_T *)
      &sim_model_lib0_P->pinhole_projection_model_id,
      sim_model_lib0_P->landmark_image_processing_pixel,
      sim_model_lib0_P->cvs_noise_seed * 3.0);

    // End of SystemInitialize for SubSystem: '<S137>/pinhole_projection_model'

    // SystemInitialize for MATLAB Function: '<S82>/generate_output'
    memset(&sim_model_lib0_DW->state_d[0], 0, (uint32_T)(625U * sizeof(uint32_T)));
    r = 5489U;
    sim_model_lib0_DW->state_d[0] = 5489U;
    for (i = 0; i < 623; i++) {
      r = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(r >> 30U) ^ r) *
        1812433253U) + (uint32_T)i) + 1U);
      sim_model_lib0_DW->state_d[(int32_T)(i + 1)] = r;
    }

    sim_model_lib0_DW->state_d[624] = 624U;

    // End of SystemInitialize for MATLAB Function: '<S82>/generate_output'

    // SystemInitialize for Iterator SubSystem: '<S161>/pinhole_projection_model' 
    pinhole_projection_mod_Init(3979,
      sim_model_lib0_DW->pinhole_projection_model_o,
      (P_pinhole_projection_model_si_T *)
      &sim_model_lib0_P->pinhole_projection_model_o,
      sim_model_lib0_P->optical_flow_image_processing_p,
      sim_model_lib0_P->cvs_noise_seed * 5.0);

    // End of SystemInitialize for SubSystem: '<S161>/pinhole_projection_model'

    // SystemInitialize for MATLAB Function: '<S83>/generate_output'
    sim_model_lib0_DW->int_id_hist_not_empty = false;
    memset(&sim_model_lib0_DW->int_id_hist[0], 0, (uint32_T)(50U * sizeof
            (real32_T)));
    memset(&sim_model_lib0_DW->state[0], 0, (uint32_T)(625U * sizeof(uint32_T)));
    r = 5489U;
    sim_model_lib0_DW->state[0] = 5489U;
    for (i = 0; i < 623; i++) {
      r = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(r >> 30U) ^ r) *
        1812433253U) + (uint32_T)i) + 1U);
      sim_model_lib0_DW->state[(int32_T)(i + 1)] = r;
    }

    sim_model_lib0_DW->state[624] = 624U;

    // End of SystemInitialize for MATLAB Function: '<S83>/generate_output'
  }
}

// Model terminate function
void sim_model_lib0_terminate(RT_MODEL_sim_model_lib0_T * sim_model_lib0_M)
{
  // model code
  rt_FREE(sim_model_lib0_M->blockIO);
  if (sim_model_lib0_M->paramIsMalloced) {
    rt_FREE(sim_model_lib0_M->defaultParam);
  }

  rt_FREE(sim_model_lib0_M->dwork);
  rt_FREE(sim_model_lib0_M);
}

// Model data allocation function
RT_MODEL_sim_model_lib0_T *sim_model_lib0(act_msg *sim_model_lib0_U_act_msg_l,
  cmc_msg *sim_model_lib0_U_cmc_msg_in, cvs_optical_flow_msg
  *sim_model_lib0_Y_cvs_optical_flow_msg_n, cvs_handrail_msg
  *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg *sim_model_lib0_Y_cmc_msg_c,
  imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg *sim_model_lib0_Y_env_msg_i,
  bpm_msg *sim_model_lib0_Y_bpm_msg_h, cvs_registration_pulse
  *sim_model_lib0_Y_cvs_registration_pulse_d, cvs_landmark_msg
  *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
  *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg *sim_model_lib0_Y_ex_time_msg_m)
{
  RT_MODEL_sim_model_lib0_T *sim_model_lib0_M;
  sim_model_lib0_M = (RT_MODEL_sim_model_lib0_T *) malloc(sizeof
    (RT_MODEL_sim_model_lib0_T));
  if (sim_model_lib0_M == NULL) {
    return NULL;
  }

  (void) memset((char *)sim_model_lib0_M, 0,
                sizeof(RT_MODEL_sim_model_lib0_T));

  // block I/O
  {
    B_sim_model_lib0_T *b = (B_sim_model_lib0_T *) malloc(sizeof
      (B_sim_model_lib0_T));
    rt_VALIDATE_MEMORY(sim_model_lib0_M,b);
    sim_model_lib0_M->blockIO = (b);
  }

  // parameters
  {
    P_sim_model_lib0_T *p;
    static int_T pSeen = 0;

    // only malloc on multiple model instantiation
    if (pSeen == 1 ) {
      p = (P_sim_model_lib0_T *) malloc(sizeof(P_sim_model_lib0_T));
      rt_VALIDATE_MEMORY(sim_model_lib0_M,p);
      (void) memcpy(p, &sim_model_lib0_P,
                    sizeof(P_sim_model_lib0_T));
      sim_model_lib0_M->paramIsMalloced = (true);
    } else {
      p = &sim_model_lib0_P;
      sim_model_lib0_M->paramIsMalloced = (false);
      pSeen = 1;
    }

    sim_model_lib0_M->defaultParam = (p);
  }

  // states (dwork)
  {
    DW_sim_model_lib0_T *dwork = (DW_sim_model_lib0_T *) malloc(sizeof
      (DW_sim_model_lib0_T));
    rt_VALIDATE_MEMORY(sim_model_lib0_M,dwork);
    sim_model_lib0_M->dwork = (dwork);
  }

  {
    P_sim_model_lib0_T *sim_model_lib0_P = ((P_sim_model_lib0_T *)
      sim_model_lib0_M->defaultParam);
    B_sim_model_lib0_T *sim_model_lib0_B = ((B_sim_model_lib0_T *)
      sim_model_lib0_M->blockIO);
    DW_sim_model_lib0_T *sim_model_lib0_DW = ((DW_sim_model_lib0_T *)
      sim_model_lib0_M->dwork);

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    // block I/O
    (void) memset(((void *) sim_model_lib0_B), 0,
                  sizeof(B_sim_model_lib0_T));

    // states (dwork)
    (void) memset((void *)sim_model_lib0_DW, 0,
                  sizeof(DW_sim_model_lib0_T));

    // external inputs
    *sim_model_lib0_U_act_msg_l = sim_model_lib0_rtZact_msg;
    *sim_model_lib0_U_cmc_msg_in = sim_model_lib0_rtZcmc_msg;

    // external outputs
    (*sim_model_lib0_Y_cvs_optical_flow_msg_n) =
      sim_model_lib0_rtZcvs_optical_flow_msg;
    (*sim_model_lib0_Y_cvs_handrail_msg_h) = sim_model_lib0_rtZcvs_handrail_msg;
    (*sim_model_lib0_Y_cmc_msg_c) = sim_model_lib0_rtZcmc_msg;
    (*sim_model_lib0_Y_imu_msg_o) = sim_model_lib0_rtZimu_msg;
    (*sim_model_lib0_Y_env_msg_i) = sim_model_lib0_rtZenv_msg;
    (*sim_model_lib0_Y_bpm_msg_h) = sim_model_lib0_rtZbpm_msg;
    (*sim_model_lib0_Y_cvs_registration_pulse_d) =
      sim_model_lib0_rtZcvs_registration_pulse;
    (*sim_model_lib0_Y_cvs_landmark_msg_n) = sim_model_lib0_rtZcvs_landmark_msg;
    (*sim_model_lib0_Y_cvs_ar_tag_msg) = sim_model_lib0_rtZcvs_landmark_msg;
    (*sim_model_lib0_Y_ex_time_msg_m) = sim_model_lib0_rtZex_time_msg;
  }

  return sim_model_lib0_M;
}

//
// File trailer for generated code.
//
// [EOF]
//
