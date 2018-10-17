//
// File: ert_main.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Aug 22 07:24:56 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "est_estimator.h"             // Model's header file
#include "rtwtypes.h"

// '<Root>/landmark_msg'
static cvs_landmark_msg est_estimator_U_landmark_msg;

// '<Root>/Vision Registration'
static cvs_registration_pulse est_estimator_U_VisionRegistration;

// '<Root>/optical_flow_msg'
static cvs_optical_flow_msg est_estimator_U_cvs_optical_flow_msg_n;

// '<Root>/handrail_msg'
static cvs_handrail_msg est_estimator_U_handrail_msg;

// '<Root>/imu_msg'
static imu_msg est_estimator_U_imu_msg_c;

// '<Root>/cmc_msg'
static cmc_msg est_estimator_U_cmc_msg_o;

// '<Root>/true_q_ISS2B'
static real32_T est_estimator_U_Q_ISS2B[4];

// '<Root>/kfl_msg'
static kfl_msg est_estimator_Y_kfl_msg_h;

// '<Root>/P_out'
static ase_cov_datatype est_estimator_Y_P_out[13689];
const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error";

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(RT_MODEL_est_estimator_T *const est_estimator_M);
void rt_OneStep(RT_MODEL_est_estimator_T *const est_estimator_M)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(est_estimator_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  est_estimator_step(est_estimator_M, &est_estimator_U_landmark_msg,
                     &est_estimator_U_VisionRegistration,
                     &est_estimator_U_cvs_optical_flow_msg_n,
                     &est_estimator_U_handrail_msg, &est_estimator_U_imu_msg_c,
                     &est_estimator_U_cmc_msg_o, est_estimator_U_Q_ISS2B,
                     &est_estimator_Y_kfl_msg_h, est_estimator_Y_P_out);

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustrates how you do this relative to initializing the model.
//
int_T main(int_T argc, const char *argv[])
{
  RT_MODEL_est_estimator_T *est_estimator_M;

  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Allocate model data
  est_estimator_M = est_estimator(&est_estimator_U_landmark_msg,
    &est_estimator_U_VisionRegistration, &est_estimator_U_cvs_optical_flow_msg_n,
    &est_estimator_U_handrail_msg, &est_estimator_U_imu_msg_c,
    &est_estimator_U_cmc_msg_o, est_estimator_U_Q_ISS2B,
    &est_estimator_Y_kfl_msg_h, est_estimator_Y_P_out);
  if (est_estimator_M == NULL) {
    (void)fprintf(stderr,"Memory allocation error during model "
                  "registration");
    return(1);
  }

  if (rtmGetErrorStatus(est_estimator_M) != NULL) {
    (void)fprintf(stderr,"Error during model registration: %s\n",
                  rtmGetErrorStatus(est_estimator_M));

    // Disable rt_OneStep() here

    // Terminate model
    est_estimator_terminate(est_estimator_M);
    return(1);
  }

  // Initialize model
  est_estimator_initialize(est_estimator_M, &est_estimator_U_landmark_msg,
    &est_estimator_U_VisionRegistration, &est_estimator_U_cvs_optical_flow_msg_n,
    &est_estimator_U_handrail_msg, &est_estimator_U_imu_msg_c,
    &est_estimator_U_cmc_msg_o, est_estimator_U_Q_ISS2B,
    &est_estimator_Y_kfl_msg_h, est_estimator_Y_P_out);

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.016 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep(est_estimator_M);

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(est_estimator_M) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  est_estimator_terminate(est_estimator_M);
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
