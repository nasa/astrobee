//
// File: ert_main.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Aug 22 07:27:02 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "sim_model_lib0.h"            // Model's header file
#include "rtwtypes.h"

// '<Root>/act_msg'
static act_msg sim_model_lib0_U_act_msg_l;

// '<Root>/cmc_msg_in'
static cmc_msg sim_model_lib0_U_cmc_msg_in;

// '<Root>/cvs_optical_flow_msg'
static cvs_optical_flow_msg sim_model_lib0_Y_cvs_optical_flow_msg_n;

// '<Root>/cvs_handrail_msg'
static cvs_handrail_msg sim_model_lib0_Y_cvs_handrail_msg_h;

// '<Root>/cmc_msg'
static cmc_msg sim_model_lib0_Y_cmc_msg_c;

// '<Root>/imu_msg'
static imu_msg sim_model_lib0_Y_imu_msg_o;

// '<Root>/env_msg'
static env_msg sim_model_lib0_Y_env_msg_i;

// '<Root>/bpm_msg'
static bpm_msg sim_model_lib0_Y_bpm_msg_h;

// '<Root>/cvs_registration_pulse'
static cvs_registration_pulse sim_model_lib0_Y_cvs_registration_pulse_d;

// '<Root>/cvs_landmark_msg'
static cvs_landmark_msg sim_model_lib0_Y_cvs_landmark_msg_n;

// '<Root>/cvs_ar_tag_msg'
static cvs_landmark_msg sim_model_lib0_Y_cvs_ar_tag_msg;

// '<Root>/ex_time_msg'
static ex_time_msg sim_model_lib0_Y_ex_time_msg_m;
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
void rt_OneStep(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M);
void rt_OneStep(RT_MODEL_sim_model_lib0_T *const sim_model_lib0_M)
{
  static boolean_T OverrunFlags[5] = { 0, 0, 0, 0, 0 };

  static boolean_T eventFlags[5] = { 0, 0, 0, 0, 0 };// Model has 5 rates

  static int_T taskCounter[5] = { 0, 0, 0, 0, 0 };

  int_T i;

  // Disable interrupts here

  // Check base rate for overrun
  if (OverrunFlags[0]) {
    rtmSetErrorStatus(sim_model_lib0_M, "Overrun");
    return;
  }

  OverrunFlags[0] = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here

  //
  //  For a bare-board target (i.e., no operating system), the
  //  following code checks whether any subrate overruns,
  //  and also sets the rates that need to run this time step.

  for (i = 1; i < 5; i++) {
    if (taskCounter[i] == 0) {
      if (eventFlags[i]) {
        OverrunFlags[0] = false;
        OverrunFlags[i] = true;

        // Sampling too fast
        rtmSetErrorStatus(sim_model_lib0_M, "Overrun");
        return;
      }

      eventFlags[i] = true;
    }
  }

  taskCounter[1]++;
  if (taskCounter[1] == 10) {
    taskCounter[1]= 0;
  }

  taskCounter[2]++;
  if (taskCounter[2] == 13) {
    taskCounter[2]= 0;
  }

  taskCounter[3]++;
  if (taskCounter[3] == 17) {
    taskCounter[3]= 0;
  }

  taskCounter[4]++;
  if (taskCounter[4] == 21) {
    taskCounter[4]= 0;
  }

  // Set model inputs associated with base rate here

  // Step the model for base rate
  sim_model_lib0_step0(sim_model_lib0_M, &sim_model_lib0_U_act_msg_l,
                       &sim_model_lib0_U_cmc_msg_in,
                       &sim_model_lib0_Y_cvs_optical_flow_msg_n,
                       &sim_model_lib0_Y_cvs_handrail_msg_h,
                       &sim_model_lib0_Y_cmc_msg_c, &sim_model_lib0_Y_imu_msg_o,
                       &sim_model_lib0_Y_env_msg_i, &sim_model_lib0_Y_bpm_msg_h,
                       &sim_model_lib0_Y_cvs_registration_pulse_d,
                       &sim_model_lib0_Y_cvs_landmark_msg_n,
                       &sim_model_lib0_Y_cvs_ar_tag_msg,
                       &sim_model_lib0_Y_ex_time_msg_m);

  // Get model outputs here

  // Indicate task for base rate complete
  OverrunFlags[0] = false;

  // Step the model for any subrate
  for (i = 1; i < 5; i++) {
    // If task "i" is running, don't run any lower priority task
    if (OverrunFlags[i]) {
      return;
    }

    if (eventFlags[i]) {
      OverrunFlags[i] = true;

      // Set model inputs associated with subrates here

      // Step the model for subrate "i"
      switch (i) {
       case 1 :
        sim_model_lib0_step1(sim_model_lib0_M);

        // Get model outputs here
        break;

       case 2 :
        sim_model_lib0_step2(sim_model_lib0_M);

        // Get model outputs here
        break;

       case 3 :
        sim_model_lib0_step3(sim_model_lib0_M);

        // Get model outputs here
        break;

       case 4 :
        sim_model_lib0_step4(sim_model_lib0_M);

        // Get model outputs here
        break;

       default :
        break;
      }

      // Indicate task complete for sample time "i"
      OverrunFlags[i] = false;
      eventFlags[i] = false;
    }
  }

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
  RT_MODEL_sim_model_lib0_T *sim_model_lib0_M;

  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Allocate model data
  sim_model_lib0_M = sim_model_lib0(&sim_model_lib0_U_act_msg_l,
    &sim_model_lib0_U_cmc_msg_in, &sim_model_lib0_Y_cvs_optical_flow_msg_n,
    &sim_model_lib0_Y_cvs_handrail_msg_h, &sim_model_lib0_Y_cmc_msg_c,
    &sim_model_lib0_Y_imu_msg_o, &sim_model_lib0_Y_env_msg_i,
    &sim_model_lib0_Y_bpm_msg_h, &sim_model_lib0_Y_cvs_registration_pulse_d,
    &sim_model_lib0_Y_cvs_landmark_msg_n, &sim_model_lib0_Y_cvs_ar_tag_msg,
    &sim_model_lib0_Y_ex_time_msg_m);
  if (sim_model_lib0_M == NULL) {
    (void)fprintf(stderr,"Memory allocation error during model "
                  "registration");
    return(1);
  }

  if (rtmGetErrorStatus(sim_model_lib0_M) != NULL) {
    (void)fprintf(stderr,"Error during model registration: %s\n",
                  rtmGetErrorStatus(sim_model_lib0_M));

    // Disable rt_OneStep() here

    // Terminate model
    sim_model_lib0_terminate(sim_model_lib0_M);
    return(1);
  }

  // Initialize model
  sim_model_lib0_initialize(sim_model_lib0_M, &sim_model_lib0_U_act_msg_l,
    &sim_model_lib0_U_cmc_msg_in, &sim_model_lib0_Y_cvs_optical_flow_msg_n,
    &sim_model_lib0_Y_cvs_handrail_msg_h, &sim_model_lib0_Y_cmc_msg_c,
    &sim_model_lib0_Y_imu_msg_o, &sim_model_lib0_Y_env_msg_i,
    &sim_model_lib0_Y_bpm_msg_h, &sim_model_lib0_Y_cvs_registration_pulse_d,
    &sim_model_lib0_Y_cvs_landmark_msg_n, &sim_model_lib0_Y_cvs_ar_tag_msg,
    &sim_model_lib0_Y_ex_time_msg_m);

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.016 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep(sim_model_lib0_M);

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(sim_model_lib0_M) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  sim_model_lib0_terminate(sim_model_lib0_M);
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
