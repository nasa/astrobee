//
// File: ert_main.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Aug 22 07:26:01 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "fam_force_allocation_module.h" // Model's header file
#include "rtwtypes.h"

// '<Root>/current_time'
static ex_time_msg fam_force_allocation_module_U_current_time;

// '<Root>/cmd_msg'
static cmd_msg fam_force_allocation_module_U_cmd_msg_f;

// '<Root>/ctl_msg'
static ctl_msg fam_force_allocation_module_U_ctl_msg_n;

// '<Root>/cmc_msg'
static cmc_msg fam_force_allocation_module_U_cmc_msg_h;

// '<Root>/act_msg'
static act_msg fam_force_allocation_module_Y_act_msg_c;
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
void rt_OneStep(RT_MODEL_fam_force_allocation_T *const
                fam_force_allocation_module_M);
void rt_OneStep(RT_MODEL_fam_force_allocation_T *const
                fam_force_allocation_module_M)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(fam_force_allocation_module_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  fam_force_allocation_module_step(fam_force_allocation_module_M,
    &fam_force_allocation_module_U_current_time,
    &fam_force_allocation_module_U_cmd_msg_f,
    &fam_force_allocation_module_U_ctl_msg_n,
    &fam_force_allocation_module_U_cmc_msg_h,
    &fam_force_allocation_module_Y_act_msg_c);

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
  RT_MODEL_fam_force_allocation_T *fam_force_allocation_module_M;

  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Allocate model data
  fam_force_allocation_module_M = fam_force_allocation_module
    (&fam_force_allocation_module_U_current_time,
     &fam_force_allocation_module_U_cmd_msg_f,
     &fam_force_allocation_module_U_ctl_msg_n,
     &fam_force_allocation_module_U_cmc_msg_h,
     &fam_force_allocation_module_Y_act_msg_c);
  if (fam_force_allocation_module_M == NULL) {
    (void)fprintf(stderr,"Memory allocation error during model "
                  "registration");
    return(1);
  }

  if (rtmGetErrorStatus(fam_force_allocation_module_M) != NULL) {
    (void)fprintf(stderr,"Error during model registration: %s\n",
                  rtmGetErrorStatus(fam_force_allocation_module_M));

    // Disable rt_OneStep() here

    // Terminate model
    fam_force_allocation_module_terminate(fam_force_allocation_module_M);
    return(1);
  }

  // Initialize model
  fam_force_allocation_module_initialize(fam_force_allocation_module_M,
    &fam_force_allocation_module_U_current_time,
    &fam_force_allocation_module_U_cmd_msg_f,
    &fam_force_allocation_module_U_ctl_msg_n,
    &fam_force_allocation_module_U_cmc_msg_h,
    &fam_force_allocation_module_Y_act_msg_c);

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.016 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep(fam_force_allocation_module_M);

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(fam_force_allocation_module_M) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  fam_force_allocation_module_terminate(fam_force_allocation_module_M);
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
