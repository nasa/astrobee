//
// File: ert_main.cpp
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec 18 10:14:45 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "ctl_controller0.h"           // Model's header file
#include "rtwtypes.h"

// '<Root>/ctl_input_msg'
static ctl_input_msg ctl_controller0_U_ctl_input_msg_l;

// '<Root>/cmd_msg'
static cmd_msg ctl_controller0_Y_cmd_msg_f;

// '<Root>/ctl_msg'
static ctl_msg ctl_controller0_Y_ctl_msg_n;
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
void rt_OneStep(RT_MODEL_ctl_controller0_T *const ctl_controller0_M);
void rt_OneStep(RT_MODEL_ctl_controller0_T *const ctl_controller0_M)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(ctl_controller0_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  ctl_controller0_step(ctl_controller0_M, &ctl_controller0_U_ctl_input_msg_l,
                       &ctl_controller0_Y_cmd_msg_f,
                       &ctl_controller0_Y_ctl_msg_n);

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
  RT_MODEL_ctl_controller0_T *ctl_controller0_M;

  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Allocate model data
  ctl_controller0_M = ctl_controller0(&ctl_controller0_U_ctl_input_msg_l,
    &ctl_controller0_Y_cmd_msg_f, &ctl_controller0_Y_ctl_msg_n);
  if (ctl_controller0_M == NULL) {
    (void)fprintf(stderr,"Memory allocation error during model "
                  "registration");
    return(1);
  }

  if (rtmGetErrorStatus(ctl_controller0_M) != NULL) {
    (void)fprintf(stderr,"Error during model registration: %s\n",
                  rtmGetErrorStatus(ctl_controller0_M));

    // Disable rt_OneStep() here

    // Terminate model
    ctl_controller0_terminate(ctl_controller0_M);
    return(1);
  }

  // Initialize model
  ctl_controller0_initialize(ctl_controller0_M,
    &ctl_controller0_U_ctl_input_msg_l, &ctl_controller0_Y_cmd_msg_f,
    &ctl_controller0_Y_ctl_msg_n);

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.016 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep(ctl_controller0_M);

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(ctl_controller0_M) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  ctl_controller0_terminate(ctl_controller0_M);
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
