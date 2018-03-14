//
// File: ert_main.cpp
//
// Code generated for Simulink model 'bpm_blower_1_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Jan 31 12:34:43 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "bpm_blower_1_propulsion_module.h" // Model's header file
#include "rtwtypes.h"

// '<Root>/battery_voltage'
static real32_T bpm_blower_1_propulsion_modul_U_battery_voltage;

// '<Root>/omega_B_ECI_B'
static real32_T bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[3];

// '<Root>/impeller_cmd'
static uint8_T bpm_blower_1_propulsion_modul_U_impeller_cmd;

// '<Root>/servo_cmd'
static real32_T bpm_blower_1_propulsion_modul_U_servo_cmd[6];

// '<Root>/veh_cm'
static real32_T bpm_blower_1_propulsion_modul_U_veh_cm[3];

// '<Root>/impeller_current'
static real32_T bpm_blower_1_propulsion_modul_Y_impeller_current;

// '<Root>/servo_current'
static real32_T bpm_blower_1_propulsion_modul_Y_servo_current[6];

// '<Root>/torque_B'
static real32_T bpm_blower_1_propulsion_modul_Y_torque_B[3];

// '<Root>/force_B'
static real32_T bpm_blower_1_propulsion_modul_Y_force_B[3];

// '<Root>/motor_speed'
static real32_T bpm_blower_1_propulsion_modul_Y_motor_speed;

// '<Root>/nozzle_theta'
static real32_T bpm_blower_1_propulsion_modul_Y_nozzle_theta[6];

// '<Root>/meas_motor_speed'
static real32_T bpm_blower_1_propulsion_modul_Y_meas_motor_speed;
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
void rt_OneStep(RT_MODEL_bpm_blower_1_propuls_T *const
                bpm_blower_1_propulsion_modu_M);
void rt_OneStep(RT_MODEL_bpm_blower_1_propuls_T *const
                bpm_blower_1_propulsion_modu_M)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(bpm_blower_1_propulsion_modu_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  bpm_blower_1_propulsion_module_step(bpm_blower_1_propulsion_modu_M,
    bpm_blower_1_propulsion_modul_U_battery_voltage,
    bpm_blower_1_propulsion_modul_U_omega_B_ECI_B,
    bpm_blower_1_propulsion_modul_U_impeller_cmd,
    bpm_blower_1_propulsion_modul_U_servo_cmd,
    bpm_blower_1_propulsion_modul_U_veh_cm,
    &bpm_blower_1_propulsion_modul_Y_impeller_current,
    bpm_blower_1_propulsion_modul_Y_servo_current,
    bpm_blower_1_propulsion_modul_Y_torque_B,
    bpm_blower_1_propulsion_modul_Y_force_B,
    &bpm_blower_1_propulsion_modul_Y_motor_speed,
    bpm_blower_1_propulsion_modul_Y_nozzle_theta,
    &bpm_blower_1_propulsion_modul_Y_meas_motor_speed);

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
  RT_MODEL_bpm_blower_1_propuls_T *bpm_blower_1_propulsion_modu_M;

  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Allocate model data
  bpm_blower_1_propulsion_modu_M = bpm_blower_1_propulsion_module
    (&bpm_blower_1_propulsion_modul_U_battery_voltage,
     bpm_blower_1_propulsion_modul_U_omega_B_ECI_B,
     &bpm_blower_1_propulsion_modul_U_impeller_cmd,
     bpm_blower_1_propulsion_modul_U_servo_cmd,
     bpm_blower_1_propulsion_modul_U_veh_cm,
     &bpm_blower_1_propulsion_modul_Y_impeller_current,
     bpm_blower_1_propulsion_modul_Y_servo_current,
     bpm_blower_1_propulsion_modul_Y_torque_B,
     bpm_blower_1_propulsion_modul_Y_force_B,
     &bpm_blower_1_propulsion_modul_Y_motor_speed,
     bpm_blower_1_propulsion_modul_Y_nozzle_theta,
     &bpm_blower_1_propulsion_modul_Y_meas_motor_speed);
  if (bpm_blower_1_propulsion_modu_M == NULL) {
    (void)fprintf(stderr,"Memory allocation error during model "
                  "registration");
    return(1);
  }

  if (rtmGetErrorStatus(bpm_blower_1_propulsion_modu_M) != NULL) {
    (void)fprintf(stderr,"Error during model registration: %s\n",
                  rtmGetErrorStatus(bpm_blower_1_propulsion_modu_M));

    // Disable rt_OneStep() here

    // Terminate model
    bpm_blower_1_propulsion_module_terminate(bpm_blower_1_propulsion_modu_M);
    return(1);
  }

  // Initialize model
  bpm_blower_1_propulsion_module_initialize(bpm_blower_1_propulsion_modu_M,
    &bpm_blower_1_propulsion_modul_U_battery_voltage,
    bpm_blower_1_propulsion_modul_U_omega_B_ECI_B,
    &bpm_blower_1_propulsion_modul_U_impeller_cmd,
    bpm_blower_1_propulsion_modul_U_servo_cmd,
    bpm_blower_1_propulsion_modul_U_veh_cm,
    &bpm_blower_1_propulsion_modul_Y_impeller_current,
    bpm_blower_1_propulsion_modul_Y_servo_current,
    bpm_blower_1_propulsion_modul_Y_torque_B,
    bpm_blower_1_propulsion_modul_Y_force_B,
    &bpm_blower_1_propulsion_modul_Y_motor_speed,
    bpm_blower_1_propulsion_modul_Y_nozzle_theta,
    &bpm_blower_1_propulsion_modul_Y_meas_motor_speed);

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.016 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep(bpm_blower_1_propulsion_modu_M);

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(bpm_blower_1_propulsion_modu_M) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  bpm_blower_1_propulsion_module_terminate(bpm_blower_1_propulsion_modu_M);
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
