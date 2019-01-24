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
 *
 */

// Vive driver
#include <vive/vive.h>

// Commandline argument processing
#include <argtable2.h>

// Enable X and Y axis
int en0_ = 0;
int en1_ = 0;

// Callback to display light info
void my_light_process(tracker_t const * tracker, uint8_t lh, uint8_t axis,
  uint32_t synctime, uint16_t num_sensors, light_t const * event) {
  static float angcnv, lencnv;
  // Enable X and Y
  if (axis == 0 && en0_ == 0) return;
  if (axis == 1 && en1_ == 0) return;
  // Print header info
  printf("[%u] # %s LH %u %s\n", synctime, tracker->serial, (uint32_t) lh,
    (axis == MOTOR_AXIS0 ? "AXIS 0" : "AXIS 1"));
  // Print sensor info
  for (uint16_t i = 0; i < num_sensors; i++) {
    angcnv = (180.0 / 400000.0) * ((float)(event[i].timestamp));
    lencnv = ((float)event[i].length) / 48000000.0 * 1000000.0;
    printf(" ->  SEN (%2u) ANG (%f deg) LEN (%f us)\n",
      event[i].sensor_id, angcnv, lencnv);
  }
}

// Callback to display imu info
void my_imu_process(tracker_t const * tracker, imu_t const * event) {
  static float a[3], g[3];
  a[0] = (float)(event->acc[0]) * (9.80665/4096.0);
  a[1] = (float)(event->acc[1]) * (9.80665/4096.0);
  a[2] = (float)(event->acc[2]) * (9.80665/4096.0);
  g[0] = (float)(event->gyr[0]) * ((1./32.768)*(3.14159/180.));
  g[1] = (float)(event->gyr[1]) * ((1./32.768)*(3.14159/180.));
  g[2] = (float)(event->gyr[2]) * ((1./32.768)*(3.14159/180.));
  printf("[%010u] # %s I - ACC (%7.3f,%7.3f,%7.3f) GYR (%7.3f,%7.3f,%7.3f)\n",
    event->timestamp, tracker->serial, a[0], a[1], a[2], g[0], g[1], g[2]);
}

// Callback to display button info
void my_button_process(tracker_t const * tracker, button_t const * event) {
  if (event->mask & EVENT_TRIGGER)
    printf("[EVENT] TRIGGER_CLICK\n");
  else if (event->mask & EVENT_GRIP)
    printf("[EVENT] GRIP\n");
  else if (event->mask & EVENT_MENU)
    printf("[EVENT] MENU\n");
  else if (event->mask & EVENT_PAD_CLICK)
    printf("[EVENT] PAD_CLICK - %hd %hd\n", event->horizontal, event->vertical);
  else if (event->mask & EVENT_PAD_TOUCH)
    printf("[EVENT] PAD_TOUCH - %hd %hd\n", event->horizontal, event->vertical);
  else
    printf("[EVENT] TRIGGER - %hu\n", event->trigger);
}

// Called when tracker information is pulled
void my_tracker_process(tracker_t const * t) {
  if (!t) return;
  // Sensor
  printf("Metadata received for tracker with serial %s\n", t->serial);
  printf("- Photosensor positions\n");
  for (uint32_t i = 0; i < t->cal.num_channels; i++) {
    printf("  (SENSOR %u)\n", i);
    printf("    [POS] x = %f y = %f z = %f\n",
      t->cal.positions[i][0], t->cal.positions[i][1], t->cal.positions[i][2]);
    printf("    [NML] x = %f y = %f z = %f\n",
      t->cal.normals[i][0], t->cal.normals[i][1], t->cal.normals[i][2]);
  }
  // IMU calibration
  printf("- IMU calibration parameters:\n");
  printf("  (GYR B) x = %f y = %f z = %f\n",
    t->cal.gyr_bias[0],
    t->cal.gyr_bias[1],
    t->cal.gyr_bias[2]);
  printf("  (GYR S) x = %f y = %f z = %f\n",
    t->cal.gyr_scale[0],
    t->cal.gyr_scale[1],
    t->cal.gyr_scale[2]);
  printf("  (ACC B) x = %f y = %f z = %f\n",
    t->cal.acc_bias[0],
    t->cal.acc_bias[1],
    t->cal.acc_bias[2]);
  printf("  (ACC S) x = %f y = %f z = %f\n",
    t->cal.acc_scale[0],
    t->cal.acc_scale[1],
    t->cal.acc_scale[2]);
  // Transforms
  printf("- Transform for SENSOR -> IMU:\n");
  printf("  (ROT) w = %f  x = %f y = %f z = %f\n",
    t->cal.imu_transform[0],
    t->cal.imu_transform[1],
    t->cal.imu_transform[2],
    t->cal.imu_transform[3]);
  printf("  (POS) x = %f y = %f z = %f\n",
    t->cal.imu_transform[4],
    t->cal.imu_transform[5],
    t->cal.imu_transform[6]);
  printf("- Transform for SENSOR -> TRACKER:\n");
  printf("  (ROT) w = %f  x = %f y = %f z = %f\n",
    t->cal.head_transform[0],
    t->cal.head_transform[1],
    t->cal.head_transform[2],
    t->cal.head_transform[3]);
  printf("  (POS) x = %f y = %f z = %f\n",
    t->cal.head_transform[4],
    t->cal.head_transform[5],
    t->cal.head_transform[6]);
}

// Called when OOTX data is decoded from this lighthouse
void my_lighthouse_process(lighthouse_t const * l) {
  if (!l) return;
  // Sensor
  printf("Metadata received for lighthouse with serial  %s\n", l->serial);
  for (uint32_t i = 0; i < MAX_NUM_MOTORS; i++) {
    printf("- Motor %u calibration:\n", i);
    printf("  (PHASE) %f\n", l->motors[i].phase);
    printf("  (TILT) %f\n", l->motors[i].tilt);
    printf("  (GIBPHASE) %f\n", l->motors[i].gibphase);
    printf("  (GIBMAG) %f\n", l->motors[i].gibmag);
    printf("  (CURVE) %f\n", l->motors[i].curve);
  }
  printf("- Accel: x = %f y = %f z = %f\n",
    l->accel[0], l->accel[1], l->accel[2]);
  printf("- Mode: ");
  switch (l->mode_current) {
  case 0 : printf("A\n"); break;
  case 1 : printf("b\n"); break;
  case 2 : printf("c\n"); break;
  }
  printf("- Fault mask: 0x%02x\n", l->sys_faults);
  printf("- Firmware revision: 0x%04x\n", l->fw_version);
  printf("- Hardware revision: 0x%02x\n", l->hw_version);
}

// Main entry point for application
int main(int argc, char **argv) {
  // Get commandline arguments
  struct arg_lit  *imu     = arg_lit0("i", "imu", "print imu");
  struct arg_lit  *l0      = arg_lit0("0", "ax0", "print rotation about LH AXIS 0");
  struct arg_lit  *l1      = arg_lit0("1", "ax1", "print rotation about LH AXIS 1");
  struct arg_lit  *button  = arg_lit0("b", "button", "print buttons");
  struct arg_lit  *lh      = arg_lit0("l", "lh", "print lighthouse info");
  struct arg_lit  *tracker = arg_lit0("t", "tracker", "print tracker info");
  struct arg_lit  *help    = arg_lit0(NULL, "help", "print this help and exit");
  struct arg_end  *end     = arg_end(20);
  void* argtable[] = {imu, l0, l1, button, tracker, lh, help, end};
  // Verify we allocated correcty
  const char* progname = "deepdive_tool";
  int nerrors, exitcode = 0;
  if (arg_nullcheck(argtable) != 0) {
    printf("%s: insufficient memory\n", progname);
    exitcode = 1;
    goto exit;
  }
  // Parse the arguments
  nerrors = arg_parse(argc, argv, argtable);
  // Help takes precedence
  if (help->count > 0) {
    printf("Usage: %s", progname);
    arg_print_syntax(stdout, argtable, "\n");
    printf("This program extracts and prints data from a vive system.\n");
    arg_print_glossary(stdout, argtable,"  %-25s %s\n");
    exitcode = 0;
    goto exit;
  }
  /* If the parser returned any errors then display them and exit */
  if (nerrors > 0) {
    /* Display the error details contained in the arg_end struct.*/
    arg_print_errors(stdout,end,progname);
    printf("Try '%s --help' for more information.\n", progname);
    exitcode = 2;
    goto exit;
  }
  // Initialize the driver
  driver_t * drv = vive_init(0);
  if (!drv) {
    printf("%s: could not initialize driver\n", progname);
    exitcode = 3;
    goto exit;
  }
  // Limit to X or Y
  en0_ = l0->count;
  en1_ = l1->count;
  // Install callbacks
  if (imu->count > 0)
    vive_install_imu_fn(drv, my_imu_process);
  if (l0->count + l1->count > 0)
    vive_install_light_fn(drv, my_light_process);
  if (button->count > 0)
    vive_install_button_fn(drv, my_button_process);
  if (lh->count > 0)
    vive_install_lighthouse_fn(drv, my_lighthouse_process);
  if (tracker->count > 0)
    vive_install_tracker_fn(drv, my_tracker_process);
  // Keep going until ctrl+c
  while (vive_poll(drv) == 0) {}
    return 0;
  // Cleanup
  vive_kill(drv);
  // Exit cleanly
  exitcode = 0;
exit:
  arg_freetable(argtable, sizeof(argtable) / sizeof(argtable[0]));
  return exitcode;
}
