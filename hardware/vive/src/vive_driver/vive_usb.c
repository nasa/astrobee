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
 * Based off libsurvive: https://github.com/cnlohr/libsurvive
 */

// Interface implementations
#include "vive_usb.h"

// Controller implementations
#include "vive_dev_tracker.h"
#include "vive_dev_watchman.h"

// ZLib library for unzipping tracker configuration
#include <zlib.h>

// JSON library for parsing tracker configuration
#include <json-c/json.h>

// C standard libraries
#include <math.h>
#include <stdio.h>
#include <errno.h>

// Special codes
static uint8_t magic_code_power_en_[5] = {0x04};

// Config version
typedef enum { CONFIG_VERSION_1, CONFIG_VERSION_2 } config_version_t;

// Interrupt handler
static void interrupt_handler(struct libusb_transfer* t) {
  endpoint_t * ep = t->user_data;
  if (t->status != LIBUSB_TRANSFER_COMPLETED ) {
    DEBUG_PRINTF("Transfer problem\n");
    return;
  }
  switch (ep->type) {
  case DATA_IMU:
    vive_dev_tracker_imu(ep->tracker, ep->buffer, t->actual_length);
    break;
  case DATA_LIGHT_9:
    vive_dev_tracker_light_9(ep->tracker, ep->buffer, t->actual_length);
    break;
  case DATA_LIGHT_7:
    vive_dev_tracker_light_7(ep->tracker, ep->buffer, t->actual_length);
    break;
  case DATA_BUTTONS:
    vive_dev_tracker_button(ep->tracker, ep->buffer, t->actual_length);
    break;
  case DATA_WATCHMAN:
    vive_dev_watchman(ep->tracker, ep->buffer, t->actual_length);
    break;
  }
  if (libusb_submit_transfer(t))
    DEBUG_PRINTF( "Error resubmitting transfer\n");
}

static inline int update_feature_report(libusb_device_handle* dev,
  uint16_t interface, uint8_t * data, int datalen) {
  return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS
    | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT, 0x09, 0x300 | data[0],
      interface, data, datalen, 1000 );
}


static inline int getupdate_feature_report(libusb_device_handle* dev,
  uint16_t interface, uint8_t * data, int datalen ) {
  return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS
    | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN, 0x01, 0x300 | data[0],
    interface, data, datalen, 1000 );
}

static inline int hid_get_feature_report_timeout(libusb_device_handle* dev,
  uint16_t iface, unsigned char *buf, size_t len ) {
  int ret;
  for (uint8_t i = 0; i < 50; i++) {
    ret = getupdate_feature_report(dev, iface, buf, len);
    if (ret != -9 && (ret != -1 || errno != EPIPE))
      return ret;
    usleep(1000);
  }
  return -1;
}

static int decompress(const char * input,
  int ilen, char * output, int olen) {
  z_stream zs;
  memset(&zs, 0, sizeof( zs ));
  inflateInit(&zs);
  zs.avail_in = ilen;
  zs.next_in = (z_const Bytef *) input;
  zs.avail_out = olen;
  zs.next_out = (unsigned char *) output;
  if (inflate( &zs, Z_FINISH) != Z_STREAM_END) {
    DEBUG_PRINTF("Could not inflate.");
    return -1;
  }
  int len = zs.total_out;
  inflateEnd(&zs);
  return len;
}

// Read an array from a json array data structure
static int json_read_arr_dbl(json_object * jobj, float *data, size_t len) {
  // Check that this is an array
  enum json_type type = json_object_get_type(jobj);
  if (type != json_type_array)
    return 0;
  // Read the min(desired,actual) length
  int maxlen = json_object_array_length(jobj);
  if (maxlen < len)
    len = maxlen;
  // Now read the elements
  json_object * jval;
  int numread = 0;
  for (size_t i = 0; i < len; i++) {
    jval = json_object_array_get_idx(jobj, i);
    data[numread++] =  json_object_get_double(jval);
  }
  return numread;
}

// Read an array from a json array data structure
static int json_read_arr_int(json_object * jobj, uint8_t *data, size_t len) {
  // Check that this is an array
  enum json_type type = json_object_get_type(jobj);
  if (type != json_type_array)
    return 0;
  // Read the min(desired,actual) length
  int maxlen = json_object_array_length(jobj);
  if (maxlen < len)
    len = maxlen;
  // Now read the elements
  json_object * jval;
  int numread = 0;
  for (size_t  i = 0; i < len; i++) {
    jval = json_object_array_get_idx(jobj, i);
    data[numread++] = json_object_get_int(jval);
  }
  return numread;
}

// Vector cross product
static void cross_product(float ab[3],float a[3], float b[3]) {
  ab[0] = a[1] * b[2] - a[2] * b[1];
  ab[1] = a[2] * b[0] - a[0] * b[2];
  ab[2] = a[0] * b[1] - a[1] * b[0];
}

// Create a quaternion from a rotation matrix
static void quat_from_dcm(float q[4], const float R[3][3]) {
  float trace = R[0][0] + R[1][1] + R[2][2];
  if (trace > 0 ) {
     float s = 0.5f / sqrt(trace + 1.0f);
     q[3] = 0.25f / s;
     q[0] = ( R[2][1] - R[1][2] ) * s;
     q[1] = ( R[0][2] - R[2][0] ) * s;
     q[2] = ( R[1][0] - R[0][1] ) * s;
  } else {
    if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      float s = 2.0f * sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]);
      q[3] = (R[2][1] - R[1][2] ) / s;
      q[0] = 0.25f * s;
      q[1] = (R[0][1] + R[1][0] ) / s;
      q[2] = (R[0][2] + R[2][0] ) / s;
    } else if (R[1][1] > R[2][2]) {
      float s = 2.0f * sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]);
      q[3] = (R[0][2] - R[2][0] ) / s;
      q[0] = (R[0][1] + R[1][0] ) / s;
      q[1] = 0.25f * s;
      q[2] = (R[1][2] + R[2][1] ) / s;
    } else {
      float s = 2.0f * sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]);
      q[3] = (R[1][0] - R[0][1] ) / s;
      q[0] = (R[0][2] + R[2][0] ) / s;
      q[1] = (R[1][2] + R[2][1] ) / s;
      q[2] = 0.25f * s;
    }
  }
}

// Quaternion from two vectors
static void quat_from_two_vectors(float transform[7]) {
  float R[3][3], v[3];
  cross_product(v, &transform[3], &transform[0]);
  memcpy(R[0], &transform[0], 3 * sizeof(float));
  memcpy(R[1],            &v, 3 * sizeof(float));
  memcpy(R[2], &transform[3], 3 * sizeof(float));
  quat_from_dcm(transform, R);
}

// Parse the jscond evice configuation
static void json_parse(tracker_t * tracker,
                      const char * data,
                      config_version_t version) {
  json_object *jobj = json_tokener_parse(data);
  json_object *jtmp, *jimu, *jhead;
  // IMU calibration parameters
  switch (version) {
  case CONFIG_VERSION_1:
    if (json_object_object_get_ex(jobj, "acc_bias", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.acc_bias, 3))
        DEBUG_PRINTF("Could not read the JSON field: acc_bias\n");
    if (json_object_object_get_ex(jobj, "acc_scale", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.acc_scale, 3))
        DEBUG_PRINTF("Could not read the JSON field: acc_scale\n");
    if (json_object_object_get_ex(jobj, "gyro_bias", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_bias, 3))
        DEBUG_PRINTF("Could not read the JSON field: gyr_bias\n");
    if (json_object_object_get_ex(jobj, "gyro_scale", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_scale, 3))
        DEBUG_PRINTF("Could not read the JSON field: gyr_scale\n");
    if (json_object_object_get_ex(jobj, "trackref_from_imu", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.imu_transform, 7))
        DEBUG_PRINTF("Could not read the JSON field: trackref_from_imu\n");
    if (json_object_object_get_ex(jobj, "trackref_from_head", &jtmp))
      if (!json_read_arr_dbl(jtmp, tracker->cal.head_transform, 7))
        DEBUG_PRINTF("Could not read the JSON field: trackref_from_head\n");
    break;
  case CONFIG_VERSION_2:
    // Get the IMU parameters -- they are not in the JSON block "imu"
    if (json_object_object_get_ex(jobj, "imu", &jimu)) {
      if (json_object_object_get_ex(jimu, "acc_bias", &jtmp))
        if (!json_read_arr_dbl(jtmp, tracker->cal.acc_bias, 3))
          DEBUG_PRINTF("Could not read the JSON field: acc_bias\n");
      if (json_object_object_get_ex(jimu, "acc_scale", &jtmp))
        if (!json_read_arr_dbl(jtmp, tracker->cal.acc_scale, 3))
          DEBUG_PRINTF("Could not read the JSON field: acc_scale\n");
      if (json_object_object_get_ex(jimu, "gyro_bias", &jtmp))
        if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_bias, 3))
          DEBUG_PRINTF("Could not read the JSON field: gyr_bias\n");
      if (json_object_object_get_ex(jimu, "gyro_scale", &jtmp))
        if (!json_read_arr_dbl(jtmp, tracker->cal.gyr_scale, 3))
          DEBUG_PRINTF("Could not read the JSON field: gyr_scale\n");
      if (json_object_object_get_ex(jimu, "plus_x", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.imu_transform[0], 3))
          DEBUG_PRINTF("Could not read the JSON field: plus_x\n");
      if (json_object_object_get_ex(jimu, "plus_z", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.imu_transform[3], 3))
          DEBUG_PRINTF("Could not read the JSON field: plus_z\n");
      quat_from_two_vectors(tracker->cal.imu_transform);
      if (json_object_object_get_ex(jimu, "position", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.imu_transform[4], 3))
          DEBUG_PRINTF("Could not read the JSON field: position\n");
    } else {
      DEBUG_PRINTF("Could not read the JSON field: head\n");
    }
    // Get the extrinsic information
    if (json_object_object_get_ex(jobj, "head", &jhead)) {
      if (json_object_object_get_ex(jhead, "plus_x", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.head_transform[0], 3))
          DEBUG_PRINTF("Could not read the JSON field: plus_x\n");
      if (json_object_object_get_ex(jhead, "plus_z", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.head_transform[3], 3))
          DEBUG_PRINTF("Could not read the JSON field: plus_z\n");
      quat_from_two_vectors(tracker->cal.head_transform);
      if (json_object_object_get_ex(jhead, "position", &jtmp))
        if (!json_read_arr_dbl(jtmp, &tracker->cal.head_transform[4], 3))
          DEBUG_PRINTF("Could not read the JSON field: position\n");
    } else {
      DEBUG_PRINTF("Could not read the JSON field: head\n");
    }
    break;
  }

  if (json_object_object_get_ex(jobj, "device_serial_number", &jtmp))
    strcpy((char *) tracker->serial, json_object_get_string(jtmp));
  // Photodiode calibration parameters
  json_object *jlhc;
  if (json_object_object_get_ex(jobj, "lighthouse_config", &jlhc)) {
    if (json_object_object_get_ex(jlhc, "channelMap", &jtmp)) {
      tracker->cal.num_channels = json_read_arr_int(
        jtmp, tracker->cal.channels, MAX_NUM_SENSORS);
      if (tracker->cal.num_channels > 0
       && tracker->cal.num_channels < MAX_NUM_SENSORS) {
        json_object *jnrm;
        if (json_object_object_get_ex(jlhc, "modelNormals", &jnrm)) {
          for (int i = 0; i < tracker->cal.num_channels; i++) {
            jtmp = json_object_array_get_idx(jnrm, i);
            if (!json_read_arr_dbl(jtmp, tracker->cal.normals[i], 3))
              DEBUG_PRINTF("Could not read the normal for channel %u\n", i);
          }
        }
        json_object *jpts;
        if (json_object_object_get_ex(jlhc, "modelPoints", &jpts)) {
          for (int i = 0; i < tracker->cal.num_channels; i++) {
            jtmp = json_object_array_get_idx(jpts, i);
            if (!json_read_arr_dbl(jtmp, tracker->cal.positions[i], 3))
              DEBUG_PRINTF("Could not read the position for channel %u\n", i);
          }
        }
      } else {
        DEBUG_PRINTF("Could not read the JSON: lighthouse_config::channelMap\n");
      }
    }
  }
  // Mark as valid!
  tracker->cal.timestamp = 1;
  DEBUG_PRINTF("Read calibration data for tracker %s\n", tracker->serial);
}

// Read the tracker configuration (sensor extrinsics and imu bias/scale)
static int get_config(tracker_t * tracker,
                      int send_extra_magic,
                      config_version_t version) {
  int ret, count = 0, size = 0;
  uint8_t cfgbuff[64];
  char compressed_data[8192];
  char uncompressed_data[65536];
  // Send a magic code to iniitalize the config download process
  if (send_extra_magic) {
    uint8_t cfgbuffwide[65];
    memset(cfgbuffwide, 0, sizeof(cfgbuff));
    cfgbuffwide[0] = 0x01;
    ret = hid_get_feature_report_timeout(
      tracker->udev, 0, cfgbuffwide,sizeof(cfgbuffwide) );
    usleep(1000);
    uint8_t cfgbuff_send[64] = { 0xff, 0x83 };
    for (int k = 0; k < 10; k++ ) {
      update_feature_report(tracker->udev, 0, cfgbuff_send, 64);
      usleep(1000);
    }
    cfgbuffwide[0] = 0xff;
    ret = hid_get_feature_report_timeout(
      tracker->udev, 0, cfgbuffwide, sizeof(cfgbuffwide));
    usleep(1000);
  }
  // Send Report 16 to prepare the device for reading config info
  memset(cfgbuff, 0, sizeof(cfgbuff));
  cfgbuff[0] = 0x10;
  if ((ret = hid_get_feature_report_timeout(
    tracker->udev, 0, cfgbuff, sizeof(cfgbuff))) < 0 ) {
    DEBUG_PRINTF( "No configuration found: Watchman is probably not paired.\n");
    return -1;
  }
  // Now do a bunch of Report 17 until there are no bytes left
  cfgbuff[1] = 0xaa;
  cfgbuff[0] = 0x11;
  do {
    if((ret = hid_get_feature_report_timeout(
        tracker->udev, 0, cfgbuff, sizeof(cfgbuff))) < 0 ) {
      DEBUG_PRINTF("Could not read config data on device (count: %d)\n", count );
      return -2;
    }
    size = cfgbuff[1];
    if (!size) break;
    if( size > 62 ) {
      DEBUG_PRINTF("Too much data (%d) on packet (count: %d)", size, count);
      return -3;
    }
    if (count + size >= sizeof(compressed_data)) {
      DEBUG_PRINTF("Configuration length too long (count: %d)", count);
      return -4;
    }
    memcpy(&compressed_data[count], cfgbuff + 2, size);
    count += size;
  } while( 1 );

  if (count == 0) {
    DEBUG_PRINTF( "Empty configuration");
    return -5;
  }
  // Decompress the data
  int len = decompress(compressed_data, count,
    uncompressed_data, sizeof(uncompressed_data));
  if (len <= 0) {
    DEBUG_PRINTF( "Error: data for config descriptor is bad. (%d)", len);
    return -5;
  }
  /*
  char fstname[128];
  sprintf(fstname, "%s.json", tracker->serial);
  FILE *f = fopen( fstname, "wb" );
  fwrite(uncompressed_data, len, 1, f);
  fclose(f);
  */
  // Parse the JSON data structure
  json_parse(tracker, uncompressed_data, version);
  return 0;
}

// Enumerate all USBs on the bus and return the number of devices found
uint8_t vive_usb_init(driver_t * drv) {
  // Initialize libusb
  int ret = libusb_init(&drv->usb);
  if (ret)
  {
    printf("Error: the libusb_init function returned %d which is not zero \n", ret);
    return 0;
  }

  // Get a list of devices
  libusb_device** devs;
  ret = libusb_get_device_list(drv->usb, &devs);
  if (ret < 0)
  {
    printf("Error: the libusb_get_device_list function returned %d which is not zero \n", ret);
    return 0;
  }

  // Iterate over the devices looking for vive products
  struct libusb_device_descriptor desc;
  struct libusb_device * dev;
  int did = 0;
  while ((dev = devs[did++])) {
    // Get the device descriptor
    ret = libusb_get_device_descriptor(dev, &desc);
    if (ret < 0 || dev == 0 || desc.idVendor != USB_VEND_HTC)
      continue;

    // Get a config descriptor
    struct libusb_config_descriptor *conf;
    ret = libusb_get_config_descriptor(dev, 0, &conf);
    if (ret)
      continue;

    // Allocate the tracker memory
    tracker_t * tracker = malloc(sizeof(tracker_t));
    if (!tracker)
      continue;
    // Make sure the memory is zeroed
    memset(tracker, 0, sizeof(tracker_t));
    tracker->driver = drv;

    // Null the lighthouse pointer
    for (size_t i = 0; i < MAX_NUM_LIGHTHOUSES; i++)
      tracker->ootx[i].lighthouse = NULL;

    // Try and open the device
    ret = libusb_open(dev, &tracker->udev);
    if (ret || !tracker->udev)
    {
      if (ret == -3)
        printf("Error: the libusb_open function does not have correct permissions; check your rules.d folder \n");
      else
        printf("Error: the libusb_open function failed due to an error unrelated to permissions; return code %d \n", ret);
      goto fail;
    }


    // Set to auto-detatch
    libusb_set_auto_detach_kernel_driver(tracker->udev, 1);
    for (int j = 0; j < conf->bNumInterfaces; j++)
      if (libusb_claim_interface(tracker->udev, j))
        goto fail;

    // Get the serial number from the opened device handle
    ret = libusb_get_string_descriptor_ascii(tracker->udev,
      desc.iSerialNumber, (unsigned char*) tracker->serial, MAX_SERIAL_LENGTH);
    if (ret < 0)
      goto fail;

    // The tracker type is simply the USB product ID
    tracker->type = desc.idProduct;

    // What we do depends on the product
    printf("Probing device %u\n", did);
    switch (tracker->type) {
     // V2
     case USB_PROD_TRACKER_V2:
      // Endpoint for IMU
      tracker->endpoints[0].tracker = tracker;
      tracker->endpoints[0].type = DATA_IMU;
      tracker->endpoints[0].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[0].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[0].tx, tracker->udev,
        USB_ENDPOINT_V2_GENERAL, tracker->endpoints[0].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[0], 0);
      ret = libusb_submit_transfer(tracker->endpoints[0].tx);
      if (ret) {
        printf("- Error: libusb_submit_transfer (imu)\n");
        goto fail;
      }
      // Endpoint for light
      tracker->endpoints[1].tracker = tracker;
      tracker->endpoints[1].type = DATA_LIGHT_9;
      tracker->endpoints[1].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[1].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[1].tx, tracker->udev,
        USB_ENDPOINT_V2_LIGHT, tracker->endpoints[1].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[1], 0);
      ret = libusb_submit_transfer(tracker->endpoints[1].tx);
      if (ret) {
        printf("- Error: libusb_submit_transfer (light)\n");
        goto fail;
      }
      // Endpoint for buttons
      tracker->endpoints[2].tracker = tracker;
      tracker->endpoints[2].type = DATA_BUTTONS;
      tracker->endpoints[2].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[2].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[2].tx, tracker->udev,
        USB_ENDPOINT_V2_BUTTONS, tracker->endpoints[2].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[2], 0);
      ret = libusb_submit_transfer(tracker->endpoints[2].tx);
      if(ret) {
        printf("- Error: libusb_submit_transfer (buttons)\n");
        goto fail;
      }
      // Send a magic code to power on the tracker
      if (update_feature_report(tracker->udev, 0, magic_code_power_en_,
        sizeof(magic_code_power_en_)) != sizeof(magic_code_power_en_))
          printf("Power on failed\n");
      else
        printf("- Power on success\n");
      // Get the configuration for this device
      ret = get_config(tracker, 0, CONFIG_VERSION_2);
      if (ret < 0) {
        printf("- Calibration cannot be pulled. Ignoring.\n");
        goto fail;
      }
      printf("- Found V2 tracker %s connected by USB\n", tracker->serial);
      break;
     // V1
     case USB_PROD_CONTROLLER_V1:
     case USB_PROD_TRACKER_V1:
      // Endpoint for IMU
      tracker->endpoints[0].tracker = tracker;
      tracker->endpoints[0].type = DATA_IMU;
      tracker->endpoints[0].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[0].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[0].tx, tracker->udev,
        USB_ENDPOINT_V1_GENERAL, tracker->endpoints[0].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[0], 0);
      ret = libusb_submit_transfer(tracker->endpoints[0].tx);
      if (ret) {
        printf("error: libusb_submit_transfer (imu)\n");
        goto fail;
      }
      // Endpoint for light
      tracker->endpoints[1].tracker = tracker;
      tracker->endpoints[1].type = DATA_LIGHT_7;
      tracker->endpoints[1].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[1].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[1].tx, tracker->udev,
        USB_ENDPOINT_V1_LIGHT, tracker->endpoints[1].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[1], 0);
      ret = libusb_submit_transfer(tracker->endpoints[1].tx);
      if (ret) {
        printf("- Error: libusb_submit_transfer (light)\n");
        goto fail;
      }
      // Endpoint for buttons
      tracker->endpoints[2].tracker = tracker;
      tracker->endpoints[2].type = DATA_BUTTONS;
      tracker->endpoints[2].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[2].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[2].tx, tracker->udev,
        USB_ENDPOINT_V1_BUTTONS, tracker->endpoints[2].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[2], 0);
      ret = libusb_submit_transfer(tracker->endpoints[2].tx);
      if(ret) {
        printf("- Error: libusb_submit_transfer (buttons)\n");
        goto fail;
      }
      // Send a magic code to power on the tracker
      if (update_feature_report(tracker->udev, 0, magic_code_power_en_,
        sizeof(magic_code_power_en_)) != sizeof(magic_code_power_en_))
          printf("- Power on failed\n");
      else
        printf("- Power on success\n");
      // Get the configuration for this device
      ret = get_config(tracker, 0, CONFIG_VERSION_1);
      if (ret < 0) {
        printf("- Calibration cannot be pulled. Ignoring.\n");
        goto fail;
      }
      printf(" - Found V1 tracker %s connected by USB\n", tracker->serial);
      break;
     ///////////////////////
     // WIRELESS WATCHMAN //
     ///////////////////////
     case USB_PROD_WATCHMAN:
      // Set up the interrupts
      tracker->endpoints[0].tracker = tracker;
      tracker->endpoints[0].type = DATA_WATCHMAN;
      tracker->endpoints[0].tx = libusb_alloc_transfer(0);
      if (!tracker->endpoints[0].tx)
        goto fail;
      libusb_fill_interrupt_transfer(tracker->endpoints[0].tx, tracker->udev,
        USB_ENDPOINT_WATCHMAN, tracker->endpoints[0].buffer,
        USB_INT_BUFF_LENGTH, interrupt_handler, &tracker->endpoints[0], 0);
      ret = libusb_submit_transfer(tracker->endpoints[0].tx);
      if (ret)
        goto fail;
      // Get the configuration for this device
      ret = get_config(tracker, 1, CONFIG_VERSION_1);
      if (ret < 0) {
        printf("- Calibration cannot be pulled. Ignoring.\n");
        goto fail;
      }
      printf("- Found watchman radio connected to tracker %s\n", tracker->serial);
      break;
      // UNKNOWN
      default:
       continue;
    }
    // Add the tracker to the dynamic list of trackers
    drv->trackers[drv->num_trackers++] = tracker;
    continue;

    // Catch-all to prevent memory leaks
fail:
     free(tracker);
  }

  // Free the device list
  libusb_free_device_list(devs, 1);

  // Success
  return drv->num_trackers;
}
