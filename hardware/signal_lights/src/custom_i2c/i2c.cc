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

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <custom_i2c/i2c.h>

namespace custom_i2c {

int i2c_set_retries(int handle, int retries) {
  return ioctl(handle, I2C_RETRIES, retries);
}

int i2c_set_timeout(int handle, int timeout) {
  return ioctl(handle, I2C_TIMEOUT, timeout);
}

int i2c_open(const char* bus, uint8_t addr) {
  int file;

  if ((file = open(bus, O_RDWR)) < 0) {
    fprintf(stderr, "i2c_open open error: %s\n", strerror(errno));
    return file;
  }

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    fprintf(stderr, "i2c_open ioctl error: %s\n", strerror(errno));
    return -1;
  }
  return file;
}

int i2c_write(int handle, uint8_t* buf, unsigned int length) {
  if (write(handle, buf, length) != static_cast<int>(length)) {
    fprintf(stderr, "i2c_write error: %s\n", strerror(errno));
    return -1;
  }
  return length;
}

int i2c_write_byte(int handle, uint8_t val) {
  if (write(handle, &val, 1) != 1) {
    fprintf(stderr, "i2c_write_byte error: %s\n", strerror(errno));
    return -1;
  }

  return 1;
}

int i2c_read(int handle, uint8_t* buf, unsigned int length) {
  if (read(handle, buf, length) != static_cast<int>(length)) {
    fprintf(stderr, "i2c_read error: %s\n", strerror(errno));
    return -1;
  }
  return length;
}

int i2c_read_byte(int handle, uint8_t* val) {
  if (read(handle, val, 1) != 1) {
    fprintf(stderr, "i2c_read_byte error: %s\n", strerror(errno));
    return -1;
  }

  return 1;
}

int i2c_close(int handle) {
  if ((close(handle)) != 0) {
    fprintf(stderr, "i2c_close error: %s\n", strerror(errno));
    return -1;
  }

  return 0;
}

// Do combined read/write transaction without stop in between.
int i2c_write_read(int handle, uint8_t addr_w, uint8_t* buf_w,
                   unsigned int len_w, uint8_t addr_r, uint8_t* buf_r,
                   unsigned int len_r) {
  struct i2c_rdwr_ioctl_data msgset;
  struct i2c_msg msgs[2];

  msgs[0].addr = addr_w;
  msgs[0].len = len_w;
  msgs[0].flags = 0;
  msgs[0].buf = buf_w;

  msgs[1].addr = addr_r;
  msgs[1].len = len_r;
  msgs[1].flags = I2C_M_RD;
  msgs[1].buf = buf_r;

  msgset.nmsgs = 2;
  msgset.msgs = msgs;

  if (ioctl(handle, I2C_RDWR, (uint64_t)&msgset) < 0) {
    fprintf(stderr, "i2c_write_read error: %s\n", strerror(errno));
    return -1;
  }

  return len_r;
}

int i2c_write_ignore_nack(int handle, uint8_t addr_w, uint8_t* buf,
                          unsigned int length) {
  struct i2c_rdwr_ioctl_data msgset;
  struct i2c_msg msgs[1];

  msgs[0].addr = addr_w;
  msgs[0].len = length;
  msgs[0].flags = 0 | I2C_M_IGNORE_NAK;
  msgs[0].buf = buf;

  msgset.nmsgs = 1;
  msgset.msgs = msgs;

  if (ioctl(handle, I2C_RDWR, (uint64_t)&msgset) < 0) {
    fprintf(stderr, "i2c_write_ignore_nack error: %s\n", strerror(errno));
    return -1;
  }
  return (length);
}

int i2c_read_no_ack(int handle, uint8_t addr_r, uint8_t* buf,
                    unsigned int length) {
  struct i2c_rdwr_ioctl_data msgset;
  struct i2c_msg msgs[1];

  msgs[0].addr = addr_r;
  msgs[0].len = length;
  msgs[0].flags = I2C_M_RD | I2C_M_NO_RD_ACK;
  msgs[0].buf = buf;

  msgset.nmsgs = 1;
  msgset.msgs = msgs;

  if (ioctl(handle, I2C_RDWR, (uint64_t)&msgset) < 0) {
    fprintf(stderr, "i2c_read_no_ack error: %s\n", strerror(errno));
    return -1;
  }
  return (length);
}

int delay_ms(unsigned int msec) {
  int ret;
  struct timespec t;

  if (msec > 999) {
    fprintf(stderr, "delay_ms error: delay value needs to be less than 999\n");
    msec = 999;
  }

  t.tv_nsec = ((int32_t)(msec)) * 1E6d;
  t.tv_sec = 0;

  if ((ret = nanosleep(&t, NULL)) != 0) {
    fprintf(stderr, "delay_ms error: %s\n", strerror(errno));
  }

  return 0;
}

}  // namespace custom_i2c
