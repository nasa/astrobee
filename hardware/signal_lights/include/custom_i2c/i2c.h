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

#ifndef CUSTOM_I2C_I2C_H_
#define CUSTOM_I2C_I2C_H_

#include <stdint.h>

#endif  // CUSTOM_I2C_I2C_H_

namespace custom_i2c {
// bus=1 for interface I2C2 on BBB
// returns handle to be used in remainder functions
// addr is a 7-bit value (so, for example the BMP085 datasheet specifies
// 0xEE, so we need to right-shift by 1 and use 0x77 for this function)
int i2c_open(const char* bus, uint8_t addr);

int i2c_set_retries(int handle, int retries);
int i2c_set_timeout(int handle, int timeout);

// These functions return -1 on error, otherwise return the number of bytes
// read/written. To perform a 'repeated start' use the i2c_write_read function
// which can write some data and then immediately read data without a stop bit
// in between.
int i2c_write(int handle, uint8_t* buf, unsigned int length);
int i2c_read(int handle, uint8_t* buf, unsigned int length);
int i2c_write_read(int handle, uint8_t addr_w, uint8_t* buf_w,
                   unsigned int len_w, uint8_t addr_r, uint8_t* buf_r,
                   unsigned int len_r);
int i2c_write_ignore_nack(int handle, uint8_t addr_w, uint8_t* buf,
                          unsigned int length);
int i2c_read_no_ack(int handle, uint8_t addr_r, uint8_t* buf,
                    unsigned int length);
int i2c_write_byte(int handle, uint8_t val);
int i2c_read_byte(int handle, uint8_t* val);

// These functions return -1 on error, otherwise return 0 on success
int i2c_close(int handle);
// Provides an inaccurate delay (may be useful for waiting for ADC etc).
// The maximum delay is 999msec
int delay_ms(unsigned int msec);

}  // namespace custom_i2c
