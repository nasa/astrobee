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

#include <i2c/i2c.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>

#define MAX_RETRANSMIT_TRIAL 3
#define DELAY_RETRANSMIT_US 1000

// This is a static I2c library to help with multiple programs accessing
// the same i2c bus.  It supports both i2c and smbus.

namespace I2c {

I2c::I2c() {
}

I2c::~I2c() {
  delete instance_;
  instance_ = NULL;
}

I2c& I2c::Instance() {
  if (!instance_) {
    instance_ = new I2c;
  }
  return *instance_;
}

bool I2c::Open(std::string const& device) {
  if (fd_map_[device] > 0)
    return true;

  fd_map_[device] = open(device.c_str(), O_RDWR);
  if (fd_map_[device] < 0)
    return false;
  else
    return true;
}

bool I2c::Close(std::string const& device) {
  if (fd_map_[device] < 0)
    return true;
  mtx_map_[device].lock();
  if (close(fd_map_[device]) < 0) {
    mtx_map_[device].unlock();
    return false;
  } else {
    mtx_map_[device].unlock();
    return true;
  }
}

int16_t I2c::ReadI2c(std::string const& device, const uint8_t addr,
                  uint8_t *buf, const int size) {
  struct i2c_rdwr_ioctl_data in_packets;
  struct i2c_msg in_messages[1];

  if (fd_map_[device] < 0)
    return 0;

  /* The data will get returned in this structure */
  in_messages[0].addr  = addr;
  in_messages[0].flags = I2C_M_RD;  // | I2C_M_NOSTART;
  in_messages[0].len   = size;
  in_messages[0].buf   = buf;

  /* Send the request to the kernel and get the result back */
  in_packets.msgs      = in_messages;
  in_packets.nmsgs     = 1;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_RDWR, &in_packets) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      // Data sent successfully.
      mtx_map_[device].unlock();
      return in_messages[0].len;
    }
  }

  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::WriteI2c(std::string const& device, const uint8_t addr,
                   const uint8_t &data) {
  uint8_t out_buf[1];
  struct i2c_rdwr_ioctl_data out_packets;
  struct i2c_msg out_messages[1];

  if (fd_map_[device] < 0)
    return 0;

  out_messages[0].addr = addr;
  out_messages[0].flags = 0;
  out_messages[0].len = sizeof(out_buf);
  out_messages[0].buf = out_buf;
  out_buf[0] = data;

  // Transfer the i2c packets to the kernel and verify it worked
  out_packets.msgs = out_messages;
  out_packets.nmsgs = 1;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_RDWR, &out_packets) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      return out_messages[0].len;
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::WriteI2c(std::string const& device, const uint8_t addr,
                   uint8_t *data, const size_t len) {
  struct i2c_rdwr_ioctl_data out_packets;
  struct i2c_msg out_messages[1];

  if (fd_map_[device] < 0)
    return 0;

  out_messages[0].addr = addr;
  out_messages[0].flags = 0;
  out_messages[0].len = len;
  out_messages[0].buf = data;

  // Transfer the i2c packets to the kernel and verify it worked
  out_packets.msgs = out_messages;
  out_packets.nmsgs = 1;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_RDWR, &out_packets) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      return out_messages[0].len;
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::WriteSmb(std::string const& device, const uint8_t addr,
                   const uint8_t cmd, const uint8_t& data) {
  if (fd_map_[device] < 0)
    return 0;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return 0;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_WRITE;
  packet.command = (__u8)cmd;
  packet.data->byte = data;
  packet.size = I2C_SMBUS_BYTE_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      return sizeof(uint8_t);
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::WriteSmb(std::string const& device, const uint8_t addr,
                   const uint8_t cmd, const uint16_t& data) {
  if (fd_map_[device] < 0)
    return 0;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return 0;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_WRITE;
  packet.command = (__u8)cmd;
  packet.data->word = data;
  packet.size = I2C_SMBUS_WORD_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      return sizeof(uint16_t);
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

ssize_t I2c::WriteSmb(std::string const& device, const uint8_t addr,
                      const uint8_t cmd, const size_t len,
                      const uint8_t* data) {
  if (fd_map_[device] < 0)
    return -1;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return -1;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_WRITE;
  packet.command = (__u8)cmd;

  size_t wlen = std::min(static_cast<size_t>(I2C_SMBUS_BLOCK_MAX), len);
  for (size_t i = 1; i <= wlen; i++) {
    packet.data->block[i] = data[i - 1];
  }
  packet.data->block[0] = wlen;
  packet.size = I2C_SMBUS_BLOCK_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      return wlen;
    }
  }
  mtx_map_[device].unlock();
  return -1;
}

int16_t I2c::ReadSmb(std::string const& device, const uint8_t addr,
                  const uint8_t cmd, uint8_t* data) {
  if (fd_map_[device] < 0)
    return 0;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return 0;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_READ;
  packet.command = (__u8)cmd;
  packet.size = I2C_SMBUS_BYTE_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      *data = packet.data->byte;
      return sizeof(uint8_t);
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::ReadSmb(std::string const& device, const uint8_t addr,
                  const uint8_t cmd, uint16_t* data) {
  if (fd_map_[device] < 0)
    return 0;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return 0;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_READ;
  packet.command = (__u8)cmd;
  packet.size = I2C_SMBUS_WORD_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();
      *data = packet.data->word;
      return sizeof(uint16_t);
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

int16_t I2c::ReadSmb(std::string const& device, const uint8_t addr,
                  const uint8_t cmd, const size_t len,
                  uint8_t* data) {
  if (fd_map_[device] < 0)
    return 0;

  mtx_map_[device].lock();

  if (ioctl(fd_map_[device], I2C_SLAVE, addr) < 0) {
    mtx_map_[device].unlock();
    return 0;
  }
  mtx_map_[device].unlock();

  struct i2c_smbus_ioctl_data packet;
  union i2c_smbus_data d;
  packet.data = &d;
  packet.read_write = I2C_SMBUS_READ;
  packet.command = (__u8)cmd;
  packet.size = I2C_SMBUS_BLOCK_DATA;

  mtx_map_[device].lock();
  for (int i = 0; i < MAX_RETRANSMIT_TRIAL; i++) {
    if (ioctl(fd_map_[device], I2C_SMBUS, &packet) < 0) {
      usleep(DELAY_RETRANSMIT_US);
    } else {
      mtx_map_[device].unlock();

      for (int j = 1; j <= packet.data->block[0]; j++)
        data[j-1] = packet.data->block[j];

      return packet.data->block[0];
    }
  }
  mtx_map_[device].unlock();
  return 0;
}

// statics
I2c* I2c::instance_ = NULL;
std::map<std::string, std::mutex> I2c::mtx_map_;
std::map<std::string, int> I2c::fd_map_;
}  // end namespace I2c

