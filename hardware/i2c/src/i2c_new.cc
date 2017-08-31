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

#include "i2c/i2c_new.h"

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <glog/logging.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <unordered_map>
#include <utility>

namespace {

using BusMap = std::unordered_map<std::string, i2c::Bus::WeakPtr>;
using LockGuard = std::lock_guard<std::mutex>;

}  // end namespace

i2c::Bus::Ptr i2c::Open(std::string const& dev, i2c::Error* err) {
  static BusMap s_map;

  LOG(INFO) << "request for bus: " << dev;

  BusMap::iterator itr = s_map.find(dev);
  if (itr != s_map.end()) {
    i2c::Bus::Ptr bus = itr->second.lock();
    if (bus) {
      LOG(INFO) << "returning existing weak_ptr for " << dev;
      return bus;
    }

    LOG(INFO) << "pruning dead weak_ptr for " << dev;
    s_map.erase(itr);
  }

  LOG(INFO) << "no bus, opening: " << dev;

  int fd = open(dev.data(), O_RDWR);
  if (fd < 0) {
    *err = errno;
    return i2c::Bus::Ptr{};
  }

  i2c::Bus::Ptr bus = i2c::Bus::Ptr(new Bus(fd));
  s_map.emplace(dev, bus);
  bus->wptr_ = bus;  // this is ugly :(

  return bus;
}

i2c::Device i2c::Bus::DeviceAt(const Address addr) const noexcept {
  return Device(wptr_.lock(), addr);
}

i2c::Bus::Bus(int fd)
  : fd_(fd) {
  LOG(INFO) << "creating i2c bus with fd " << fd_;
}

i2c::Bus::~Bus() {
  LOG(INFO) << "closing i2c bus with fd " << fd_;
  close(fd_);
}

int i2c::Bus::Write(const Address addr, const std::uint8_t* data,
                    const std::size_t size, Error* ec) {
  struct i2c_msg msg {
    .addr  = addr,
    .flags = 0,
    .len   = static_cast<uint16_t>(size & 0xFFFF),
    .buf   = const_cast<std::uint8_t *>(data)
  };
  struct i2c_rdwr_ioctl_data pkt {
    .msgs  = &msg,
    .nmsgs = 1
  };

  LockGuard lock(mtx_);
  int ret = ioctl(fd_, I2C_RDWR, &pkt);
  if (ret < 0) {
    *ec = errno;
    return -1;
  }

  return size;
}

int i2c::Bus::Read(const Address addr, std::uint8_t *data,
                   const std::size_t size, Error* ec) {
  struct i2c_msg msg {
    .addr  = addr,
    .flags = I2C_M_RD,
    .len   = static_cast<uint16_t>(size & 0xFFFF),
    .buf   = data
  };
  struct i2c_rdwr_ioctl_data pkt {
    .msgs = &msg,
    .nmsgs = 1
  };

  LockGuard lock(mtx_);
  int ret = ioctl(fd_, I2C_RDWR, &pkt);
  if (ret < 0) {
    *ec = errno;
    return -1;
  }

  return size;
}

int i2c::Bus::ReadRegister(const Address addr, const std::uint8_t reg,
                           std::uint8_t *data, const std::size_t size,
                           Error *ec) {
  struct i2c_msg msgs[] {
    {
      .addr  = addr,
      .flags = 0,
      .len   = 1,
      .buf   = const_cast<uint8_t*>(&reg)
    }, {
      .addr  = addr,
      .flags = I2C_M_RD,
      .len   = static_cast<uint16_t>(size & 0xFFFF),
      .buf   = data
    }
  };

  struct i2c_rdwr_ioctl_data pkt {
    .msgs = msgs,
    .nmsgs = 2
  };

  LockGuard lock(mtx_);

  int ret = ioctl(fd_, I2C_RDWR, &pkt);
  if (ret < 0) {
    *ec = ret;
    return -1;
  }

  return size;
}

void i2c::Bus::SetRetries(const int retries) noexcept {
  LockGuard lock(mtx_);
  std::uintptr_t temp = retries;
  int ret = ioctl(fd_, I2C_RETRIES, temp);
  if (ret < 0)
    LOG(ERROR) << "ioctl I2C_RETRIES failed: " << ret;
}

i2c::Device::Device(i2c::Bus::Ptr bus, const i2c::Address addr)
  : addr_(addr), bus_(std::move(bus)) {
}

i2c::Device::operator bool() const noexcept {
  return ((addr_ != -1) && bus_);
}

i2c::Address i2c::Device::addr() const noexcept {
  return addr_;
}

const i2c::Bus::Ptr& i2c::Device::bus() const noexcept {
  return bus_;
}

int i2c::Device::Write(const std::uint8_t* data, const std::size_t size,
                       Error* ec) {
  return bus_->Write(addr_, data, size, ec);
}

int i2c::Device::Write(const std::uint8_t data, Error* ec) {
  return bus_->Write(addr_, &data, 1, ec);
}

int i2c::Device::WriteRegister(const std::uint8_t reg, const std::uint8_t data,
                               Error *ec) {
  const std::uint8_t temp[2] { reg, data };
  return Write(temp, 2, ec);
}

int i2c::Device::WriteRegister(const std::uint8_t reg, const std::uint8_t* data,
                               const std::size_t size, Error* ec) {
  // prevent useless heap allocations for small-ish transfers
  static constexpr std::size_t scratch_size = 128;
  static uint8_t scratch[scratch_size];
  static std::mutex scratch_mtx;

  uint8_t* buf = scratch;

  if (size > scratch_size - 1) {
    buf = new(std::nothrow) std::uint8_t[size + 1];
    if (buf == nullptr) {
      *ec = ENOMEM;
      return -1;
    }
  } else {
    scratch_mtx.lock();
  }

  buf[0] = reg;
  std::memmove(buf+1, data, size);

  int ret = bus_->Write(addr_, buf, size+1, ec);
  if (buf != scratch)
    delete buf;
  else
    scratch_mtx.unlock();

  return ret;
}

int i2c::Device::Read(std::uint8_t* data, const std::size_t size, Error* ec) {
  return bus_->Read(addr_, data, size, ec);
}

int i2c::Device::ReadRegister(const std::uint8_t reg, std::uint8_t *data,
                              const std::size_t size, Error* ec) {
  return bus_->ReadRegister(addr_, reg, data, size, ec);
}
