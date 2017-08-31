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

#ifndef I2C_I2C_NEW_H_
#define I2C_I2C_NEW_H_

#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

namespace i2c {

using Error = int;
using Address = std::uint16_t;

class Device;

// Represents an i2c bus.
// Holds an open file descriptor that is automatically closed on
// destruction (plus holds a mutex), thus non-copyable and non-movable.
// This class *should* be thread-safe.
class Bus {
 public:
  using Ptr = std::shared_ptr<Bus>;
  using WeakPtr = std::weak_ptr<Bus>;

  Bus(Bus const& other) = delete;
  Bus(Bus &&other) = delete;
  Bus& operator=(Bus const& other) = delete;
  Bus& operator=(Bus &&other) = delete;
  ~Bus();

  Device DeviceAt(const Address addr) const noexcept;

  int Write(const Address addr,
            const std::uint8_t* data, const std::size_t size,
            Error* ec);

  int Read(const Address addr,
           std::uint8_t *data, const std::size_t size,
           Error* ec);
  int ReadRegister(const Address addr, const std::uint8_t reg,
                   std::uint8_t *data, const std::size_t size,
                   Error *ec);

  void SetRetries(const int retries) noexcept;

 private:
  explicit Bus(int fd);

  bool SetAddress(const Address addr) noexcept;

  std::mutex mtx_;
  const int fd_;
  Address current_addr_ = -1;

  WeakPtr wptr_;

  friend Bus::Ptr Open(std::string const&, Error*);
};

Bus::Ptr Open(std::string const &dev, Error* err);

// Reperesents a device at a certain address on an i2c bus.
class Device {
 public:
  Device(Bus::Ptr bus, const Address addr);
  Device() = default;

  explicit operator bool() const noexcept;

  int Write(const std::uint8_t* data, const std::size_t size,
            Error* ec);
  int Write(const std::uint8_t data, Error* ec);

  int WriteRegister(const std::uint8_t reg, const std::uint8_t data,
                    Error* ec);
  int WriteRegister(const std::uint8_t reg,
                    const std::uint8_t *data, const std::size_t size,
                    Error* ec);

  int Read(std::uint8_t *data, const std::size_t size,
           Error* ec);
  int ReadRegister(const std::uint8_t reg,
                   std::uint8_t *data, const std::size_t size,
                   Error *ec);

  i2c::Address addr() const noexcept;

  const Bus::Ptr& bus() const noexcept;

 private:
  const Address addr_ = -1;
  const Bus::Ptr bus_;
};

}  // end namespace i2c

#endif  // I2C_I2C_NEW_H_
