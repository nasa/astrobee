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

#ifndef I2C_I2C_H_
#define I2C_I2C_H_

#include <map>
#include <string>
#include <mutex>

namespace I2c {
class I2c {
 public:
  static bool Open(std::string const& device);
  static bool Close(std::string const& device);
  static int16_t WriteI2c(std::string const& device, const uint8_t addr,
                       const uint8_t& data);
  static int16_t WriteI2c(std::string const& device, const uint8_t addr,
                       uint8_t* data, const size_t len);
  static int16_t ReadI2c(std::string const& device, const uint8_t addr,
                      uint8_t *buf, const int size);
  static int16_t WriteSmb(std::string const& device, const uint8_t addr,
                       const uint8_t cmd, const uint8_t& data);
  static int16_t WriteSmb(std::string const& device, const uint8_t addr,
                       const uint8_t cmd, const uint16_t& data);
  static ssize_t WriteSmb(std::string const& device, const uint8_t addr,
                          const uint8_t cmd, const size_t len, const uint8_t* data);
  static int16_t ReadSmb(std::string const& device, const uint8_t addr,
                      const uint8_t cmd, uint8_t* data);
  static int16_t ReadSmb(std::string const& device, const uint8_t addr,
                      const uint8_t cmd, uint16_t* data);
  static int16_t ReadSmb(std::string const& device, const uint8_t addr,
                      const uint8_t cmd, const size_t len,
                      uint8_t* data);

 private:
  static I2c& Instance();
  static I2c* instance_;
  I2c();
  ~I2c();
  I2c(const I2c&) = delete;
  I2c & operator=(const I2c&) = delete;
  static std::map<std::string, std::mutex> mtx_map_;
  static std::map<std::string, int> fd_map_;
};
}  // end namespace I2c
#endif  // I2C_I2C_H_
