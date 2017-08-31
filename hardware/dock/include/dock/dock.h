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


#ifndef DOCK_DOCK_H_
#define DOCK_DOCK_H_

#include <i2c/i2c_new.h>

#include <cstdint>

namespace dock {

enum Bay : std::uint8_t {
  kBayOne = 0x01,
  kBayTwo = 0x02,
};

class Dock {
 public:
  explicit Dock(i2c::Device const &dock, i2c::Device const &loop);

  bool Init(i2c::Error *ec);
  bool Undock(Bay bay, i2c::Error *ec);
  bool Docked(bool *docked, i2c::Error *ec);

 private:
  i2c::Device dock_;
  i2c::Device loop_;
};

}  // namespace dock

#endif  // DOCK_DOCK_H_
