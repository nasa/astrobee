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


#ifndef LASER_LASER_H_
#define LASER_LASER_H_

#include <i2c/i2c_new.h>

namespace laser {

class Laser {
 public:
  explicit Laser(i2c::Device const& dev);

  bool Init(i2c::Error* ec);

  bool SetEnabled(bool enabled, i2c::Error* ec);

  bool enabled() { return enabled_; }

 private:
  i2c::Device dev_;
  bool enabled_;
};

}  // namespace laser

#endif  // LASER_LASER_H_
