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

#ifndef EPSON_IMU_GPIO_H_
#define EPSON_IMU_GPIO_H_

#include <sys/time.h>
#include <sys/types.h>

#include <iostream>
#include <string>

namespace gpio {

enum Direction {
  IN = 0,
  OUT = 1
};

enum Edge {
  NONE = 0,
  RISING = 1,
  FALLING = 2,
  BOTH = 3
};

enum Value {
  UNDEFINED = -1,
  LOW = 0,
  HIGH = 1
};

enum Interrupt {
  FAILED = -1,
  TIMEOUT = 0,
  SUCCESS = 1
};

class GPIO {
 public:
  explicit GPIO(int gpio);
  ~GPIO(void);

  bool IsExported(void);
  bool Export(void);
  bool Unexport(void);
  bool SetDirection(Direction direction);
  bool SetEdge(Edge edge);
  Value GetValue(void);
  bool SetValue(Value value);
  Interrupt WaitInterrupt(const struct timespec *timeout);

 private:
  int gpio_;
  int fd_value_;
  std::string sysfs_gpio_path_;

 private:
  bool OpenValue(int oflags);
  void CloseValue(void);
};

}  // namespace gpio

#endif  // EPSON_IMU_GPIO_H_
