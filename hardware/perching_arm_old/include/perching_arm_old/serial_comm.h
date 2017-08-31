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


#ifndef PERCHING_ARM_OLD_SERIAL_COMM_H_
#define PERCHING_ARM_OLD_SERIAL_COMM_H_

#include <boost/asio.hpp>

#include <iostream>
#include <string>

class SerialComm {
 public:
  SerialComm(boost::asio::io_service* io,
             const unsigned int baud_rate,
             const std::string& port);
  ~SerialComm(void);

  void start();
  void write(unsigned char* data, int length);
  int read(unsigned char* data, int readLength);
  int read_all(unsigned char* data, int readLength);
  int native_handle();
  bool active() const;

 private:
  const unsigned int m_baud_rate;
  bool active_;
  boost::asio::serial_port serial_;
};

#endif  // PERCHING_ARM_OLD_SERIAL_COMM_H_

