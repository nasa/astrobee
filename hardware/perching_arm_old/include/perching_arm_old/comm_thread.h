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


#ifndef PERCHING_ARM_OLD_COMM_THREAD_H_
#define PERCHING_ARM_OLD_COMM_THREAD_H_

#include <unistd.h>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include "perching_arm_old/serial_comm.h"

// #define ASTROBEE_P4C

template<class _T, unsigned int _N>
inline unsigned char NTH_BYTE(_T __x) {
  return ((__x >> (8 * _N)) & 0xff);
}
template<class _T>
inline unsigned char highbyte(_T __x) {
  return NTH_BYTE<_T, 1>(__x);
}
template<class _T>
inline unsigned char lowbyte(_T __x) {
  return NTH_BYTE<_T, 0>(__x);
}

class CommThread {
 public:
  CommThread();
  void serial_init();
  void send_command(int target, int command, int data);
  int receive_feedback(unsigned char* buf, int size);
  uint8_t calculate_checksum(unsigned char* buf, int size);
  void set_DTR(int command);

 private:
  unsigned int baudRate;
  boost::asio::io_service io_;

  boost::scoped_ptr<boost::thread> serialThreadPAC_;
  boost::scoped_ptr<SerialComm> serialPAC_;
};

#endif  // PERCHING_ARM_OLD_COMM_THREAD_H_

