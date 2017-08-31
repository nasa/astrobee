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


#include <unistd.h>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>

#include <sys/ioctl.h>

#include "perching_arm/comm_thread.h"

CommThread::CommThread() {
  serial_init();
}

void CommThread::serial_init() {
  baudRate = 115200;
  std::string port("/dev/ttyUSB0");
  std::cout << "Perching arm controller ";
  serialPAC_.reset(new SerialComm(&io_, baudRate, port));
  serialThreadPAC_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_)));
}

void CommThread::send_command(int target, int address, int data) {
#ifdef ASTROBEE_P4C
  unsigned char txBuffer[8] = {};

  txBuffer[0] = 0xff;
  txBuffer[1] = 0xff;
  txBuffer[2] = 5;
  txBuffer[3] = target;
  txBuffer[4] = address;
  txBuffer[5] = lowbyte(data);
  txBuffer[6] = highbyte(data);
  txBuffer[7] = calculate_checksum(txBuffer, sizeof(txBuffer));
#else
  unsigned char txBuffer[9] = {};

  txBuffer[0] = 0xff;
  txBuffer[1] = 0xff;
  txBuffer[2] = 6;
  txBuffer[3] = target;
  txBuffer[4] = lowbyte(address);
  txBuffer[5] = highbyte(address);
  txBuffer[6] = lowbyte(data);
  txBuffer[7] = highbyte(data);
  txBuffer[8] = calculate_checksum(txBuffer, sizeof(txBuffer));
#endif

  serialPAC_->write(txBuffer, sizeof(txBuffer));
}

int CommThread::receive_feedback(unsigned char* buf, int size) {
  unsigned char *p = buf;

  size = serialPAC_->read_all(p, 1);
  if (*p != 0xff)
    return -1;

  size += serialPAC_->read_all(++p, 1);
  if (*p != 0xff)
    return -1;

  size += serialPAC_->read_all(++p, 1);
  size += serialPAC_->read_all(buf+3, buf[2]);

  if (buf[size-1] != calculate_checksum(buf, size))
    return -1;

  return 0;
}

uint8_t CommThread::calculate_checksum(unsigned char* buf, int size) {
  uint8_t sum = 0, i;
  for (i = 2; i < size-1; i++)
    sum += buf[i];
  return ~sum;
}

void CommThread::set_DTR(int command) {
  int fd;
  fd = serialPAC_->native_handle();
  int DTR_flag = TIOCM_DTR;
  if (command == 1)
    ioctl(fd, TIOCMBIS, &DTR_flag);
  else
    ioctl(fd, TIOCMBIC, &DTR_flag);
}

