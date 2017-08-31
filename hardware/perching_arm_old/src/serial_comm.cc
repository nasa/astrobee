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


#include <boost/asio.hpp>
#include <iostream>

#include "perching_arm/serial_comm.h"

SerialComm::SerialComm(boost::asio::io_service* io,
                       const unsigned int baud_rate,
                       const std::string& port)
  : m_baud_rate(baud_rate), active_(false), serial_(*io) {
  serial_.open(port);
  if (serial_.is_open()) {
    active_ = true;
    std::cout << "serial open success!" << std::endl;
    start();
  } else {
    std::cout << "serial fail to open!" << std::endl;
  }
}

SerialComm::~SerialComm(void) {
  serial_.close();
  active_ = false;
  std::cout << "serial close" << std::endl;
}

void SerialComm::start() {
  serial_.set_option(boost::asio::serial_port::baud_rate(m_baud_rate));
  serial_.set_option(boost::asio::serial_port::flow_control());
  serial_.set_option(boost::asio::serial_port::stop_bits());
  serial_.set_option(boost::asio::serial_port::character_size());
  serial_.set_option(boost::asio::serial_port::parity());
}

void SerialComm::write(unsigned char* data, int length) {
  serial_.write_some(boost::asio::buffer(data, length));
}

int SerialComm::read(unsigned char* data, int readLength) {
  return boost::asio::read(serial_, boost::asio::buffer(data, readLength));
  // return serial_.read(boost::asio::buffer(data, readLength));
}

int SerialComm::read_all(unsigned char* data, int readLength) {
  int total = 0, length;

  while (total < readLength) {
    length = read(data, readLength - total);
    data += length;
    total += length;
  }
  return total;
}

int SerialComm::native_handle() {
  return serial_.native_handle();
}

bool SerialComm::active() const {
  return active_;
}

