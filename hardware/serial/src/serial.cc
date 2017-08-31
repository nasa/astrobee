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

#include <serial/serial.h>

#include <sys/ioctl.h>

namespace serial {

// Default constructor
Serial::Serial(SerialReadCallback cb_read, SerialTimeoutCallback cb_timeout,
  uint32_t timeout_ms) : port_(asio_), timer_(asio_),
  cb_read_(cb_read), cb_timeout_(cb_timeout), timeout_ms_(timeout_ms) {}

// Destructor closes port safely
Serial::~Serial() {
  Close();
}

// Open a serial port
bool Serial::Open(const std::string& port_name, unsigned int baud_rate,
  boost::asio::serial_port_base::parity option_parity,
  boost::asio::serial_port_base::character_size option_csize,
  boost::asio::serial_port_base::flow_control option_flow,
  boost::asio::serial_port_base::stop_bits option_stop) {
  // Open the port with the given options
  port_.open(port_name);
  if (!port_.is_open())
    return false;
  // Set the serial port options
  port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_.set_option(option_parity);
  port_.set_option(option_csize);
  port_.set_option(option_flow);
  port_.set_option(option_stop);
  // Send work tot the ASIO service
  asio_.post(boost::bind(&Serial::ReadStart, this));
  // Start a background thread for running the ASIO service
  thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &asio_));
  // Success!
  return true;
}

// Check serial port is open
bool Serial::IsOpen() {
  return port_.is_open();
}

// Close a serial port
void Serial::Close() {
  if (!IsOpen()) return;
  asio_.post(boost::bind(&boost::asio::serial_port::close, &port_));
  asio_.stop();
  thread_.join();
}

// Initialize a read action
void Serial::ReadStart() {
  port_.async_read_some(boost::asio::buffer(buffer_, MAX_BUFFER_SIZE),
    boost::bind(&Serial::ReadStop, this, boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  // If we have specified a timeout
  if (timeout_ms_ > 0) {
    timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms_));
    timer_.async_wait(boost::bind(&Serial::TimeoutCallback, this, boost::asio::placeholders::error));
  }
}

// Sotp a read action
void Serial::ReadStop(const boost::system::error_code& error, size_t bytes_transferred) {
  if (error == boost::asio::error::operation_aborted && timeout_) {
    if (cb_timeout_)
      cb_timeout_();
    timeout_ = false;
  } else {
    timer_.cancel();
    if (!error && cb_read_)
      cb_read_(buffer_, bytes_transferred);
  }
  // Start another read
  ReadStart();
}

void Serial::TimeoutCallback(const boost::system::error_code& error) {
  if (!error) {
    port_.cancel();
    timeout_ = true;
  }
}

// Blocking write some data
size_t Serial::Write(const uint8_t *buff, size_t len) {
  if (!IsOpen())
    return 0;
  return boost::asio::write(port_, boost::asio::buffer(buff, len));
}

// Set reset pint state
void Serial::SetResetPinState(SerialResetPinState state) {
  int fd = port_.native_handle();
  if (fd < 0)
    return;
  int DTR_flag = TIOCM_DTR;
  switch (state) {
  case RESET_PIN_LOW:
    ioctl(fd, TIOCMBIC, &DTR_flag);
    break;
  case RESET_PIN_HIGH:
    ioctl(fd, TIOCMBIS, &DTR_flag);
    break;
  }
}

}  // namespace serial
