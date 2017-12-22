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

#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>

// STL includes
#include <functional>
#include <vector>
#include <string>

// Default read buffer size
#define MAX_BUFFER_SIZE 512
#define MAX_READ_ERRORS 10
#define READ_TIMEOUT_MS 1000

// Serial read callback signature
typedef std::function<void(const uint8_t*, size_t)> SerialReadCallback;
typedef std::function<void(void)> SerialTimeoutCallback;
typedef std::function<void(void)> SerialShutdownCallback;

namespace serial {

enum SerialResetPinState{
  RESET_PIN_LOW  = 0,
  RESET_PIN_HIGH = 1
};

// Provides asynchronous serial access
class Serial : private boost::noncopyable {
 public:
  // Constructor flags not ready
  explicit Serial(SerialReadCallback cb_read);

  // Set a callback when a read timeout occurs
  void SetTimeoutCallback(SerialTimeoutCallback cb, uint32_t ms = 1000);

  // Set a callback when a shutdown of the connection occurs (EOF)
  void SetShutdownCallback(SerialShutdownCallback cb);

  // Destructor
  virtual ~Serial();

  // Open a serial port
  bool Open(const std::string& port_name, unsigned int baud_rate,
    boost::asio::serial_port_base::parity option_parity =
        boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none),
    boost::asio::serial_port_base::character_size option_csize =
        boost::asio::serial_port_base::character_size(8),
    boost::asio::serial_port_base::flow_control option_flow =
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none),
    boost::asio::serial_port_base::stop_bits option_stop =
        boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));

  // Check if serial is open
  bool IsOpen();

  // Close a serial port
  void Close();

  // Write some data to the serial port
  size_t Write(const uint8_t *data, size_t size);

  // Set reset pint state
  void SetResetPinState(SerialResetPinState state);

 protected:
  // Start a read action (timeout = 0 implies no timeout)
  void ReadStart();

  // Stop a read action
  void ReadStop(const boost::system::error_code& error, size_t bytes);

  // Timeout on read
  void TimeoutCallback(const boost::system::error_code& error);

 private:
  boost::asio::io_service asio_;                       // ASIO service
  boost::asio::serial_port port_;                      // Serial port
  boost::asio::deadline_timer timer_;                  // Timeout
  boost::thread thread_;                               // Worker thread
  SerialReadCallback cb_read_;                         // Read callback
  SerialTimeoutCallback cb_timeout_;                   // Timeout callback
  SerialShutdownCallback cb_shutdown_;                 // Shutdown callback
  uint8_t buffer_[MAX_BUFFER_SIZE];                    // Data buffer
  uint32_t timeout_ms_;                                // Ms read timeout
  bool timeout_;                                       // Was there a timeout?
};

}  // namespace serial

#endif  // SERIAL_SERIAL_H_
