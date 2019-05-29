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


#ifndef EPSON_IMU_EPSON_IMU_NODELET_H_
#define EPSON_IMU_EPSON_IMU_NODELET_H_

#include <pluginlib/class_list_macros.h>

#include <epson_imu/G362P.h>
#include <epson_imu/GPIO.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <ff_util/ff_nodelet.h>
#include <config_reader/config_reader.h>

#include <thread>
#include <string>

namespace epson_imu {

class EpsonImuNodelet : public ff_util::FreeFlyerNodelet {
 public:
  EpsonImuNodelet();
  ~EpsonImuNodelet();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);

 private:
  void ReadParams(void);
  void Run(void);

  bool InitGPIO(void);
  void CloseGPIO(void);
  bool InitSPI(void);
  void CloseSPI(void);
  void Exit();

  std::shared_ptr<std::thread> thread_;
  config_reader::ConfigReader config_;
  ros::Publisher pub_data_;

  int fd_spi_dev_;
  gpio::GPIO *gpio_data_ready_;
  gpio::GPIO *gpio_reset_;
  gpio::GPIO *gpio_cs_;

  // PARAMETERS
  int prototype_;
  std::string spi_dev_;

  // Gravitational acceleration.
  double G_;

  // JP-4 6
  int gpio_num_data_ready_;
  // JP-4 10
  int gpio_num_reset_;
  // Chip select. GPIO2_30
  int gpio_num_cs_;

  // SPI configuration.
  uint8_t spi_mode_;
  // The maximum clock speed of the IMU is 1 MHz.
  uint32_t spi_speed_hz_;
  // uint8_t spi_bits_;

  // Enable output.
  bool enable_flag_;
  bool enable_temp_;
  bool enable_gyro_;
  bool enable_accl_;
  bool enable_gpio_;
  bool enable_count_;
  bool enable_chksm_;

  // Data bits.
  bool data_32_bits_;

  // Sampling rate.
  epson_imu::SamplingRate sampling_rate_;

  // Filter setting.
  epson_imu::Filter filter_;

  // different rate in calibration mode
  bool calibration_mode_;
};

}  // namespace epson_imu

#endif  // EPSON_IMU_EPSON_IMU_NODELET_H_
