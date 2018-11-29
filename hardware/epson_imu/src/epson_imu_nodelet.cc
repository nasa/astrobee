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

#include <epson_imu/epson_imu_nodelet.h>

#include <sensor_msgs/Imu.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace epson_imu {

EpsonImuNodelet::EpsonImuNodelet() :
    ff_util::FreeFlyerNodelet(NODE_EPSON_IMU), spi_mode_(0)  {}

EpsonImuNodelet::~EpsonImuNodelet() {}

void EpsonImuNodelet::Initialize(ros::NodeHandle *nh) {
  pub_data_ = GetPlatformHandle(true)->advertise<sensor_msgs::Imu>(TOPIC_HARDWARE_IMU, 1);

  calibration_mode_ = false;
  if (GetName() == "calibration_imu") {
    calibration_mode_ = true;
  }

  // configuration files
  config_.AddFile("hw/epson_imu.config");
  ReadParams();

  // start a new thread to listen to IMU
  thread_.reset(new std::thread(&epson_imu::EpsonImuNodelet::Run, this));
}

// imu driver does not support parameter reloading!!!
void EpsonImuNodelet::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
  }

  config_reader::ConfigReader::Table imu(&config_, "epson_imu");
  if (!imu.GetInt("prototype", &prototype_)) {
    ROS_FATAL("IMU prototype not specified.");
  }
  if (!imu.GetStr("spi_dev", &spi_dev_)) {
    ROS_FATAL("IMU spi_dev not specified.");
  }
  if (!imu.GetReal("G", &G_)) {
    ROS_FATAL("IMU G not specified.");
  }
  if (!imu.GetUInt("spi_speed_hz", &spi_speed_hz_)) {
    ROS_FATAL("IMU spi_speed_hz not specified.");
  }
  if (!imu.GetInt("gpio_num_data_ready", &gpio_num_data_ready_)) {
    ROS_FATAL("IMU gpio_num_data_ready not specified.");
  }
  if (!imu.GetInt("gpio_num_reset", &gpio_num_reset_)) {
    ROS_FATAL("IMU gpio_num_reset not specified.");
  }
  if (!imu.GetInt("gpio_num_cs", &gpio_num_cs_)) {
    ROS_FATAL("IMU gpio_num_cs not specified.");
  }
  int temp = 0;
  if (!imu.GetInt("sampling_rate", &temp)) {
    ROS_FATAL("IMU sampling_rate not specified.");
  }
  sampling_rate_ = static_cast<epson_imu::SamplingRate>(temp);
  if (!imu.GetInt("filter", &temp)) {
    ROS_FATAL("IMU filter not specified.");
  }
  filter_ = static_cast<epson_imu::Filter>(temp);
  int calibration_filter, calibration_sampling_rate;
  if (!imu.GetInt("calibration_sampling_rate", &calibration_sampling_rate)) {
    ROS_FATAL("IMU calibration_sampling_rate not specified.");
  }
  if (!imu.GetInt("calibration_filter", &calibration_filter)) {
    ROS_FATAL("IMU filter not specified.");
  }
  if (!imu.GetBool("enable_flag", &enable_flag_)) {
    ROS_FATAL("IMU enable_flag not specified.");
  }
  if (!imu.GetBool("enable_temp", &enable_temp_)) {
    ROS_FATAL("IMU enable_temp not specified.");
  }
  if (!imu.GetBool("enable_gyro", &enable_gyro_)) {
    ROS_FATAL("IMU enable_gyro not specified.");
  }
  if (!imu.GetBool("enable_accl", &enable_accl_)) {
    ROS_FATAL("IMU enable_accl not specified.");
  }
  if (!imu.GetBool("enable_gpio", &enable_gpio_)) {
    ROS_FATAL("IMU enable_gpio not specified.");
  }
  if (!imu.GetBool("enable_count", &enable_count_)) {
    ROS_FATAL("IMU enable_count not specified.");
  }
  if (!imu.GetBool("enable_chksm", &enable_chksm_)) {
    ROS_FATAL("IMU enable_chksm not specified.");
  }
  if (!imu.GetBool("data_32_bits", &data_32_bits_)) {
    ROS_FATAL("IMU data_32_bits not specified.");
  }

  // validity checks
  // The minimum and maximum clock speeds of the IMU is 0.01 and 1 MHz.
  if (spi_speed_hz_ < 10000 || spi_speed_hz_ > 1000000) {
    ROS_FATAL("Unsupported spi clock speed (Hz): %d", spi_speed_hz_);
  }
  if (sampling_rate_ < 0x01 || sampling_rate_ > 0x07) {
    ROS_FATAL("Unsupported sampling rate (Hz): %d", sampling_rate_);
  }
  if (calibration_sampling_rate < 0x01 || calibration_sampling_rate > 0x07) {
    ROS_FATAL("Unsupported calibration sampling rate (Hz): %d", calibration_sampling_rate);
  }
  if (filter_ < 0x01 || filter_ > 0x13) {
    ROS_FATAL("Unsupported filter: %d", filter_);
  }
  if (calibration_filter < 0x01 || calibration_filter > 0x13) {
    ROS_FATAL("Unsupported calibration filter: %d", calibration_filter);
  }

  // override the rate if we are named "calibration_imu", this means
  // we are running the calibration launch file
  if (calibration_mode_) {
    sampling_rate_ = static_cast<epson_imu::SamplingRate>(calibration_sampling_rate);
    filter_ = static_cast<epson_imu::Filter>(calibration_filter);
  }
}

void EpsonImuNodelet::Run(void) {
  if (!InitGPIO()) {
    ROS_ERROR("Failed to initialize GPIO: %s", std::strerror(errno));
    Exit();
  }
  if (!InitSPI()) {
    ROS_ERROR("Failed to initialize SPI: %s", std::strerror(errno));
    Exit();
  }

  epson_imu::G362P imu(fd_spi_dev_, spi_speed_hz_, gpio_data_ready_, gpio_reset_);

  if (!imu.PowerOn()) {
    ROS_ERROR("Failed to power on the IMU: %s", std::strerror(errno));
    Exit();
  }

  imu.SetMode(epson_imu::Mode::CONFIGURATION);
  imu.SetFilter(filter_);
  imu.SetSamplingRate(sampling_rate_);
  // Disable UART, just in case.
  imu.EnableUART(false);
  // Enable output.
  imu.EnableOutput(enable_flag_, enable_temp_, enable_gyro_, enable_accl_,
    enable_gpio_, enable_count_, enable_chksm_);
  // Select the output bit length.
  imu.SetOutputBits(
    // temperature
    data_32_bits_ ? epson_imu::OutputBits::B32 : epson_imu::OutputBits::B16,
    // gyro
    data_32_bits_ ? epson_imu::OutputBits::B32 : epson_imu::OutputBits::B16,
    // accl
    data_32_bits_ ? epson_imu::OutputBits::B32 : epson_imu::OutputBits::B16);

  // Dump registers for debugging.
  // imu.DumpRegisters();

  // Go to the sampling mode.
  if (!imu.SetMode(epson_imu::Mode::SAMPLING)) {
    ROS_ERROR("Failed to change mode: %s", std::strerror(errno));
    Exit();
  }

  // Dummy read.
  // for (int i = 0; i < 500; i++) {
  //   epson_imu::Data epson_imu_data;
  //   imu.ReadData(&epson_imu_data);
  // }

  uint32_t seq = 0;

  epson_imu::Data epson_imu_data;
  sensor_msgs::Imu ros_imu_data;

  // Initialize data.
  // Header. Frame ID.
  ros_imu_data.header.frame_id = "epson_imu";

  // Orientation. Unknown.
  ros_imu_data.orientation.x = 0;
  ros_imu_data.orientation.y = 0;
  ros_imu_data.orientation.z = 0;
  ros_imu_data.orientation.w = 0;

  // Orientation covariance. Unknown.
  ros_imu_data.orientation_covariance = { -1, -1, -1, -1, -1, -1, -1, -1, -1 };

  // Angular velocity covariance. Unkown.
  ros_imu_data.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // Linear acceleration covariance. Unknown.
  ros_imu_data.linear_acceleration_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  while (ros::ok()) {
    if (imu.ReadData(&epson_imu_data)) {
      // Header.
      ros_imu_data.header.seq = seq++;
      ros_imu_data.header.stamp = ros::Time::now();

      // Angular velocity (deg/sec --> rad/sec)
      ros_imu_data.angular_velocity.x = epson_imu_data.gyro_[0] * M_PI / 180.0;
      ros_imu_data.angular_velocity.y = epson_imu_data.gyro_[1] * M_PI / 180.0;
      ros_imu_data.angular_velocity.z = epson_imu_data.gyro_[2] * M_PI / 180.0;

      // Linear acceleration (mG --> m/s^2)
      ros_imu_data.linear_acceleration.x = epson_imu_data.accl_[0] * G_ / 1000.0;
      ros_imu_data.linear_acceleration.y = epson_imu_data.accl_[1] * G_ / 1000.0;
      ros_imu_data.linear_acceleration.z = epson_imu_data.accl_[2] * G_ / 1000.0;

      pub_data_.publish(ros_imu_data);
    } else {
      ROS_WARN("Error while reading IMU data. Skip publishing.");
    }
  }

  // Return to the configuration mode.
  if (!imu.SetMode(epson_imu::Mode::CONFIGURATION)) {
    ROS_ERROR("Failed to change mode: %s", std::strerror(errno));
  }

  Exit();
}

bool EpsonImuNodelet::InitGPIO(void) {
  if (
    ((gpio_data_ready_ = new gpio::GPIO(gpio_num_data_ready_)) == NULL) ||
    ((gpio_reset_ = new gpio::GPIO(gpio_num_reset_)) == NULL) ||
    ((gpio_cs_ = new gpio::GPIO(gpio_num_cs_)) == NULL)) {
    ROS_ERROR("Failed to create a gpio::GPIO instance: %s", std::strerror(errno));
    return false;
  }

  if (!gpio_data_ready_->Unexport() ||
    !gpio_reset_->Unexport() ||
    !gpio_cs_->Unexport()) {
    ROS_ERROR("Failed to unexport: %s", std::strerror(errno));
    return false;
  }

  if (!gpio_data_ready_->Export() ||
    !gpio_reset_->Export() ||
    !gpio_cs_->Export()) {
    ROS_ERROR("Failed to export: %s", std::strerror(errno));
    return false;
  }

  if (!gpio_data_ready_->SetDirection(gpio::Direction::IN) ||
    !gpio_reset_->SetDirection(gpio::Direction::OUT) ||
    !gpio_cs_->SetDirection(gpio::Direction::OUT)) {
    ROS_ERROR("Failed to set gpio direction: %s", std::strerror(errno));
    return false;
  }

  if (!gpio_data_ready_->SetEdge(gpio::Edge::RISING)) {
    ROS_ERROR("Failed to set GPIO edge: %s", std::strerror(errno));
    return false;
  }

  // The current version assumes that SPI chip select is diabled.
  // Engage chip select signal here manually.
  // Keep it low assuming that there is no other SPI device on the same bus.
  if (prototype_ != 3)
    gpio_cs_->SetValue(gpio::Value::LOW);

  return true;
}

void EpsonImuNodelet::CloseGPIO(void) {
  if (gpio_data_ready_ != NULL)
    gpio_data_ready_->Unexport();

  if (gpio_reset_ != NULL)
    gpio_reset_->Unexport();

  if (gpio_cs_ != NULL && prototype_ != 3) {
    gpio_cs_->SetValue(gpio::Value::HIGH);
    gpio_cs_->Unexport();
  }
}

bool EpsonImuNodelet::InitSPI(void) {
  if ((fd_spi_dev_ = open(spi_dev_.c_str(), O_RDWR)) < 0) {
    ROS_ERROR("Failed to open '%s': %s", spi_dev_.c_str(), std::strerror(errno));
    return false;
  }

  // Initialize SPI
  if ((ioctl(fd_spi_dev_, SPI_IOC_WR_MODE, &spi_mode_) < 0) ||
    (ioctl(fd_spi_dev_, SPI_IOC_RD_MODE, &spi_mode_) < 0)) {
    std::cerr << "Failed to set SPI mode: ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }
  /*
  if (ioctl(fd_spi_dev_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_) < 0) {
    std::cerr << "Failed to set SPI write bits per word: ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }

  if (ioctl(fd_spi_dev_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_) < 0) {
    std::cerr << "Failed to set SPI read bits per word: ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }
  */
  if (ioctl(fd_spi_dev_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz_) < 0) {
    std::cerr << "Failed to set SPI write max speed: ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }

  if (ioctl(fd_spi_dev_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed_hz_) < 0) {
    std::cerr << "Failed to set SPI read max speed: ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

void EpsonImuNodelet::CloseSPI(void) {
  if (!(fd_spi_dev_ < 0))
    close(fd_spi_dev_);
}

void EpsonImuNodelet::Exit() {
  CloseGPIO();
  CloseSPI();

  ros::shutdown();
}

}  // namespace epson_imu

PLUGINLIB_EXPORT_CLASS(epson_imu::EpsonImuNodelet, nodelet::Nodelet);
