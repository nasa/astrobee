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

#ifndef EPSON_IMU_G362P_H_
#define EPSON_IMU_G362P_H_

#define EPSON_POWER_ON_DELAY        800000  // usec

#define STALL_NORMAL_MODE             20  // usec
#define STALL_BURST_MODE_BEFORE_DATA  45  // usec
#define STALL_BURST_MODE_BETWEEN_DATA 5  // usec

// Window 0
#define REG_BURST_H     0x00
#define REG_BURST_L     0x01
#define REG_MODE_CTRL_H 0x02
#define REG_MODE_CTRL_L 0x03
#define REG_DIAG_STAT_H 0x04
#define REG_DIAG_STAT_L 0x05
#define REG_FLAG_H      0x06
#define REG_FLAG_L      0x07
#define REG_GPIO_H      0x08
#define REG_GPIO_L      0x09
#define REG_COUNT       0x0A
#define REG_TEMP_HIGH   0x0E
#define REG_TEMP_LOW    0x10
#define REG_XGYRO_HIGH  0x12
#define REG_XGYRO_LOW   0x14
#define REG_YGYRO_HIGH  0x16
#define REG_YGYRO_LOW   0x18
#define REG_ZGYRO_HIGH  0x1A
#define REG_ZGYRO_LOW   0x1C
#define REG_XACCL_HIGH  0x1E
#define REG_XACCL_LOW   0x20
#define REG_YACCL_HIGH  0x22
#define REG_YACCL_LOW   0x24
#define REG_ZACCL_HIGH  0x26
#define REG_ZACCL_LOW   0x28

// Window 1
#define REG_SIG_CTRL_H    0x00
#define REG_SIG_CTRL_L    0x01
#define REG_MSC_CTRL_H    0x02
#define REG_MSC_CTRL_L    0x03
#define REG_SMPL_CTRL_H   0x04
#define REG_SMPL_CTRL_L   0x05
#define REG_FILTER_CTRL_H 0x06
#define REG_FILTER_CTRL_L 0x07
#define REG_UART_CTRL_H   0x08
#define REG_UART_CTRL_L   0x09
#define REG_GLOB_CMD_H    0x0A
#define REG_GLOB_CMD_L    0x0B
#define REG_BURST_CTRL1_H 0x0C
#define REG_BURST_CTRL1_L 0x0D
#define REG_BURST_CTRL2_H 0x0E
#define REG_BURST_CTRL2_L 0x0F
#define REG_WIN_CTRL_H    0x7E
#define REG_WIN_CTRL_L    0x7F

// Scale Factors
#define SF_TEMP (0.0042725)
#define SF_GYRO (0.005)
#define SF_ACCL (0.125)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define CHECK_REG_VALUE 1

#include <epson_imu/GPIO.h>

#include <time.h>

#include <iostream>

namespace epson_imu {

enum Mode {
  SAMPLING = 0x00,
  CONFIGURATION = 0x01
};

enum Window {
  WIN0 = 0x00,
  WIN1 = 0x01
};

enum OutputBits {
  B16 = 0,
  B32 = 1
};

enum Filter {
  MOV_AVG_TAP_2 = 0x01,
  MOV_AVG_TAP_4 = 0x02,
  MOV_AVG_TAP_8 = 0x03,
  MOV_AVG_TAP_16 = 0x04,
  MOV_AVG_TAP_32 = 0x05,
  MOV_AVG_TAP_64 = 0x06,
  MOV_AVG_TAP_128 = 0x07,
  FIR_TAP_32_FC_50 = 0x08,
  FIR_TAP_32_FC_100 = 0x09,
  FIR_TAP_32_FC_200 = 0x0A,
  FIR_TAP_32_FC_400 = 0x0B,
  FIR_TAP_64_FC_50 = 0x0C,
  FIR_TAP_64_FC_100 = 0x0D,
  FIR_TAP_64_FC_200 = 0x0E,
  FIR_TAP_64_FC_400 = 0x0F,
  FIR_TAP_128_FC_50 = 0x10,
  FIR_TAP_128_FC_100 = 0x11,
  FIR_TAP_128_FC_200 = 0x12,
  FIR_TAP_128_FC_400 = 0x13
};

enum SamplingRate {
  SPS2000 = 0x01,
  SPS1000 = 0x02,
  SPS500 = 0x03,
  SPS250 = 0x04,
  SPS125 = 0x05,
  SPS62_5 = 0x06,
  SPS31_25 = 0x07
};

typedef struct _Data {
  uint16_t flag_;
  double temp_;     // degree celcius
  double gyro_[3];  // deg/s
  double accl_[3];  // mG
  uint16_t gpio_;
  uint16_t count_;
  uint16_t chksm_;
} Data;

typedef struct _Register {
  // Window ID
  Window window;
  // Address
  uint8_t address;
  // Name
  const char *name;
  // Readable?
  bool readable;
  // Writable?
  bool writable;
} Register;

constexpr Register kRegisters[] = {
  // Window ID, Address, Name, Readable, Writable
  { Window::WIN0, REG_BURST_H, "BURST_H", false, true },
  { Window::WIN0, REG_BURST_L, "BURST_H", false, false },
  { Window::WIN0, REG_MODE_CTRL_H, "MODE_CTRL_H", true, true },
  { Window::WIN0, REG_MODE_CTRL_L, "MODE_CTRL_L", true, true },
  { Window::WIN0, REG_DIAG_STAT_H, "DIAG_STAT_H", true, false },
  { Window::WIN0, REG_DIAG_STAT_L, "DIAG_STAT_L", true, false },
  { Window::WIN0, REG_FLAG_H, "FLAG_H", true, false },
  { Window::WIN0, REG_FLAG_L, "FLAG_L", true, false },
  { Window::WIN0, REG_GPIO_H, "GPIO_H", true, true },
  { Window::WIN0, REG_GPIO_L, "GPIO_L", true, true },
  { Window::WIN0, REG_COUNT, "COUNT", true, false },
  { Window::WIN0, REG_TEMP_HIGH, "TEMP_HIGH", true, false },
  { Window::WIN0, REG_TEMP_LOW, "TEMP_LOW", true, false },
  { Window::WIN0, REG_XGYRO_HIGH, "XGYRO_HIGH", true, false },
  { Window::WIN0, REG_XGYRO_LOW, "XGYRO_LOW", true, false },
  { Window::WIN0, REG_YGYRO_HIGH, "YGYRO_HIGH", true, false },
  { Window::WIN0, REG_YGYRO_LOW, "YGYRO_LOW", true, false },
  { Window::WIN0, REG_ZGYRO_HIGH, "ZGYRO_HIGH", true, false },
  { Window::WIN0, REG_ZGYRO_LOW, "ZGYRO_LOW", true, false },
  { Window::WIN0, REG_XACCL_HIGH, "XACCL_HIGH", true, false },
  { Window::WIN0, REG_XACCL_LOW, "XACCL_LOW", true, false },
  { Window::WIN0, REG_YACCL_HIGH, "YACCL_HIGH", true, false },
  { Window::WIN0, REG_YACCL_LOW, "YACCL_LOW", true, false },
  { Window::WIN0, REG_ZACCL_HIGH, "ZACCL_HIGH", true, false },
  { Window::WIN0, REG_ZACCL_LOW, "ZACCL_LOW", true, false },
  { Window::WIN0, REG_WIN_CTRL_H, "WIN_CTRL_H", true, true },
  { Window::WIN0, REG_WIN_CTRL_L, "WIN_CTRL_L", false, false },
  { Window::WIN1, REG_SIG_CTRL_H, "SIG_CTRL_H", true, true },
  { Window::WIN1, REG_SIG_CTRL_L, "SIG_CTRL_L", true, true },
  { Window::WIN1, REG_MSC_CTRL_H, "MSC_CTRL_H", true, true },
  { Window::WIN1, REG_MSC_CTRL_L, "MSC_CTRL_L", true, true },
  { Window::WIN1, REG_SMPL_CTRL_H, "SMPL_CTRL_H", true, false},
  { Window::WIN1, REG_SMPL_CTRL_L, "SMPL_CTRL_L", true, true},
  { Window::WIN1, REG_FILTER_CTRL_H, "FILTER_CTRL_H", true, true },
  { Window::WIN1, REG_FILTER_CTRL_L, "FILTER_CTRL_L", true, false },
  { Window::WIN1, REG_UART_CTRL_H, "UART_CTRL_H", true, true },
  { Window::WIN1, REG_UART_CTRL_L, "UART_CTRL_L", true, true },
  { Window::WIN1, REG_GLOB_CMD_H, "GLOB_CMD_H", true, true },
  { Window::WIN1, REG_GLOB_CMD_L, "GLOB_CMD_H", true, false },
  { Window::WIN1, REG_BURST_CTRL1_H, "BURST_CTRL1_H", true, true },
  { Window::WIN1, REG_BURST_CTRL1_L, "BURST_CTRL1_L", true, true },
  { Window::WIN1, REG_BURST_CTRL2_H, "BURST_CTRL2_H", true, false },
  { Window::WIN1, REG_BURST_CTRL2_L, "BURST_CTRL2_L", true, true },
  { Window::WIN1, REG_WIN_CTRL_H, "WIN_CTRL_H", true, true },
  { Window::WIN1, REG_WIN_CTRL_L, "WIN_CTRL_L", false, false },
};

class G362P {
 private:
  int fd_spi_dev_;
  uint32_t spi_speed_hz_;

  gpio::GPIO *gpio_data_ready_;
  gpio::GPIO *gpio_reset_;

  // Current operation mode.
  Mode mode_;
  SamplingRate sampling_rate_;

  // DRDY pin timeout
  struct timespec timeout_data_ready_;

  // Enable/disable outputs
  bool flag_enabled_, temp_enabled_, gyro_enabled_;
  bool accl_enabled_, gpio_enabled_, count_enabled_, chksm_enabled_;

  // Data bit-width
  OutputBits temp_bits_, gyro_bits_, accl_bits_;

 public:
  // Create a G362P instance in SPI mode.
  G362P(int fd_spi_dev, uint32_t spi_speed_hz, gpio::GPIO *gpio_data_ready, gpio::GPIO *gpio_reset);
  ~G362P(void);

  // Indicates whether the IMU is currently ready.
  bool IsReady(void);
  // Check if the IMU has any hardware error.
  bool HasHardError(void);
  void HardReset(void);
  void SoftReset(void);

  // Self test. Use this function to check whether the outputs of the sensor
  // are within the pre-defined range and operating properly.
  // Use this function while the sensor is not moving.
  // The tests are done by the sensor itself.
  // Returns true if the sensor passes the tests. False otherwise.
  bool SelfTest(void);

  // Initializes the IMU by executing the power-on sequence.
  bool PowerOn(void);
  void SetWindow(Window window);
  bool SetFilter(Filter filter);
  bool SetSamplingRate(SamplingRate sampling_rate);
  bool StartSampling(void);
  bool StopSampling(void);
  // Enable/disable UART.
  void EnableUART(bool enable);
  bool EnableOutput(bool flag, bool temp, bool gyro, bool accl, bool gpio, bool count, bool chksm);
  bool SetOutputBits(OutputBits temp, OutputBits gyro, OutputBits accl);
  // Write a 8-bit data into a register.
  void WriteRegister(Window window, uint8_t address, uint8_t value);
  // Read a 16-bit data from a register.
  uint16_t ReadRegister(Window window, uint8_t address);
  void DumpRegisters(void);
  bool SetMode(Mode mode);
  // Returns the current operation mode.
  Mode GetMode(void);
  // Read data uing the burst mode.
  bool ReadData(Data *data);
  // Burst read
  void PrintData(const Data& data);

  uint16_t TransferWord(uint16_t word, uint16_t delay_usecs);

 private:
  uint16_t MakeWord(uint8_t high, uint8_t low);
  void BurstRead(Data *data);
};

}  // namespace epson_imu

#endif  // EPSON_IMU_G362P_H_
