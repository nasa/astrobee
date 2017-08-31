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

#include <epson_imu/G362P.h>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace epson_imu {

G362P::G362P(int fd_spi_dev, uint32_t spi_speed_hz,
  gpio::GPIO *gpio_data_ready, gpio::GPIO *gpio_reset)
  : fd_spi_dev_(fd_spi_dev)
  , spi_speed_hz_(spi_speed_hz)
  , gpio_data_ready_(gpio_data_ready)
  , gpio_reset_(gpio_reset)
  , mode_(Mode::CONFIGURATION)
  , sampling_rate_(SamplingRate::SPS125)
  , flag_enabled_(true)
  , temp_enabled_(true)
  , gyro_enabled_(true)
  , accl_enabled_(true)
  , gpio_enabled_(true)
  , count_enabled_(true)
  , chksm_enabled_(true)
  , temp_bits_(OutputBits::B32)
  , gyro_bits_(OutputBits::B32)
  , accl_bits_(OutputBits::B32) {
  timeout_data_ready_.tv_sec = 1;
  timeout_data_ready_.tv_nsec = 0;
}

G362P::~G362P(void) {
}

bool G362P::IsReady(void) {
  return ((ReadRegister(Window::WIN1, REG_GLOB_CMD_H) & 0x0400) == 0);
}

bool G362P::HasHardError(void) {
  return ((ReadRegister(Window::WIN0, REG_DIAG_STAT_H) & 0x6000) != 0);
}

void G362P::HardReset(void) {
  gpio_reset_->SetValue(gpio::Value::LOW);
  usleep(EPSON_POWER_ON_DELAY);
  gpio_reset_->SetValue(gpio::Value::HIGH);
  usleep(EPSON_POWER_ON_DELAY);
}

void G362P::SoftReset(void) {
  WriteRegister(Window::WIN1, REG_GLOB_CMD_H, 0x08);
  // Wait 800 msec.
  usleep(EPSON_POWER_ON_DELAY);
}

bool G362P::PowerOn(void) {
  HardReset();
  SoftReset();

  // Wait until NOT_READY bit goes to 0.
  int retry_count = 0;
  while (!IsReady()) {
    std::cout << "Waiting until the IMU gets ready ("
      << ++retry_count << "/10) ..." << std::endl;
    if (retry_count >= 10) {
      std::cerr << "The IMU is not ready." << std::endl;
      return false;
    }
    usleep(EPSON_POWER_ON_DELAY);
  }

  // Confirm HARD_ERR bits.
  retry_count = 0;
  while (HasHardError()) {
    std::cout << "The IMU has a hardware error. ";
    std::cout << "Doing hardware reset... (" << ++retry_count << "/10) ..." << std::endl;
    HardReset();
     if (retry_count >= 10) {
      std::cerr << "The IMU has a hardware error." << std::endl;
      return false;
    }
  }

  usleep(EPSON_POWER_ON_DELAY);

  return true;
}

uint16_t G362P::MakeWord(uint8_t high, uint8_t low) {
  return (uint16_t)(((uint16_t)(high) << 8) | (uint16_t)(low));
}

void G362P::SetWindow(Window window) {
  TransferWord(MakeWord((REG_WIN_CTRL_H | 0x80), window), STALL_NORMAL_MODE);
}

void G362P::WriteRegister(Window window, uint8_t address, uint8_t value) {
  SetWindow(window);
  TransferWord(MakeWord((address | 0x80), value), STALL_NORMAL_MODE);
}

uint16_t G362P::ReadRegister(Window window, uint8_t address) {
  // Select window.
  SetWindow(window);

  // Read command.
  TransferWord(MakeWord(address, 0x00), STALL_NORMAL_MODE);

  // Return value.
  return TransferWord(0x0000, STALL_NORMAL_MODE);
}

void G362P::DumpRegisters(void) {
  std::cout << std::endl;
  std::cout << "Dump all readable registers:" << std::endl;
  std::cout << "=============================================" << std::endl;
  std::cout << std::hex;
  for (unsigned int i = 0; i < ARRAY_SIZE(kRegisters); i++) {
    const Register &reg = kRegisters[i];

    if (reg.readable) {
      uint16_t value = ReadRegister(reg.window, reg.address);

      std::cout << (unsigned int)(reg.window) << "\t"
        << reg.name << " (0x" << (unsigned int)(reg.address)
        << "): 0x" << (unsigned int)(value) << std::endl;
    }
  }
  std::cout << std::dec;
  std::cout << "=============================================" << std::endl << std::endl;;
}

bool G362P::SetSamplingRate(SamplingRate sampling_rate) {
  if (mode_ != Mode::CONFIGURATION) {
    std::cerr << "Unable to change the sampling rate in Sampling mode." << std::endl;
    return false;
  }

  double scale = 1.1;

  switch (sampling_rate) {
  case SamplingRate::SPS2000:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 2000);
  case SamplingRate::SPS1000:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 1000);
    break;
  case SamplingRate::SPS500:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 500);
    break;
  case SamplingRate::SPS250:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 250);
    break;
  case SamplingRate::SPS125:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 125);
    break;
  case SamplingRate::SPS62_5:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 62.5);
    break;
  case SamplingRate::SPS31_25:
    timeout_data_ready_.tv_sec = 0;
    timeout_data_ready_.tv_nsec = (int64_t)(1000000000 * scale / 31.25);
    break;
  default:
    timeout_data_ready_.tv_sec = 1;
    timeout_data_ready_.tv_nsec = 0;
    break;
  }

  WriteRegister(Window::WIN1, REG_SMPL_CTRL_L, sampling_rate);

  return true;
}

bool G362P::SetFilter(Filter filter) {
  if (mode_ != Mode::CONFIGURATION) {
    std::cerr << "Unable to change the sampling rate in Sampling mode." << std::endl;
    return false;
  }

  // Send filter setting command.
  WriteRegister(Window::WIN1, REG_FILTER_CTRL_H, filter);

  int retry_count = 0;
  // Wait until filter setting has finished.
  while ((ReadRegister(Window::WIN1, REG_FILTER_CTRL_H) & 0x2000) != 0) {
    std::cout << "Wait until filter setting has finished... ("
      << ++retry_count << "/10)" << std::endl;

    if (retry_count >= 10) {
      std::cerr << "Failed to set filter" << std::endl;
      return false;
    }

    usleep(100000);
  }

  return true;
}

void G362P::EnableUART(bool enable) {
  WriteRegister(Window::WIN1, REG_UART_CTRL_H, enable ? 0x01 : 0x00);
}


bool G362P::SetOutputBits(OutputBits temp, OutputBits gyro, OutputBits accl) {
  if (mode_ != Mode::CONFIGURATION) {
    std::cerr << "Unable to configure output in Sampling mode." << std::endl;
    return false;
  }

  temp_bits_ = temp;
  gyro_bits_ = gyro;
  accl_bits_ = accl;

  uint8_t bits = 0x00;

  if (temp_bits_ == OutputBits::B32)
    bits |= 0x40;
  if (gyro_bits_ == OutputBits::B32)
    bits |= 0x20;
  if (accl_bits_ == OutputBits::B32)
    bits |= 0x10;

  WriteRegister(Window::WIN1, REG_BURST_CTRL2_L, bits);

  return true;
}

bool G362P::EnableOutput(bool flag, bool temp, bool gyro, bool accl,
  bool gpio, bool count, bool chksm) {
  if (mode_ != Mode::CONFIGURATION) {
    std::cerr << "Unable to configure output in Sampling mode." << std::endl;
    return false;
  }

  flag_enabled_ = flag;
  temp_enabled_ = temp;
  gyro_enabled_ = gyro;
  accl_enabled_ = accl;
  gpio_enabled_ = gpio;
  count_enabled_ = count;
  chksm_enabled_ = chksm;

  uint8_t output = 0x00;

  if (flag_enabled_)
    output |= 0x80;
  if (temp_enabled_)
    output |= 0x40;
  if (gyro_enabled_)
    output |= 0x20;
  if (accl_enabled_)
    output |= 0x10;

  WriteRegister(Window::WIN1, REG_BURST_CTRL1_L, output);

  output = 0x00;

  if (gpio_enabled_)
    output |= 0x04;
  if (count_enabled_)
    output |= 0x02;
  if (chksm_enabled_)
    output |= 0x01;

  WriteRegister(Window::WIN1, REG_BURST_CTRL1_H, output);

  return true;
}

bool G362P::SetMode(Mode mode) {
  int retry_count = 0;

  while (true) {
    // Set mode.
    if (mode == Mode::CONFIGURATION) {
      WriteRegister(Window::WIN0, REG_MODE_CTRL_L, 0x02);
    } else {
      WriteRegister(Window::WIN0, REG_MODE_CTRL_L, 0x01);
    }

    // Give some time to set the register value.
    usleep(10000);

    // Check if mode is set correctly.
    Mode ret = GetMode();

    if (ret != mode) {
      if (retry_count >= 10) {
        std::cerr << "Failed to change mode" << std::endl;
        return false;
      }

      std::cerr << "Tried to set mode to '" << mode << "' ";
      std::cerr << "but returned '" << ret << "'" << std::endl;
      std::cerr << "Try again... (" << ++retry_count << "/10)" << std::endl;

      usleep(10000);
    } else {
      // Success
      break;
    }
  }

  return true;
}

Mode G362P::GetMode(void) {
  return (((ReadRegister(Window::WIN0, REG_MODE_CTRL_H) & 0x0400) == 0) ?
    Mode::SAMPLING : Mode::CONFIGURATION);
}

void G362P::BurstRead(Data *data) {
  // Burst read command.
  SetWindow(Window::WIN0);
  TransferWord(MakeWord((REG_BURST_H | 0x80), 0x00), STALL_BURST_MODE_BEFORE_DATA);

  data->chksm_ = 0;

  if (flag_enabled_) {
    data->flag_ = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
    data->chksm_ += data->flag_;
  } else {
    data->flag_ = 0;
  }

  if (temp_enabled_) {
    if (temp_bits_ == OutputBits::B32) {
      uint16_t t_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
      uint16_t t_l = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

      // 32-bit 2's complement
      int32_t t = (int32_t)((t_h << 16) | t_l);

      data->temp_ = SF_TEMP / 65536.0 * (t + 997064704.0) + 25.0;
      data->chksm_ += (t_h + t_l);
    } else {
      uint16_t t_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

      // 16-bit 2's complement
      int32_t t = (int32_t)(t_h);

      data->temp_ = SF_TEMP * (t + 15214) + 25.0;
      data->chksm_ += t_h;
    }
  } else {
    data->temp_ = 0;
  }

  if (gyro_enabled_) {
    if (gyro_bits_ == OutputBits::B32) {
      for (int i = 0; i < 3; i++) {
        uint16_t g_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
        uint16_t g_l = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

        // 32-bit 2's complement
        int32_t g = (int32_t)((g_h << 16) | g_l);

        data->gyro_[i] = SF_GYRO / 65536.0 * g;
        data->chksm_ += (g_h + g_l);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        uint16_t g_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

        // 16-bit 2's complement
        int32_t g = (int32_t)(g_h);

        data->gyro_[i] = SF_GYRO * g;
        data->chksm_ += g_h;
      }
    }
  } else {
    for (int i = 0; i < 3; i++)
      data->gyro_[i] = 0;
  }

  if (accl_enabled_) {
    if (accl_bits_ == OutputBits::B32) {
      for (int i = 0; i < 3; i++) {
        uint16_t a_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
        uint16_t a_l = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

        // 32-bit 2's complement
        int32_t a = (int32_t)((a_h << 16) | a_l);

        data->accl_[i] = SF_ACCL / 65536.0 * a;
        data->chksm_ += (a_h + a_l);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        uint16_t a_h = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);

        // 16-bit 2's complement
        int32_t a = (int32_t)(a_h);

        data->accl_[i] = SF_ACCL * a;
        data->chksm_ += a_h;
      }
    }
  } else {
    for (int i = 0; i < 3; i++)
      data->accl_[i] = 0;
  }

  if (gpio_enabled_) {
    data->gpio_ = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
    data->chksm_ += data->gpio_;
  } else {
    data->gpio_ = 0;
  }

  if (count_enabled_) {
    data->count_ = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
    data->chksm_ += data->count_;
  } else {
    data->count_ = 0;
  }

  if (chksm_enabled_) {
    uint16_t chksm = TransferWord(0x00, STALL_BURST_MODE_BETWEEN_DATA);
    // Checksum should be zero when data is good.
    data->chksm_ -= chksm;
  } else {
    data->chksm_ = 0;
  }
}

void G362P::PrintData(const Data& data) {
  std::cout << "---------------------------" << std::endl;
  std::cout << "FLAG: 0x" << std::hex << (unsigned int)data.flag_ << std::endl;
  std::cout << "Temp: " << std::dec << data.temp_ << std::endl;
  std::cout << "Gyro-X: " << data.gyro_[0] << std::endl;
  std::cout << "Gyro-Y: " << data.gyro_[1] << std::endl;
  std::cout << "Gyro-Z: " << data.gyro_[2] << std::endl;
  std::cout << "Accl-X: " << data.accl_[0] << std::endl;
  std::cout << "Accl-Y: " << data.accl_[1] << std::endl;
  std::cout << "Accl-Z: " << data.accl_[2] << std::endl;
  std::cout << "GPIO: 0x" << std::hex << (unsigned int)(data.gpio_) << std::endl;
  std::cout << "COUNT: " << std::dec << (unsigned int)data.count_ << std::endl;
  std::cout << "CHKSM: 0x" << std::hex << (unsigned int)(data.chksm_) << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << std::dec << std::endl;
}

bool G362P::ReadData(Data *data) {
  // Wait until DRDY is asserted.
  gpio::Interrupt ret = gpio_data_ready_->WaitInterrupt(&timeout_data_ready_);

  if (ret == gpio::Interrupt::SUCCESS) {
    BurstRead(data);

    if (data->chksm_ == 0) {
      return true;
    } else {
      std::cerr << "Checksum failed: " << data->chksm_ << std::endl;
      return false;
    }
  } else if (ret == gpio::Interrupt::TIMEOUT) {
    BurstRead(data);
    std::cerr << "IMU TIMEOUT" << std::endl;
    return false;
  } else {
    std::cerr << "WaitInterrupt() failed" << std::endl;
    return false;
  }
}

uint16_t G362P::TransferWord(uint16_t word, uint16_t delay_usecs) {
  uint16_t tx_buf[1] = { word };
  uint16_t rx_buf[1] = { 0x0000 };

  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(struct spi_ioc_transfer));

  tr.tx_buf = (uint64_t)(tx_buf);
  tr.rx_buf = (uint64_t)(rx_buf);
  tr.len = 2;  // sizeof(uint16_t) / sizeof(uint8_t)
  tr.speed_hz = spi_speed_hz_;
  tr.delay_usecs = delay_usecs;
  tr.bits_per_word = 16;
  tr.pad = 0;

  if (ioctl(fd_spi_dev_, SPI_IOC_MESSAGE(1), &tr) < 1) {
    std::cerr << "Failed to send SPI messages: ";
    std::cerr << std::strerror(errno) << std::endl;
  }

#ifdef DEBUG_SPI_MESSAGE
  std::cout << std::hex  << (unsigned int)(tx_buf[0]) << "\t"
    << (unsigned int)(rx_buf[0]) << std::endl;
  std::cout << std::dec;
#endif

  return rx_buf[0];
}

}  // namespace epson_imu
