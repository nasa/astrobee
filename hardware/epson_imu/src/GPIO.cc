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

#include <epson_imu/GPIO.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

namespace gpio {

GPIO::GPIO(int gpio)
  : gpio_(gpio)
  , fd_value_(-1) {
  sysfs_gpio_path_ = "/sys/class/gpio";
}

GPIO::~GPIO(void) {
  Unexport();
}

bool GPIO::IsExported(void) {
  std::string path = sysfs_gpio_path_ + "/gpio" + std::to_string(gpio_);
  struct stat buf;
  return (stat(path.c_str(), &buf) == 0);
}

bool GPIO::Export(void) {
  if (IsExported())
    return true;

  std::string path = sysfs_gpio_path_ + "/export";
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open()) {
    std::cerr << "Failed to open '" << path << "'";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }
  ofs << gpio_;
  ofs.close();

  // XXX(tfmorse): We are adding a hack to make sure udev has
  // time to kick in before we try fiddling with the files that
  // this thing creates. Thus, we are going to stat the 'value' file
  // until it has a user that is acceptable (non-0)

  std::string val_path =
    sysfs_gpio_path_ + "/gpio" + std::to_string(gpio_) + "/value";
  struct stat buf;
  int retval;
  int tries = 0;

  do {
    retval = stat(val_path.data(), &buf);
    if (retval < 0 && errno != ENOENT) {
      std::cerr << "Failed to stat " << val_path << ": "
                << std::strerror(errno) << std::endl;
      return false;
    }

    tries++;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  } while (buf.st_uid == 0 && tries <= 10);

  if (tries > 10) {
    std::cerr << "file permissions failed to change for GPIO pin "
              << gpio_ << std::endl;
    return false;
  }

  return true;
}

bool GPIO::Unexport() {
  if (!IsExported())
    return true;

  CloseValue();

  std::string path = sysfs_gpio_path_ + "/unexport";
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open()) {
    std::cerr << "Failed to open '" << path << "'";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }
  ofs << gpio_;
  ofs.close();

  return true;
}

bool GPIO::SetDirection(Direction direction) {
  std::string path = sysfs_gpio_path_ + "/gpio" + std::to_string(gpio_) + "/direction";
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open()) {
    std::cerr << "Failed to open '" << path << "'";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }
  direction == Direction::IN ? ofs << "in" : ofs << "out";
  ofs.close();

  return (direction == Direction::IN ? OpenValue(O_RDONLY) : OpenValue(O_WRONLY));
}

bool GPIO::OpenValue(int oflags) {
  std::string path = sysfs_gpio_path_ + "/gpio" + std::to_string(gpio_) + "/value";

  if ((fd_value_ = open(path.c_str(), oflags)) < 0) {
    std::cerr << "Failed to open file '" << path.c_str() << "': ";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

void GPIO::CloseValue(void) {
  if (fd_value_ > 0)
    close(fd_value_);
}

bool GPIO::SetEdge(Edge edge) {
  std::string path = sysfs_gpio_path_ + "/gpio" + std::to_string(gpio_) + "/edge";
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open()) {
    std::cerr << "Failed to open '" << path << "'";
    std::cerr << std::strerror(errno) << std::endl;
    return false;
  }

  switch (edge) {
  case RISING:
    ofs << "rising";
    break;
  case FALLING:
    ofs << "falling";
    break;
  case BOTH:
    ofs << "both";
    break;
  default:
    ofs << "NONE";
    break;
  }

  ofs.close();

  return true;
}

Value GPIO::GetValue(void) {
  char value;

  if ((lseek(fd_value_, 0, SEEK_SET) < 0) || (read(fd_value_, &value, 1) < 0)) {
    std::cerr << "Failed to read value: " << std::strerror(errno) << std::endl;
    return Value::UNDEFINED;
  }

  return value == '0' ? Value::LOW : Value::HIGH;
}

bool GPIO::SetValue(Value value) {
  char c;

  if (value == Value::LOW) {
    c = '0';
  } else if (value == Value::HIGH) {
    c = '1';
  } else {
    std::cerr << "Failed to write value: UNDEFINED" << std::endl;
    return false;
  }

  if (write(fd_value_, &c, 1) < 0) {
    std::cerr << "Failed to write value: " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

Interrupt GPIO::WaitInterrupt(const struct timespec *timeout) {
  struct pollfd fd_set[1];

  Interrupt ret = Interrupt::SUCCESS;

  memset(reinterpret_cast<void *>(fd_set), 0, sizeof(fd_set));

  fd_set[0].fd = fd_value_;
  fd_set[0].events = POLLPRI | POLLERR;
  fd_set[0].revents = 0;

  char buf[32];

  // dummy read
  lseek(fd_set[0].fd, 0, SEEK_SET);
  read(fd_set[0].fd, buf, 32);

  // poll
  int ready = ppoll(fd_set, 1, timeout, NULL);

  if (ready < 0) {
    std::cerr << "Failed to poll(): " << std::strerror(errno) << std::endl;
    ret = Interrupt::FAILED;
  } else if (ready == 0) {
    if (GetValue() == Value::HIGH) {
      // There was a rising edge, but not detected by the processor
      // (or the interrupt is not delivered to the user space yet)
      ret = Interrupt::SUCCESS;
    } else {
      // No interrupt
      std::cout << "IMU timeout (" << timeout->tv_sec + (timeout->tv_nsec / 1e9) << ")" << std::endl;
      ret = Interrupt::TIMEOUT;
    }
  }

  lseek(fd_set[0].fd, 0, SEEK_SET);
  read(fd_set[0].fd, buf, 32);

  return ret;
}

}  // namespace gpio
