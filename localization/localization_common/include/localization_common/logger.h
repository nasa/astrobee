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

#ifndef LOCALIZATION_COMMON_LOGGER_H_
#define LOCALIZATION_COMMON_LOGGER_H_

#include <ros/console.h>

#include <glog/logging.h>

#include <sstream>
#include <string>

// Select logger here
// TODO(rsoussan): Add compile definition for this?
#define USE_ROS_LOGGING
// #define USE_GLOG_LOGGING

#ifdef USE_ROS_LOGGING
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// clang-format off
#define LogInfo(msg) \
  do { \
    std::stringstream ss; \
    ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
    ROS_INFO_STREAM(ss.str()); \
  } while (0)

#define LogWarning(msg) \
  do { \
    std::stringstream ss; \
    ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
    ROS_WARN_STREAM(ss.str()); \
  } while (0)

#define LogError(msg) \
  do { \
    std::stringstream ss; \
    ss << __FILENAME__  << ":" << __LINE__ << ": " << msg << std::endl; \
    ROS_ERROR_STREAM(ss.str()); \
  } while (0)

#define LogFatal(msg) \
  do { \
    std::stringstream ss; \
    ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
    ROS_FATAL_STREAM(ss.str()); \
  } while (0)

#define LogDebug(msg) \
  do { \
    std::stringstream ss; \
    ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
    ROS_DEBUG_STREAM(ss.str()); \
  } while (0)

#define LogInfoEveryN(n, msg) \
  do { \
    static int count = 0; \
    ++count; \
    if (count % n == 0) { \
      std::stringstream ss; \
      ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
      ROS_INFO_STREAM(ss.str()); \
    } \
  } while (0)

#define LogWarningEveryN(n, msg) \
  do { \
    static int count = 0; \
    ++count; \
    if (count % n == 0) { \
      std::stringstream ss; \
      ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
      ROS_WARN_STREAM(ss.str()); \
    } \
  } while (0)

#define LogErrorEveryN(n, msg) \
  do { \
    static int count = 0; \
    ++count; \
    if (count % n == 0) { \
      std::stringstream ss; \
      ss << __FILENAME__ << ":" << __LINE__ << ": " << msg << std::endl; \
      ROS_ERROR_STREAM(ss.str()); \
    } \
  } while (0)


#elif defined(USE_GLOG_LOGGING)
#define LogInfo(msg) \
  do { \
    LOG(INFO) << msg; \
  } while (0)
#define LogWarning(msg) \
  do { \
    LOG(WARNING) << msg; \
  } while (0)

#define LogError(msg) \
  do { \
  LOG(ERROR) << msg; \
  } while (0)

#define LogFatal(msg) \
  do { \
  LOG(FATAL) << msg; \
  } while (0)

#define LogDebug(msg) \
  do { \
  VLOG(2) << msg; \
  } while (0)

#define LogInfoEveryN(n, msg) \
  do { \
  LOG_EVERY_N(INFO, n) << msg; \
  } while (0)

#define LogWarningEveryN(n, msg) \
  do { \
  LOG_EVERY_N(WARNING, n) << msg; \
  } while (0)

#define LogErrorEveryN(n, msg) \
  do { \
  LOG_EVERY_N(ERROR, n) << msg; \
  } while (0)


#endif
// clang-format on
#endif  // LOCALIZATION_COMMON_LOGGER_H_
