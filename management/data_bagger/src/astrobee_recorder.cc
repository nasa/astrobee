/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include "data_bagger/astrobee_recorder.h"

#include <sys/stat.h>
#include <boost/filesystem.hpp>
// Boost filesystem v3 is default in 1.46.0 and above
// Fallback to original posix code (*nix only) if this is not true
#if BOOST_FILESYSTEM_VERSION < 3
  #include <sys/statvfs.h>
#endif
#include <time.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <queue>
#include <set>
#include <sstream>
#include <string>

#include "ros/network.h"
#include "ros/xmlrpc_manager.h"
#include "xmlrpcpp/XmlRpc.h"

#define foreach BOOST_FOREACH

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

using rosbag::OutgoingMessage;
using rosbag::OutgoingQueue;
using rosbag::RecorderOptions;
using rosbag::Bag;
using rosbag::BagException;

namespace astrobee_rosbag {
// Recorder

  Recorder::Recorder(RecorderOptions const& options) :
  options_(options),
  child_process_(-1),
  num_subscribers_(0),
  exit_code_(0),
  split_count_(0),
  writing_enabled_(true),
  should_stop_(false) {}

  // Called by the data bagger to start recording
  int Recorder::run() {
    // If no topics were specifically specified
    if (options_.topics.size() == 0) {
      // Make sure limit is not specified with automatic topic subscription
      if (options_.limit > 0) {
        fprintf(stderr, "Specifing a count is not valid with automatic topic subscription.\n");
        return 1;
      }

      // Make sure topics are specified
      if (!options_.record_all && (options_.node == std::string(""))) {
        fprintf(stderr, "No topics specified.\n");
        return 1;
      }
    }

    if (!node_handle_.ok())
      return 0;

    // Start the recording
    child_process_ = doRecord();


    // // Schedule the disk space check
    // warn_next_ = ros::WallTime();
    // check_disk_next_ = ros::WallTime::now();

    // // Start check timers
    // checkSize();
    // // checkDuration(out.time);
    // checkDisk();
    // checkLogging();
  }

  // Called by the data bagger to stop recording
  void Recorder::stop() {
    if (child_process_ != -1)
      kill(child_process_, SIGKILL);
  }


//! Thread that actually does writing to file.
pid_t Recorder::doRecord() {
  // Start recording command
  std::string cmd = "rosbag record ";
  // Specify topics to record
  for (uint i = 0 ; i < options_.topics.size(); ++i) {
    cmd += options_.topics[i] + " ";
  }
  // Add split option




  // ROS_ERROR_STREAM(cmd);
  // // Fork
  // pid_t pid = fork();

  // // Error, failed to fork()
  // if (pid == -1) {
  //   ROS_ERROR("Failed to start the fork");
  //   return pid;
  // // This is the parent
  // } else if (pid > 0) {
  //   return pid;
  // // This is the child, start recording
  // } else {



  //     execve(...);
  //     _exit(EXIT_FAILURE);   // exec never returns
  // }
}

  // void Recorder::updateFilenames() {
  //   vector<string> parts;

  //   std::string prefix = options_.prefix;
  //   size_t ind = prefix.rfind(".bag");

  //   if (ind != std::string::npos && ind == prefix.size() - 4) {
  //     prefix.erase(ind);
  //   }

  //   if (prefix.length() > 0)
  //     parts.push_back(prefix);
  //   if (options_.append_date)
  //     parts.push_back(timeToStr(ros::WallTime::now()));
  //   if (options_.split)
  //     parts.push_back(boost::lexical_cast<string>(split_count_));

  //   if (parts.size() == 0) {
  //     throw BagException("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
  //   }

  //   target_filename_ = parts[0];
  //   for (unsigned int i = 1; i < parts.size(); i++)
  //     target_filename_ += string("_") + parts[i];

  //   target_filename_ += string(".bag");
  //   write_filename_ = target_filename_ + string(".active");
  // }

void Recorder::checkNumSplits() {
//     if (options_.max_splits > 0) {
//         current_files_.push_back(target_filename_);
//         if (current_files_.size() > options_.max_splits) {
//             int err = unlink(current_files_.front().c_str());
//             if (err != 0) {
//                 ROS_ERROR("Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
//             }
//             current_files_.pop_front();
//         }
//     }
}

bool Recorder::checkSize() {
//     if (options_.max_size > 0) {
//         if (bag_.getSize() > options_.max_size) {
//             if (options_.split) {
//                 stopWriting();
//                 split_count_++;
//                 checkNumSplits();
//                 startWriting();
//             } else {
//                 ros::shutdown();
//                 return true;
//             }
//         }
//     }
    return false;
}

bool Recorder::checkDuration(const ros::Time& t) {
//     if (options_.max_duration > ros::Duration(0)) {
//         if (t - start_time_ > options_.max_duration) {
//             if (options_.split) {
//                 while (start_time_ + options_.max_duration < t) {
//                     stopWriting();
//                     split_count_++;
//                     checkNumSplits();
//                     start_time_ += options_.max_duration;
//                     startWriting();
//                 }
//             } else {
//                 ros::shutdown();
//                 return true;
//             }
//         }
//     }
    return false;
}

// Check
bool Recorder::checkDisk() {
  // Check if 20s have passed to check the disk again
  if (ros::WallTime::now() < check_disk_next_)
    return true;
  check_disk_next_ += ros::WallDuration().fromSec(20.0);

  // Get the disk stats
#if BOOST_FILESYSTEM_VERSION < 3
  struct statvfs fiData;
  if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0) {
    ROS_WARN("Failed to check filesystem stats.");
    return true;
  }
  uint64_t free_space = 0;
  free_space = (uint64_t) (fiData.f_bsize) * (uint64_t) (fiData.f_bavail);
  if (free_space < options_.min_space) {
    ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.",
      options_.min_space_str.c_str(), bag_.getFileName().c_str());







    writing_enabled_ = false;
    return false;
  } else if (free_space < 5 * options_.min_space) {
    ROS_WARN("Less than 5 x %s of space free on disk with %s.",
      options_.min_space_str.c_str(), bag_.getFileName().c_str());
  } else {
    writing_enabled_ = true;
  }
#else
  // boost::filesystem::path p(boost::filesystem::system_complete(bag_.getFileName().c_str()));
  // p = p.parent_path();
  // boost::filesystem::space_info info;
  // try {
  //   info = boost::filesystem::space(p);
  // }
  // catch (boost::filesystem::filesystem_error &e) {
  //   ROS_WARN("Failed to check filesystem stats [%s].", e.what());
  //   writing_enabled_ = false;
  //   return false;
  // }
  // if (info.available < options_.min_space) {
  //   ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.",
  //     options_.min_space_str.c_str(), bag_.getFileName().c_str());








  //   writing_enabled_ = false;
  //   return false;
  // } else if (info.available < 5 * options_.min_space) {
  //   ROS_WARN("Less than 5 x %s of space free on disk with %s.",
  //     options_.min_space_str.c_str(), bag_.getFileName().c_str());
  //   writing_enabled_ = true;
  // } else {
  //   writing_enabled_ = true;
  // }
#endif
  return true;
}

bool Recorder::checkLogging() {
  if (writing_enabled_)
    return true;

  // Send warning every 5s if it is not logging
  ros::WallTime now = ros::WallTime::now();
  if (now >= warn_next_) {
    warn_next_ = now + ros::WallDuration().fromSec(5.0);
    ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
  }
  return false;
}



}  // namespace astrobee_rosbag

