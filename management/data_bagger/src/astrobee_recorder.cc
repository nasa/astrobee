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

#define READ 0
#define WRITE 1

using std::cout;
using std::endl;
using std::set;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Time;

namespace astrobee_rosbag {
// Recorder

  Recorder::Recorder(RecorderOptions const& options, ros::NodeHandle *nh) :
  options_(options),
  child_process_(-1) {
    // Declare check disk timer
    timer_check_disk_ = nh->createTimer(ros::Duration(20.0), &Recorder::checkDisk, this, false, false);
  }

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

    // Start the recording
    doRecord();
  }

  // Called by the data bagger to stop recording
  void Recorder::stop() {
    // There is a rosbag record running
    if ((child_process_ != -1) && (system(("rosnode kill /" + options_.name).c_str()) < 0)) {
      ROS_ERROR("Could not stop recording, recorder PID is %i", child_process_);
      return;
    }
    timer_check_disk_.stop();
    child_process_ = -1;
  }

  //! Thread that starts the rosbag record command.
  void Recorder::doRecord() {
    // Start recording command
    std::string cmd = "rosbag record ";

    // Specify name
    if (options_.prefix != "") {
      cmd += " --output-name=" + options_.prefix;
    }

    // Specify split option
    if (options_.split == true) {
      cmd += " --split --size=" + std::to_string(options_.max_size);
    }

    // // Specify maximum duration
    if (options_.max_duration.toSec() != -1) {
      cmd += " --duration=" + std::to_string(options_.max_duration.toSec());
    }

    // Specify topics to record
    for (uint i = 0 ; i < options_.topics.size(); ++i) {
      cmd += " " + options_.topics[i];
    }

    // Specify node name
    cmd += " __name:=" + options_.name;

    ROS_DEBUG("%s", cmd.c_str());
    int infp, outfp;
    if ((child_process_ = popen2(cmd.c_str(), &infp, &outfp)) <= 0) {
      printf("Unable to start recording\n");
      exit(1);
    }
    // Start timer for disk check
    timer_check_disk_.start();
  }

// Check dist usage to make sure the recorder doesn't overflow the disk
void Recorder::checkDisk(const ros::TimerEvent&) {
  // Get the disk stats
#if BOOST_FILESYSTEM_VERSION < 3
  struct statvfs fiData;
  if ((statvfs(options_.prefix.substr(0, options_.prefix.find_last_of('/')).c_str(), &fiData)) < 0) {
    ROS_WARN("Failed to check filesystem stats.");
    return true;
  }
  uint64_t free_space = 0;
  free_space = (uint64_t) (fiData.f_bsize) * (uint64_t) (fiData.f_bavail);
  if (free_space < options_.min_space) {
    ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.",
      options_.min_space_str.c_str(), options_.prefix.c_str());
    // Stop recording
    stop();
    return;
  } else if (free_space < 5 * options_.min_space) {
    ROS_WARN("Less than 5 x %s of space free on disk with %s.",
      options_.min_space_str.c_str(), options_.prefix.c_str());
  }
#else
  boost::filesystem::path p(
    boost::filesystem::system_complete(options_.prefix.substr(0, options_.prefix.find_last_of('/')).c_str()));
  p = p.parent_path();
  boost::filesystem::space_info info;
  try {
    info = boost::filesystem::space(p);
  }
  catch (boost::filesystem::filesystem_error &e) {
    ROS_WARN("Failed to check filesystem stats [%s].", e.what());
    return;
  }
  if (info.available < options_.min_space) {
    ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.",
      options_.min_space_str.c_str(), options_.prefix.c_str());
    // Stop recording
    stop();
    return;
  } else if (info.available < 5 * options_.min_space) {
    ROS_WARN("Less than 5 x %s of space free on disk with %s.",
      options_.min_space_str.c_str(), options_.prefix.c_str());
  }
#endif
  return;
}

// Generalized popen2 function that returns the PID and allows read/write
pid_t Recorder::popen2(const char *command, int *infp, int *outfp) {
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
    return -1;

  pid = fork();

  if (pid < 0) {
    return pid;
  } else if (pid == 0) {
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", command, NULL);
    perror("execl");
    exit(1);
  }
  if (infp == NULL)
    close(p_stdin[WRITE]);
  else
    *infp = p_stdin[WRITE];

  if (outfp == NULL)
    close(p_stdout[READ]);
  else
    *outfp = p_stdout[READ];

  return pid;
}

}  // namespace astrobee_rosbag

