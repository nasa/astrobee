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

#ifndef DATA_BAGGER_ASTROBEE_RECORDER_H_
#define DATA_BAGGER_ASTROBEE_RECORDER_H_

#include <sys/stat.h>
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <time.h>
#include <signal.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include <set>
#include <string>
#include <vector>
#include <list>

#include "rosbag/bag.h"
#include "rosbag/stream.h"
#include "rosbag/macros.h"
#include "rosbag/recorder.h"

namespace astrobee_rosbag {

using rosbag::OutgoingMessage;
using rosbag::OutgoingQueue;
using rosbag::RecorderOptions;
using rosbag::Bag;

class ROSBAG_DECL Recorder {
 public:
  explicit Recorder(RecorderOptions const& options, ros::NodeHandle *nh);

  int run();

  void stop();

 private:
  void checkDisk(const ros::TimerEvent&);

  void doRecord();

  pid_t popen2(const char *command, int *infp, int *outfp);

 private:
    RecorderOptions               options_;

    pid_t                         child_process_;

    ros::Timer                    timer_check_disk_;
};

}  // namespace astrobee_rosbag

#endif  // DATA_BAGGER_ASTROBEE_RECORDER_H_

