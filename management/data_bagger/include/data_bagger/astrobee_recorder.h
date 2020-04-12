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

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>

#include <std_msgs/Empty.h>
#include <topic_tools/shape_shifter.h>

#include <queue>
#include <set>
#include <string>
#include <vector>
#include <list>
#include <mutex>
#include <condition_variable> // NOLINT

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
  explicit Recorder(RecorderOptions const& options);

  void doTrigger();

  bool isSubscribed(std::string const& topic) const;

  boost::shared_ptr<ros::Subscriber> subscribe(std::string const& topic);

  int run();

  void stop();

 private:
  void printUsage();

  void updateFilenames();
  void startWriting();
  void stopWriting();

  bool checkLogging();
  bool scheduledCheckDisk();
  bool checkDisk();

  void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
  void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
               std::string const& topic,
               boost::shared_ptr<ros::Subscriber> subscriber,
               boost::shared_ptr<int> count);
  void doRecord();
  void checkNumSplits();
  bool checkSize();
  bool checkDuration(const ros::Time&);
  void doRecordSnapshotter();
  void doCheckMaster(ros::TimerEvent const& e, ros::NodeHandle& node_handle);

  bool shouldSubscribeToTopic(std::string const& topic, bool from_node = false);

  template<class T>
  static std::string timeToStr(T ros_t);

 private:
    RecorderOptions               options_;

    Bag                           bag_;

    std::string                   target_filename_;
    std::string                   write_filename_;
    std::list<std::string>        current_files_;

    std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
    int                           num_subscribers_;      //!< used for book-keeping of our number of subscribers

    int                           exit_code_;            //!< eventual exit code

    boost::condition_variable_any queue_condition_;      //!< conditional variable for queue
    boost::mutex                  queue_mutex_;          //!< mutex for queue
    std::queue<OutgoingMessage>*  queue_;                //!< queue for storing
    uint64_t                      queue_size_;           //!< queue size
    uint64_t                      max_queue_size_;       //!< max queue size

    uint64_t                      split_count_;          //!< split count

    std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders

    ros::Time                     last_buffer_warn_;

    ros::Time                     start_time_;

    bool                          writing_enabled_;
    boost::mutex                  check_disk_mutex_;
    ros::WallTime                 check_disk_next_;
    ros::WallTime                 warn_next_;
    ros::NodeHandle               node_handle_;
    ros::CallbackQueue            callback_queue_;
    std::mutex                    mutex_;
    bool                          should_stop_;
    std::condition_variable       condition_variable_;
};

}  // namespace astrobee_rosbag

#endif  // DATA_BAGGER_ASTROBEE_RECORDER_H_

