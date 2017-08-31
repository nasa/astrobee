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

#include <ros/ros.h>

// FSW nodelet
#include <ff_util/ff_nodelet.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Boost includes
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <perching_arm/comm_thread.h>

// C++ STL includes
#include <iostream>

/**
 * \ingroup hardware
 */
namespace perching_arm {

class PerchingArm : public ff_util::FreeFlyerNodelet {
 public:
  PerchingArm() : ff_util::FreeFlyerNodelet(NODE_PERCHING_ARM),
    target_(0), address_(0), data_(0), dataF_(false) {}
  virtual ~PerchingArm() {}

 protected:
  virtual void Initialize(ros::NodeHandle *nh) {
    sub_ = nh->subscribe(TOPIC_HARDWARE_ARM_COMMAND, 1, &PerchingArm::ArmCommandCallback, this);
    pub_ = nh->advertise<std_msgs::Float32MultiArray>(TOPIC_HARDWARE_ARM_FEEDBACK, 1);
    thread_ = boost::thread(&PerchingArm::ThreadInterrupt, this);
  }

  void ThreadInterrupt() {
    unsigned char rxBuffer[50] = {};
    int rxBufLen = 0;
    int rxDataLen = 12;

    std_msgs::Float32MultiArray feedback;
    feedback.data.resize(rxDataLen);

    for (;;) {
      try {
        start_ = boost::posix_time::microsec_clock::local_time();

        if (actuator_.receive_feedback(rxBuffer, rxBufLen) != -1) {
          int rxIdx = 3;

          for (int i = 0; i < rxDataLen; i++) {
            if (i == 1 || i == 2 || i == 4 || i == 5) {
              int fstbyte = rxBuffer[rxIdx++];
              int sndbyte = rxBuffer[rxIdx++];
              int trdbyte = rxBuffer[rxIdx++];
              int fthbyte = rxBuffer[rxIdx++];
              feedback.data[i] = ((fthbyte & 0x00ff) << 24) + ((trdbyte & 0x00ff) << 16)
                               + ((sndbyte & 0x00ff) << 8) + (fstbyte & 0x00ff);
            } else {
              int  lowbyte = rxBuffer[rxIdx++];
              int highbyte = rxBuffer[rxIdx++];
              feedback.data[i] = ((highbyte & 0x00ff) << 8) + (lowbyte & 0x00ff);
            }
          }

          if (feedback.data[0] > 32768)
            feedback.data[0] = feedback.data[0] - 65536;
          if (feedback.data[3] > 32768)
            feedback.data[3] = feedback.data[3] - 65536;
          if (feedback.data[6] > 32768)
            feedback.data[6] = feedback.data[6] - 65536;

          feedback.data[2] = 0.088 * (feedback.data[2] - 2048);
          feedback.data[5] = 0.088 * (feedback.data[5] - 2048);

          feedback.data[7] = 3.3 * feedback.data[7] / (1024 * 0.525);
          feedback.data[8] = 3.3 * feedback.data[8] / (1024 * 100 * 0.0075);
          feedback.data[9] = 3.3 * feedback.data[9] / (1024 * 100 * 0.0100);
        }

        if (dataF_) {
          if (target_ == 10) {
            actuator_.set_DTR(1);
            ros::Duration(0.5).sleep();
            actuator_.set_DTR(0);
            // timer = nh.createTimer(ros::Duration(0.5), &PerchingArm::FTDITimerCallBack, this, true, true);
          } else if (address_ != 255) {
            std::cout << "*** Command Sent ***" << std::endl;
            actuator_.send_command(target_, address_, data_);
          }
          dataF_ = false;
        }

        pub_.publish(feedback);
        ros::spinOnce();

        stop_ = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration dur = stop_ - start_;

        boost::this_thread::sleep(boost::posix_time::milliseconds(500-dur.total_milliseconds()));
      } catch (boost::thread_interrupted&) {
        ros::shutdown();
        return;
      }
    }
  }

  void ArmCommandCallback(const std_msgs::Int16MultiArray& pac_command) {
    target_  = pac_command.data[0];
    address_ = pac_command.data[1];
    data_    = pac_command.data[2];

    ROS_DEBUG_STREAM("[---Received Message---]");
    ROS_DEBUG_STREAM("Target: " << target_);
    ROS_DEBUG_STREAM("Command: " << address_);
    ROS_DEBUG_STREAM("Value: " << data_);

    dataF_ = true;

    if (target_ == -1 || address_ == 100) {
      thread_.interrupt();
      thread_.join();
      ROS_DEBUG_STREAM("Terminate Process!! Good Bye!!");
      return;
    }
  }

 private:
  ros::Publisher pub_;
  ros::Subscriber sub_;

  boost::thread thread_;
  boost::posix_time::ptime start_;
  boost::posix_time::ptime stop_;

  CommThread actuator_;

  int target_;
  int address_;
  int data_;
  bool dataF_;
};

PLUGINLIB_DECLARE_CLASS(perching_arm, PerchingArm,
                        perching_arm::PerchingArm, nodelet::Nodelet);

}  // namespace perching_arm
