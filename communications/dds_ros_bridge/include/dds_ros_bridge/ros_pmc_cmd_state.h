// Copyright 2017 Intelligent Robotics Geoup, NASA ARC

#ifndef DDS_ROS_BRIDGE_ROS_PMC_CMD_STATE_H_
#define DDS_ROS_BRIDGE_ROS_PMC_CMD_STATE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_hw_msgs/PmcCommand.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidDds/RapidConstants.h"

#include "rapidIo/RapidIoParameters.h"

#include "rapidUtil/RapidHelper.h"

#include "AstrobeeConstants.h"
#include "PmcCmdStateSupport.h"

namespace ff {

class RosPmcCmdStateToRapid : public RosSubRapidPub {
 public:
  RosPmcCmdStateToRapid(const std::string& subscribe_topic,
                            const std::string& pub_topic,
                            const ros::NodeHandle& nh,
                            const unsigned int queue_size = 10);

  void CopyPmcGoal(const ff_hw_msgs::PmcGoal& ros_goal, rapid::ext::astrobee::PmcGoal& dds_goal);
  void MsgCallback(const ff_hw_msgs::PmcCommandConstPtr& msg);
  void PubPmcCmdState(const ros::TimerEvent& event);
  void SetPmcPublishRate(float rate);

 private:
  ff_hw_msgs::PmcCommandConstPtr pmc_msg_;

  using StateSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::PmcCmdState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  ros::Timer pmc_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_PMC_CMD_STATE_H_
