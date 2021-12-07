// Copyright 2017 Intelligent Robotics Geoup, NASA ARC

#ifndef DDS_ROS_BRIDGE_ROS_LOG_SAMPLE_H_
#define DDS_ROS_BRIDGE_ROS_LOG_SAMPLE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "rosgraph_msgs/Log.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidDds/RapidConstants.h"

#include "rapidIo/RapidIoParameters.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstants.h"
#include "dds_msgs/LogSampleSupport.h"

namespace ff {

class RosLogSampleToRapid : public RosSubRapidPub {
 public:
  RosLogSampleToRapid(const std::string& subscribe_topic,
                            const std::string& pub_topic,
                            const ros::NodeHandle& nh,
                            const unsigned int queue_size = 10);

  void MsgCallback(const rosgraph_msgs::LogConstPtr& msg);

 private:
  using StateSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::LogSample>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_LOG_SAMPLE_H_
