// Copyright 2017 Intelligent Robotics Geoup, NASA ARC

#ifndef DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_
#define DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_

#include <string>

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/ControlState.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidDds/RapidConstants.h"

#include "rapidIo/RapidIoParameters.h"

#include "rapidUtil/RapidHelper.h"

#include "AstrobeeConstants.h"
#include "GncControlStateSupport.h"

namespace ff {

class RosGncControlStateToRapid : public RosSubRapidPub {
 public:
  RosGncControlStateToRapid(const std::string& subscribe_topic,
                            const std::string& pub_topic,
                            const ros::NodeHandle& nh,
                            const unsigned int queue_size = 10);

  void CopyVec3D(rapid::Vec3d& vec_out, const geometry_msgs::Vector3& vec_in);
  void MsgCallback(const ff_msgs::ControlStateConstPtr& msg);
  void PubGncControlState(const ros::TimerEvent& event);
  void SetGncPublishRate(float rate);

 private:
  ff_msgs::ControlStateConstPtr gnc_msg_;

  using StateSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::GncControlState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr state_supplier_;

  ros::Timer gnc_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_
