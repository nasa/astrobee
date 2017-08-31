// Copyright 2017 Intelligent Robotics Geoup, NASA ARC

#ifndef DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_
#define DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_

#include <string>

#include "knDds/DdsTypedSupplier.h"

#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/util.h"

#include "ff_msgs/ControlState.h"

#include "AstrobeeConstants.h"
#include "GncControlStateSupport.h"

#include "rapidIo/RapidIoParameters.h"
#include "rapidUtil/RapidHelper.h"

namespace ff {

class RosGncControlStateToRapid : public RosSubRapidPub {
 public:
  RosGncControlStateToRapid(const std::string& subscribeTopic,
                            const std::string& pubTopic,
                            const ros::NodeHandle& nh,
                            const unsigned int queueSize = 10);

  void CopyVec3D(rapid::Vec3d& vecOut, const geometry_msgs::Vector3& vecIn);
  void MsgCallback(const ff_msgs::ControlStateConstPtr& msg);
  void PubGncControlState(const ros::TimerEvent& event);
  void SetGncPublishRate(float rate);

 private:
  ff_msgs::ControlStateConstPtr gnc_msg_;

  using StateSupplier =
      kn::DdsTypedSupplier<rapid::ext::astrobee::GncControlState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  StateSupplierPtr s_supplier_;

  ros::Timer gnc_timer_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_GNC_CONTROL_STATE_H_
