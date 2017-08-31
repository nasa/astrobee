// Copyright 2015 Intelligent Robotics Groupp, NASA ARC

#ifndef DDS_ROS_BRIDGE_RAPID_PUB_H_
#define DDS_ROS_BRIDGE_RAPID_PUB_H_

#include <string>
#include <memory>

namespace ff {

/**
 * @brief base class for rapid publishers
 * @details base class for rapid publishers,
 *          which do not need to subscribe to ros
 */
class RapidPub {
 protected:
  explicit RapidPub(const std::string& pubTopic):
    m_publishTopic_(pubTopic) {}

  std::string m_publishTopic_;
};

typedef std::shared_ptr<ff::RapidPub> RapidPubPtr;

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_RAPID_PUB_H_
