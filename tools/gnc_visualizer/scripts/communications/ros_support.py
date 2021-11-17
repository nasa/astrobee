#!/usr/bin/env python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os
import signal
import socket
import subprocess
import time

import rosgraph
import rospy
from ff_hw_msgs.msg import PmcCommand
from ff_msgs.msg import CommandStamped, ControlState, EkfState, FamCommand
from ff_msgs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log
from std_srvs.srv import Empty


def is_shutdown():
    return rospy.is_shutdown()


class RosSubscriberManager:
    subscribers = dict()
    rosmaster = None
    logger_function = None

    def __init__(self, node_name, logger_function=None):
        self.logger_function = logger_function
        self.__ensure_ros_master()
        rospy.init_node(node_name, anonymous=False, disable_signals=True)

    def add_subscriber(self, key, topic, callback, msg_type):
        self.subscribers[key] = rospy.Subscriber(topic, msg_type, callback)

    def get_subscriber(self, key):
        return self.subscribers.get(key, None)

    def __ensure_ros_master(self):
        try:
            rosgraph.Master("/rostopic").getPid()
        except socket.error:
            if self.logger_function != None:
                self.logger_function("Starting roscore.", "#FFB266")

            self.rosmaster = subprocess.Popen(
                "roscore",
                preexec_fn=os.setsid,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            time.sleep(1)

    def stop_all(self):
        rospy.signal_shutdown("Finished")
        if self.rosmaster != None:
            os.killpg(os.getpgid(self.rosmaster.pid), signal.SIGINT)
            rosmaster.wait()


class RosCommandExecutor:
    def __init__(self):
        pass

    def reset_ekf(self):
        try:
            reset = rospy.ServiceProxy("gnc/ekf/reset", Empty)
            reset()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def initialize_bias(self):
        try:
            initialize = rospy.ServiceProxy("/gnc/ekf/init_bias", Empty)
            initialize()
        except rospy.ServiceException as e:
            print(("Service call failed: %s" % e))

    def toggle_pmc(self, current_value):
        try:
            pmc_enable = rospy.ServiceProxy("/hw/pmc/enable", SetBool)
            new_value = not current_value
            pmc_enable(new_value)
            return new_value
        except rospy.ServiceException as e:
            print(("Service call failed: %s" % e))
            return current_value
