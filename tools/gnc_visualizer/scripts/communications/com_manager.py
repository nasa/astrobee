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

DDS_COM = 'dds'
ROS_COM = 'ros'

class ComManager:
    default_com_method = DDS_COM
    current_com_method = None
    viz = None
    subs_manager = None
    started = False
    com = None
    executor = None

    reset_ekf = lambda self : self.executor.reset_ekf()
    initialize_bias = lambda self : self.executor.initialize_bias()
    toggle_pmc = lambda self, current_value : self.executor.toggle_pmc(current_value)

    def __init__(self, com_method = None):
        if com_method != None:
            self.set_com_method(self.com_method)

    def start_communications(self, viz):
        if com == None:
            return

        if self.started:
            return

        self.viz = viz

        if self.current_com_method == DDS_COM:
            self.__start_dds_com()
        else:
            self.__start_ros_com()

        self.started = True

    def stop_communications(self):
        if self.subs_manager != None:
            self.subs_manager.stop_all()

        self.started = False

    def was_shutdown(self):
        if self.subs_manager == None:
            return False
        
        return com.is_shutdown()

    def set_com_method(self, com_method):
        if com_method == DDS_COM or com_method == ROS_COM:
            self.current_com_method = com_method
        else:
            self.current_com_method = self.default_com_method

        global com
        com = __import__(self.current_com_method + '_support', globals(), locals())

    def __start_ros_com(self):
        self.subs_manager = com.RosSubscriberManager('gnc_visualizer', self.viz.print_to_log)
        self.subs_manager.add_subscriber('rosout', "/rosout", self.viz.log_callback, com.Log)
        self.subs_manager.add_subscriber('truth', "/loc/truth", self.viz.ground_truth_callback, com.PoseStamped)
        self.subs_manager.add_subscriber('ekf', "/gnc/ekf", self.viz.ekf_callback, com.EkfState)
        self.subs_manager.add_subscriber('ctl_cmd', "/gnc/ctl/command", self.viz.command_callback, com.FamCommand)
        self.subs_manager.add_subscriber('ctl_traj', "/gnc/ctl/traj", self.viz.traj_callback, com.ControlState)
        self.subs_manager.add_subscriber('ctl_shaper', "/gnc/ctl/shaper", self.viz.shaper_callback, \
                com.ControlState)
        self.subs_manager.add_subscriber('pmc_cmd', "/hw/pmc/command", self.viz.pmc_callback, com.PmcCommand)

        self.executor = com.RosCommandExecutor()

    def __start_dds_com(self):
        self.subs_manager = com.DdsSubscriberManager()
        self.subs_manager.add_subscriber(
                "ekf_sub", com.DdsSubscriber(
                    "EkfSubscriber::EkfReader", self.viz.ekf_callback, com.EkfState), True)
        self.subs_manager.add_subscriber(
                "fam_sub", com.DdsSubscriber(
                    "FamCmdSubscriber::FamCmdReader", self.viz.command_callback, com.FamCommand), True)
        self.subs_manager.add_subscriber(
                "shaper_sub", com.DdsSubscriber(
                    "GncControlStateSubscriber::ShaperReader", self.viz.shaper_callback, com.ControlState), True)
        self.subs_manager.add_subscriber(
                "traj_sub", com.DdsSubscriber(
                    "GncControlStateSubscriber::TrajReader", self.viz.traj_callback, com.ControlState), True)
        self.subs_manager.add_subscriber(
                "pmc_sub", com.DdsSubscriber(
                    "PmcCmdSubscriber::PmcReader", self.viz.pmc_callback, com.PmcCommand), True)
        self.subs_manager.add_subscriber(
                "log_sub", com.DdsSubscriber(
                    "LogSubscriber::LogReader", self.viz.log_callback, com.Log), True)

        self.executor = com.DdsCommandExecutor()
    
