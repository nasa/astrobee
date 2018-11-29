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

from data_support import PmcCommand, EkfState, FamCommand, ControlState, Quaternion, Vector3, Log

from sys import path as sysPath
from os import path as osPath
from time import sleep

try:
    import rticonnextdds_connector as rti
except Exception as e:
    raise ImportError("\nYou haven't installed a needed RTI DDS library. \n\nFirst, ensure you have" +
            " pip installed:\n\n\tsudo apt-get install python-pip\n\n" +
            "Then install the library:\n\n\tpip install rticonnextdds-connector\n")

import threading
import math

filepath = osPath.dirname(osPath.realpath(__file__))
connector = rti.Connector("MyParticipantLibrary::Zero", filepath + "/dds_types/DDSProfile.xml")

def is_shutdown():
    return False

class DdsSubscriberManager:
    subscribers = dict()
    sem = None

    def __init__(self):
        self.sem = threading.Semaphore()

    def add_subscriber(self, key, subscriber, auto_start):
        self.subscribers[key] = subscriber

        if auto_start:
            self.subscribers[key].start_sync(self.sem)
        else:
            self.subscribers[key].sem = self.sem


    def get_subscriber(self, key):
        return self.subscribers.get(key, None)

    def start_subscriber(self, key):
        sub = self.get_subscriber(key)

        if sub == None:
            return False
        else:
            sub.start_sync(self.sem)
            if sub.isAlive():
                return True
            else:
                return False

    def stop_all(self):
        for key, value in self.subscribers.iteritems():
            value.stop()

    def start_all(self):
        for key, value in self.subscribers.itemitems():
            value.start_sync(self.sem)

class DdsSubscriber(threading.Thread):

    inputDDS = None
    sem = None
    stopper = None
    callback = None
    timeout = -1
    translator = None

    def __init__(self, dds_sub, callback, ros_type):
	threading.Thread.__init__(self)
	self.inputDDS = connector.getInput(dds_sub)
	self.stopper = threading.Event()
	self.callback = callback
	self.translator = Dict2RosMsgTranslator(ros_type)

    def run(self):
        while not self.stopper.is_set():
	    self.sem.acquire(True)
            connector.wait(self.timeout)
            self.inputDDS.take()
            numOfSamples = self.inputDDS.samples.getLength()

            for j in range(1, numOfSamples + 1):
                if self.inputDDS.infos.isValid(j):
	            dictionary = self.inputDDS.samples.getDictionary(j)
		    data = self.translator.translate(dictionary)
		    self.callback(data)
	    self.sem.release()

    def start_sync(self, sem):
        self.sem = sem

        if not self.isAlive():
            self.start()

    def stop(self):
	self.stopper.set()

class Dict2RosMsgTranslator:

    supported_ros_types = {
	   EkfState:	lambda self, data: self.__dictionary_to_ekf_msg(data),
	   FamCommand:	lambda self, data: self.__dictionary_to_fam_msg(data),
	   ControlState:	lambda self, data: self.__dictionary_to_control_msg(data),
       PmcCommand:     lambda self, data: self.__dictionary_to_pmc_msg(data),
       Log:            lambda self, data: self.__dictionary_to_log_msg(data)
    }

    ros_type = None

    def __init__(self, ros_type):
	self.ros_type = ros_type

    def translate(self, dictionary):
	ros_msg = self.supported_ros_types[self.ros_type](self, dictionary)
	return ros_msg

    def __array_to_vector3d(self, vector_array):
        vector3d = Vector3()
        vector3d.x = vector_array[0]
        vector3d.y = vector_array[1]
        vector3d.z = vector_array[2]

        return vector3d

    def __array_to_quaternion(self, rot_array):
        quaternion = Quaternion()
        quaternion.x = rot_array[0]
        quaternion.y = rot_array[1]
        quaternion.z = rot_array[2]
        quaternion.w = rot_array[3]

        return quaternion

    def __dictionary_to_control_msg(self, dic):
        msg = ControlState()

        msg.when = dic['hdr']['timeStamp'] * 1000

        msg.pose.position = self.__array_to_vector3d(dic['pose']['xyz'])
        msg.pose.orientation = self.__array_to_quaternion(dic['pose']['rot'])

        msg.twist.linear = self.__array_to_vector3d(dic['twist']['linear'])
        msg.twist.angular = self.__array_to_vector3d(dic['twist']['angular'])

        msg.accel.linear = self.__array_to_vector3d(dic['accel']['linear'])
        msg.accel.angular = self.__array_to_vector3d(dic['accel']['angular'])

        return msg

    def __dictionary_to_fam_msg(self, dic):
        msg = FamCommand()

        msg.header.stamp = dic['hdr']['timeStamp']
        msg.wrench.force = self.__array_to_vector3d(dic['wrench']['force'])
        msg.wrench.torque = self.__array_to_vector3d(dic['wrench']['torque'])
        msg.accel = self.__array_to_vector3d(dic['accel'])
        msg.alpha = self.__array_to_vector3d(dic['alpha'])
        msg.status = dic['status']
        msg.position_error = self.__array_to_vector3d(dic['position_error'])
        msg.position_error_integrated = self.__array_to_vector3d(dic['position_error_integrated'])
        msg.attitude_error = self.__array_to_vector3d(dic['attitude_error'])
        msg.attitude_error_integrated = self.__array_to_vector3d(dic['attitude_error_integrated'])
        msg.attitude_error_mag = dic['attitude_error_mag']
        msg.control_mode = dic['control_mode']

        #print msg.asDict()
        return msg

    def __dictionary_to_ekf_msg(self, dic):
        #print dic
        msg = EkfState()

        msg.header.stamp = dic['hdr']['timeStamp']

        msg.pose.position = self.__array_to_vector3d(dic['pose']['xyz'])

        msg.pose.orientation = self.__array_to_quaternion(dic['pose']['rot'])

        msg.velocity = self.__array_to_vector3d(dic['velocity'])
        msg.omega = self.__array_to_vector3d(dic['omega'])
        msg.gyro_bias = self.__array_to_vector3d(dic['gyro_bias'])
        msg.accel = self.__array_to_vector3d(dic['accel'])
        msg.accel_bias = self.__array_to_vector3d(dic['accel_bias'])

        for i in range(0, len(dic['cov_diag'])):
	   msg.cov_diag[i] = dic['cov_diag'][i]

        msg.confidence = dic['confidence']
        msg.status = dic['status']
        msg.of_count = dic['of_count']
        msg.ml_count = dic['ml_count']

        msg.hr_global_pose.position = self.__array_to_vector3d(dic['hr_global_pose']['xyz'])

        msg.hr_global_pose.orientation = self.__array_to_quaternion(dic['hr_global_pose']['rot'])

        for i in range(0, len(dic['ml_mahal_dists'])):
    	    msg.ml_mahal_dists[i] = dic['ml_mahal_dists'][i]

        #print msg.asDict()
        return msg

    def __dictionary_to_pmc_msg(self, dic):
        msg = PmcCommand()
        msg.header.stamp = dic['hdr']['timeStamp']

        for i in range(0, len(dic['goals'])):
            goal = PmcCommand.PmcGoal(dic['goals'][i]['motorSpeed'])
            for j in range(0, len(dic['goals'][i]['nozzlePositions'])):
                goal.nozzle_positions.append(chr(dic['goals'][i]['nozzlePositions'][j]))
            msg.goals.append(goal)

        #print msg.asDict()
        return msg

    def __dictionary_to_log_msg(self, dic):
        msg = Log()
        msg.header.stamp = dic['hdr']['timeStamp']
        msg.level = math.pow(2, dic['level'])
        msg.name = dic['name']
        msg.msg = dic['msg']
        #print msg.asDict()
        return msg

class DdsCommandExecutor:

    def __init__(self):
        pass

    def reset_ekf(self):
        print "reset_ekf function is not yet implemented on dds"


    def initialize_bias(self):
        print "initialize_bias function is not yet implemented on dds"

    def toggle_pmc(self):
        print "toggle_pmc function is not yet implemented on dds"
