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

import rospy
import rosgraph
import os
import pwd
import time
import getpass
import socket
import thread

from ff_msgs.msg import AckStamped, GuestScienceState, GuestScienceConfig, \
    GuestScienceData, AccessControlStateStamped, CommandStamped, CommandArg, \
    GuestScienceApk, GuestScienceCommand, AckCompletedStatus, AckStatus
from std_msgs.msg import Header
from os import system, name

class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)

pub = rospy.Publisher('comm/dds/command', CommandStamped, queue_size=10)

base_id = 'LocalParticipant'
count = 0
requesting = False
state = None
config = None
current_controller = None
user = None
ack_response = None
data_response = Queue()
last_command_id = None
fault_state = False
apps = None
current_app = None
new_ack = False

ACTION_CONTINUE = 0
ACTION_GO_BACK = 1
ACTION_EXIT = 2

def get_user():
    # TODO Check portability
    user = getpass.getuser()
    machine = socket.gethostname()
    return user + '@' + machine

def clear():
    # for windows
    if name == 'nt':
        _ = system('cls')
    # for mac and linux(here, os.name is 'posix')
    else:
        _ = system('clear')

    time.sleep(0.5)
    print "\n ------- Ground Data System Local Simulator -------\n\n"

def request_control():
    global requesting
    requesting = True
    send_command('requestControl')

def grab_control(msg):
    global requesting
    if msg.cookie != "":
        arg = CommandArg()
        arg.data_type = 5
        arg.s = msg.cookie

        send_command('grabControl', [arg])
        requesting = False

def send_command(name, args = []):
    global count, last_command_id, new_ack, data_response
    new_ack = False
    #data_response = None

    cmd = CommandStamped()
    cmd.header = Header()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = 'world'
    cmd.cmd_name = name
    cmd.args = args

    last_command_id = str(count) + base_id
    cmd.cmd_id = last_command_id
    cmd.cmd_src = user
    cmd.cmd_origin = 'ground'
    cmd.subsys_name = ''

    pub.publish(cmd)
    count = count + 1

def get_manager_config():
    pass

def access_control_callback(data):
    global current_controller

    if requesting:
        grab_control(data)
    else:
        current_controller = data.controller

def ack_callback(data):
    global fault_state, new_ack, ack_response
    if data.cmd_id == last_command_id:
        if (data.status.status == AckStatus.COMPLETED and
            data.completed_status.status != AckCompletedStatus.OK):
            fault_state = True
        ack_response = data
        new_ack = True

def gs_state_callback(data):
    global state
    state = data

def gs_config_callback(data):
    global config
    config = data

def gs_data_callback(data):
    global data_response
    data_response.enqueue(data)
    #data_response = data

def start_subscribers():
    rospy.init_node('gds_gs_simulator')
    rospy.Subscriber("gs/gs_manager/state", GuestScienceState, gs_state_callback)
    rospy.Subscriber("gs/gs_manager/config", GuestScienceConfig, gs_config_callback)
    rospy.Subscriber("gs/data", GuestScienceData, gs_data_callback)
    rospy.Subscriber("mgt/ack", AckStamped, ack_callback)
    rospy.Subscriber("mgt/access_control/state", AccessControlStateStamped, access_control_callback)

    # Wait for master to register subs and pubs
    rospy.sleep(2.)

def gain_control():
    timer = 0

    if current_controller == None:
        print "Astrobee's current controller is undetermined. We cannot proceed"
        return False
    elif current_controller == user:
        print ("Astrobee's controller is: " + current_controller + "\n"
            + "You are the current controller")
        raw_input("Press any key to continue")
        return True
    else:
        print "Astrobee's controller is: " + current_controller + "\n"
        raw_input("Press any key to grab control of the robot")

    # Request and grab control
    print ' > Requesting control'
    request_control()

    while requesting and timer < 20:
        time.sleep(0.5)
        timer += 1

    if fault_state:
        print " > Request failed with message: " + ack_response.message
        return False
    elif timer >= 20:
        print ' > Timeout'
        return False

    timer = 0

    print ' > Grabbing control'
    while current_controller != user:
        time.sleep(0.5)
        timer += 1

    if fault_state:
        print " > Request failed with message: " + ack_response.message
        return False
    elif timer >= 20:
        print ' > Timeout'
        return False

    return True

def get_apk_info():

    # Wait until the GS_manager shows up
    timer = 0

    print (" > Waiting for Guest Science Manager communication."
            " Make sure the app is running in the android device and that you "
            "can ping it from this computer")

    while (state == None or config == None) and timer < 30:
        time.sleep(0.5)
        timer += 1

    if timer >= 30:
        print ' > Timeout'
        return False

    if state.serial != config.serial:
        print ' > Guest Science state and config do not match'
        return False

    print ' > Guest Science Manager found!'
    return True

def select_app():
    global apps, current_app

    # Show available apps and states
    apps = config.apks

    print '\nAvailable Guest Science applications in HLP'

    for i, app in enumerate(apps):
        app_state = 'Running' if state.runningApks[i] else 'Stopped'
        print str(i + 1) + ')  ' + app.short_name + '    ' + app_state

    print str(len(apps) + 1) + ')  ' + 'Exit'

    # Choose an app
    try:
        selection = input("\nType the number of app you want to select: ")
    except:
        print ' > Invalid entry'
        time.sleep(1)
        return None

    if selection == len(apps) + 1:
        return -1

    if selection < 1 or selection > len(apps):
        print ' > Invalid entry'
        time.sleep(1)
        return None

    current_app = apps[selection - 1]
    return (selection - 1)

def select_action():
    print ("a) See available commands\n"
            "b) Start application\n"
            "c) Stop application\n"
            "d) Send Custom Guest Science command\n"
            "e) Go back to apps menu\n"
            "f) Exit")

    option = raw_input("\nType an option: ")
    return option

def input_thread(a_list):
    raw_input()
    a_list.append(True)


def print_gs_feedback():
    global data_response
    print 'Waiting for feedback (command execution).\n'

    # Print ACK
    while new_ack == False:
        time.sleep(0.5)

    print '> Execution response'
    if (ack_response.status.status == AckStatus.COMPLETED and
        ack_response.completed_status.status == AckCompletedStatus.OK):
        print "  Execution was successful!\n"
    else:
        print "  Something went wrong\n"
    print ack_response.message

    # Print GS Data
    print 'Waiting for feedback (app response).\n'
    print ("Please note that some apps may send a confirmation when receiving"
            " a new command and then data feedback. Since we cannot know when"
            " the app will send feedback, we will listen until you manually"
            "stop it.\n You can stop listening by pressing ENTER")

    # Variable and thread used to stop the loop
    a_list = []
    thread.start_new_thread(input_thread, (a_list,))

    while not a_list:
        if not data_response.isEmpty():
            qsize = data_response.size()
            data = data_response.items[qsize - 1]
            if data.apk_name == current_app.apk_name:
                print ('\n> Data response\n  Topic: ' + data.topic +
                    '\n  Data: ' + str(data.data))
                data_response.dequeue()


def execute_action(selection):
    final_act = None
    print '\nYou selected ' + apps[selection].short_name + '. Choose an option\n'

    option = select_action()

    arg = CommandArg()
    arg.data_type = 5
    arg.s = apps[selection].apk_name

    if option == 'a':
        # List commands
        clear()
        print_app_cmd(selection)
        final_act = ACTION_CONTINUE
    elif option == 'b':
        # Start app
        clear()
        send_command('startGuestScience', [arg])
        print_gs_feedback()
        final_act = ACTION_CONTINUE
    elif option == 'c':
        # Stop app
        clear()
        if state.runningApks[selection] == False:
            print '\n > App already stopped'
        else:
            send_command('stopGuestScience', [arg])
            print_gs_feedback()

        final_act = ACTION_CONTINUE
    elif option == 'd':
        # Execute command
        command = None
        while True:
            clear()
            num_cmds = len(apps[selection].commands)
            print_app_cmd(selection)
            print str(num_cmds + 1) + ')  Exit program'
            try:
                command = input('\nType the number of the command you wish to send: ')
            except:
                print ' > Invalid entry'
                time.sleep(1)
                continue

            if command == num_cmds + 1:
                return ACTION_EXIT

            if command < 1 or command > len(apps[selection].commands):
                print ' > Invalid entry'
                time.sleep(1)
            else:
                command -= 1
                break

        arg2 = CommandArg()
        arg2.data_type = 5
        arg2.s = apps[selection].commands[command].command
        clear()
        send_command('customGuestScience', [arg, arg2])
        print_gs_feedback()

        final_act = ACTION_CONTINUE
    elif option == 'e':
        # Go back
        final_act = ACTION_GO_BACK
    elif option == 'f':
        # Exit
        final_act = ACTION_EXIT
    else:
        print ' > Invalid entry'
        final_act = ACTION_CONTINUE

    if final_act != ACTION_GO_BACK and final_act != ACTION_EXIT:
        raw_input("\nPress any key to continue")

    return final_act

def print_app_cmd(selection):
    print '\nAvailable commands'
    for i, cmd in enumerate(apps[selection].commands):
        print str(i + 1) + ')  ' + cmd.name + '\n\t' + cmd.command

def is_ros_running():
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        return False

    return True

def main():
    global user
    timer = 0

    clear()

    # Check ROS master presence
    print ' > Waiting for ROS communications...\n'

    while not is_ros_running():
        if timer == 0:
            print ' > Are you running Astrobee Robot Software?\n'
        elif timer == 30:
            print ' > Timeout. Shutting down...'
            time.sleep(1)
            exit()

        timer += 1
        time.sleep(1)

    print ' > ROS Master has been found!\n'

    # Get the user
    user = get_user()

    # Start ROS communications
    start_subscribers()

    # Grab control
    if gain_control():
        print '\nCongrats! You are now the Astrobee controller\n'
    else:
        print '\nUnable to grab control of Astrobee. Shutting down...'
        exit()

    # Get info from Guest Science Manager
    if not get_apk_info():
        print '\nUnable to communicate with the Guest Science Manager. Shutting down...'
        exit()

    time.sleep(3)

    while True:
        # Clear the screen
        clear()

        selection = None
        return_val = None

        # Select and app
        while selection == None:
            selection = select_app()
            if selection == -1:
                exit()
            clear()

        # Choose an action
        while return_val == None or return_val == ACTION_CONTINUE:
            return_val = execute_action(selection)
            if return_val == ACTION_CONTINUE:
                clear()

        if return_val == ACTION_EXIT:
            break

if __name__ == '__main__':
    main()
