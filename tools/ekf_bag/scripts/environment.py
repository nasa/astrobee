#!/usr/bin/python
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

import rospkg

import sys
import os
import os.path

# sets all needed environment variables. returns map and bag specified
# by environment variables, if any
def initialize_environment(astrobee_map, astrobee_bag):
  if astrobee_map == None and 'ASTROBEE_MAP' in os.environ:
    astrobee_map = os.environ['ASTROBEE_MAP']
  if astrobee_map == None:
    print >> sys.stderr, 'ASTROBEE_MAP not set.'
    sys.exit(0)
  if astrobee_bag == None and 'ASTROBEE_BAG' in os.environ:
    astrobee_bag = os.environ['ASTROBEE_BAG']
  if astrobee_bag == None:
    print >> sys.stderr, 'ASTROBEE_BAG not set.'
    sys.exit(0)

  robot_config = os.path.dirname(os.path.abspath(astrobee_bag))
  robot_config += '/robot.config'
  if not os.path.isfile(robot_config):
    print >> sys.stderr, 'Please create the robot config file %s.' % (robot_config)
    sys.exit(0)
  os.environ['ASTROBEE_ROBOT'] = robot_config
  os.environ['ASTROBEE_WORLD'] = 'granite'

  rospack = rospkg.RosPack()
  astrobee_path = ''
  try:
    astrobee_path = rospack.get_path('astrobee')
  except:
    print >> sys.stderr, 'Failed to find package astrobee.'
    sys.exit(0)

  os.environ['ASTROBEE_RESOURCE_DIR'] = astrobee_path + '/resources'
  os.environ['ASTROBEE_CONFIG_DIR'] = astrobee_path + '/config'
  
  return (astrobee_map, astrobee_bag)

