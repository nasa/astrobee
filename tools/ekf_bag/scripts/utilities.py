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

import datetime
import glob
import os
import rospkg

# Forward errors so we can recover failures
# even when running commands through multiprocessing
# pooling
def full_traceback(func):
  import traceback, functools

  @functools.wraps(func)
  def wrapper(*args, **kwargs):
    try:
      return func(*args, **kwargs)
    except Exception as e:
      msg = "{}\n\nOriginal {}".format(e, traceback.format_exc())
      raise type(e)(msg)

  return wrapper

def get_files(directory, file_string):
  return glob.glob(os.path.join(directory, file_string))

def create_directory(directory=None):
  if directory == None:
    directory = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
  if os.path.exists(directory):
    print(directory + " already exists!")
    exit()
  os.makedirs(directory)
  return directory

def get_gnc_config(gnc_config):
  if gnc_config == None:
    #Load from astrobee/config/gnc.confg
    astrobee_path = rospkg.RosPack().get_path('astrobee')
    gnc_config = os.path.join(astrobee_path, 'config/gnc.config')
  else:
    if not os.path.exists(gnc_config):
      print("GNC config does not exists! {}".format(gnc_config))
      exit()

  return gnc_config
