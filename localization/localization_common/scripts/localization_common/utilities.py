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
import subprocess

import pandas as pd


# Forward errors so we can recover failures
# even when running commands through multiprocessing
# pooling
def full_traceback(func):
    import functools
    import traceback

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


def get_files_recursive(directory, file_string):
    subdirectory_csv_files = []
    _, subdirectories, _ = next(os.walk(directory))
    for subdirectory in subdirectories:
        subdirectory_path = os.path.join(directory, subdirectory)
        for subdirectory_csv_file in get_files(subdirectory_path, file_string):
            subdirectory_csv_files.append(subdirectory_csv_file)
    return subdirectory_csv_files


def create_directory(directory=None):
    if directory == None:
        directory = os.path.join(
            os.getcwd(), datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        )
    if os.path.exists(directory):
        print((directory + " already exists!"))
        exit()
    os.makedirs(directory)
    return directory


def load_dataframe(files):
    dataframes = [pd.read_csv(file) for file in files]
    dataframe = pd.concat(dataframes)
    return dataframe


def run_command_and_save_output(command, output_filename, print_command=True):
    if print_command:
        print(command)
    with open(output_filename, "w") as output_file:
        subprocess.call(command, shell=True, stdout=output_file, stderr=output_file)


def basename(filename):
    return os.path.splitext(os.path.basename(filename))[0]
