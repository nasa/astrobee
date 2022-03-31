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


def check_and_fill_line(value_map, config_file_line):
    line_strings = config_file_line.split()
    # Overwrite val if config variable is in value map
    if len(line_strings) > 0 and line_strings[0] in value_map:
        return line_strings[0] + " = " + str(value_map[line_strings[0]]) + "\n"
    return config_file_line


def fill_in_values(original_config, value_map, new_config):
    original_config_file = open(original_config, "r")
    new_config_file = open(new_config, "w")
    for config_file_line in original_config_file:
        new_config_file.write(check_and_fill_line(value_map, config_file_line))


def make_value_map(values, value_names):
    value_map = {}
    if len(values) != len(value_names):
        print("values and value_names not same length!")
        exit()

    for index, value_name in enumerate(value_names):
        value_map[value_name] = values[index]

    return value_map


def make_config(values, value_names, original_config, new_config):
    value_map = make_value_map(values, value_names)
    fill_in_values(original_config, value_map, new_config)
