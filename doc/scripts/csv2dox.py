#!/usr/bin/env python
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

import csv
import os
import sys

from common import *


def hyperlink(url, name):
    return '<a href="%s">%s</a>' % (url, name)


# Grab the CSV content
content = ""
header = True
with open(sys.argv[1], "rb") as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",", quotechar='"')
    for row in csvreader:
        if header:
            header = False
        else:
            content += (
                "| "
                + hyperlink(row[2], row[1])
                + " | "
                + row[3]
                + " | "
                + row[4]
                + " |\n"
            )

# This is the data that will be injected into the template
data = {"content": content}

# Load the message template
template = load_file("./doc/scripts/templates/csv.template")

# Populate the template
print((template % data))
