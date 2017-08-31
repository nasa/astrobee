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

import os
import sys
from os.path import basename
from common import *

# What to search for
token = '/msg/'

# The full path to the file
path = sys.argv[1];
idx_dot = path.rfind('.');
idx_msg = path.find(token);
package = basename(path[:idx_msg])
message = basename(path[idx_msg+len(token):idx_dot]);

# Load the message template
template = load_file('./doc/scripts/templates/msg.template')
header = load_file('./doc/scripts/templates/license.template')

# Load the raw data describing the message
chunks = split_into_chunks(path)
if len(chunks) != 1:
  sys.stderr.write("File '%s' does not have one chunk" % (filename))
  sys.exit(1)

# Remove the license as well as any whitespace padding
chunks[0] = chunks[0].replace(header, "").rstrip().strip()

# Find the description
desc = ""
data = ""
state = 0 # 0: padding, 1: header, 2:data
for line in chunks[0].splitlines():
  rec = line.replace("#","").rstrip().strip()
  if len(rec) < 2:
    state += 1;
  if len(rec) > 0:
    if state == 1:
      desc += line.replace("#","").rstrip().strip() + " " 
    else:
      data += line + "\n" 
desc = desc.rstrip().strip()
data = data.rstrip().strip()

# This is the data that will be injected into the template
data = {'file': path,
        'extension': 'msg',
        'type': 'Message',
        'package': package,
        'message': message,
        'description': desc,
        'content': extract_message(data) }

print template % data