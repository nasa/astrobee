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


def typerlink(typ):
    tok = typ.replace("[", "")
    tok = tok.replace("]", "")
    tok = tok.split("/")
    if len(tok) == 1:
        return typ
    package = tok[0].replace("_", "__")
    message = tok[1].replace("_", "__")
    if tok[0].strip() in ("ff_msgs", "ff_hw_msgs", "vpp_msgs"):
        return '<a href="./group__%s__%s.html">%s</a>' % (package, message, typ)
    return typ


# Load and return a message template
def split_into_chunks(filename):
    chunks = []
    buff = ""
    for line in open(filename):
        if line.startswith("---"):
            chunks.append(buff)
            buff = ""
        else:
            buff += line
    chunks.append(buff)
    return chunks


def extract_message(str):
    variables = []
    constants = []
    desc = ""
    for line in str.splitlines():
        rec = line.strip()
        if len(rec) == 0:
            continue
        if rec[0] == "#":
            desc += line.replace("#", "").strip() + " "
        else:
            idx = rec.find("#")
            if idx >= 0:
                desc = rec[idx + 1 :].strip()
                rec = rec[:idx]
            lst = rec.split("=")
            tok = (" ".join(lst[0].split())).split()
            dtype = tok[0].strip()
            dname = tok[1].strip()
            if len(lst) > 1:
                dname += " = " + lst[1].strip()
                constants.append(
                    "| %s | **%s** | %s |" % (typerlink(dtype), dname, desc)
                )
                desc = ""
            else:
                variables.append("| %s | `%s` | %s |" % (typerlink(dtype), dname, desc))
                desc = ""

    buff = ""
    for line in variables:
        buff += line + "\n"
    for line in constants:
        buff += line + "\n"
    if len(buff) == 0:
        buff = "| | | |"
    return buff


# Load and return a message template
def load_file(filename):
    if not os.path.isfile(filename):
        sys.stderr.write("Cannot locate file '%s'\n" % (filename))
        sys.exit(1)
    with open(filename, "r") as f:
        content = f.read()
        if not content:
            sys.stderr.write("File '%s' is empty\n" % (filename))
            sys.exit(1)
        return content
