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

"""
A command-line tool for generating an HTML command dictionary from
an XPJSON schema.
"""

import argparse
import logging
import os
import sys

# hack to ensure xgds_planner2 submodule is at head of PYTHONPATH
ffroot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ffroot, "astrobee", "commands", "xgds_planner2"))

from xgds_planner2 import commandDictionary


def main():
    parser = argparse.ArgumentParser(
        description=__doc__ + "\n\n",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "inSchemaPath",
        help="input XPJSON schema path",
    )
    parser.add_argument(
        "outHtmlPath",
        help="output HTML command dictionary path",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG, format="%(message)s")
    commandDictionary.writeCommandDictionary(
        args.inSchemaPath,
        args.outHtmlPath,
        includeCommandSpecNameField=False,
        includeCommandSpecNotesField=False,
    )


if __name__ == "__main__":
    main()
