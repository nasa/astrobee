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

# from __future__ import print_function

import os
import re
import sys

from common import *

doc_dir = os.path.normpath(
    os.path.join(os.path.abspath(os.path.dirname(sys.argv[0])), "../")
)

puml_path = os.path.relpath(sys.argv[1], os.getcwd())
name_no_ext = os.path.splitext(os.path.basename(puml_path))[0]

plantuml_jar = doc_dir + "/diagrams/plantuml/plantuml.jar"

template = load_file(doc_dir + "/scripts/templates/diagram.template")

#
# Extract diagram width
#
max_width = 900
svg = open(doc_dir + "/diagrams/gen/svg/" + name_no_ext + ".svg", "r")
first = svg.readline()
pattern = re.compile('viewBox="0 0 ([0-9]+) ([0-9]+)"')
result = re.search(pattern, first)
if not result:
    print("width not found!")
    width = max_width
else:
    width = int(result.group(1))

if width > max_width:
    scaled_width = max_width
else:
    scaled_width = width

#
# Generate the PNG image from the PUML diagram
#
output_dir = os.path.join(doc_dir + "/html")
# plantuml '-output' option is always relative to the source file,
# so we need to compute the generation directory relative to it!
relative_gen_dir = os.path.relpath(output_dir, os.path.dirname(puml_path))

# Disable diagram generation: assume it was done at a previous stage
# (The Mafile is more efficient and avoid CPU intensive re-generation)
# os.system('java -jar %s -tpng -output %s %s' % \
#     (plantuml_jar, relative_gen_dir, puml_path))

#
# Generate a page that includes the diagram image
#
# html_relative_path = os.path.join('./', name_no_ext+'.png')
html_relative_path = os.path.join("../diagrams/gen/svg", name_no_ext + ".svg")
package = puml_path.replace(os.getcwd(), "").split(os.sep)[0]


data = {
    "package": package,
    "diagram_name": name_no_ext,
    "diagram_path": html_relative_path,
    "width": scaled_width,
}

print((template % data))
