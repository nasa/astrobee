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
import sys
from common import *

puml_path = os.path.relpath(sys.argv[1], os.getcwd())

plantuml_jar = './doc/diagrams/plantuml/plantuml.jar'

template = load_file('./doc/scripts/templates/diagram.template')

#
# Generate the PNG image from the PUML diagram
#
puml_dir = os.path.dirname(puml_path)
output_dir = os.path.normpath('./doc/html/'+puml_dir)
# plantuml '-output' option is always relative to the source file, 
# so we need to compute the generation directory relative to it!
relative_gen_dir = os.path.relpath(output_dir, puml_dir)
# print('relative_gen_dir = %s' % relative_gen_dir, file=sys.stderr)

os.system('java -jar %s -tpng -output %s %s' % \
    (plantuml_jar, relative_gen_dir, puml_path))

#
# Generate a page that includes the diagram image
#
name_no_ext = os.path.splitext(os.path.basename(puml_path))[0]
html_relative_path = os.path.join(puml_dir, name_no_ext+'.png')
package = puml_path.replace(os.getcwd(), '').split(os.sep)[0]
# print('html_relative_path = %s' % html_relative_path, file=sys.stderr)
data = {
    'package' : package,
    'diagram_name' : name_no_ext,
    'diagram_path' : html_relative_path
}

print(template % data)

