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

import re, sys

def find_close_parenthese(s):
  assert(s[0] == '(')
  count = 1
  for i in range(1, len(s)):
    if s[i] == '(':
      count += 1
    elif s[i] == ')':
      count -= 1
    if count == 0:
      return i
  return -1

def find_close_bracket(s):
  assert(s[0] == '{')
  count = 1
  for i in range(1, len(s)):
    if s[i] == '{':
      count += 1
    elif s[i] == '}':
      count -= 1
    if count == 0:
      return i
  return -1

def lua_find_transform(text, transform_name):
  # Search for lines starting with the transform name, ignoring, for
  # example, lines starting with lua comments, that is, with "--".
  prog = re.compile('(^|\n\s*)' + re.escape(transform_name) + '\s*=\s*transform\(')
  match = re.search(prog, text)
  if not match:
    return (None, None, None)
  open_bracket = match.end() - 1
  close_bracket = find_close_parenthese(text[open_bracket:])
  if close_bracket < 0:
    return (None, None, None)
  close_bracket = open_bracket + close_bracket
  transform_text = text[open_bracket:close_bracket+1]
  return (transform_text, open_bracket, close_bracket)

def lua_read_transform(filename, transform_name):
  try:
    with open(filename, 'r') as f:
      contents = f.read()
  except IOError:
    return None
  (transform_text, open_bracket, close_bracket) = lua_find_transform(contents, transform_name)
  prog = re.compile('\(vec3\((.*)\),\s*quat4\((.*)\)\s*\)')
  m = re.match(prog, transform_text)
  if m == None:
    print >> sys.stderr, "Could not extract the transform from string: %s " % transform_text
    return None
  trans = map(float, map(lambda x: x.strip(), m.group(1).split(',')))
  quat = map(float, map(lambda x: x.strip(), m.group(2).split(',')))
  t = quat[0]
  return (trans, quat)

def lua_replace_transform(filename, transform_name, transform):
  try:
    with open(filename, 'r') as f:
      contents = f.read()
  except IOError:
    print >> sys.stderr, 'Failed to open lua file.'
    return True
  (transform_text, open_bracket, close_bracket) = lua_find_transform(contents, transform_name)
  if transform_text == None:
    print >> sys.stderr, 'Failed to read field %s from %s.' % (transform_name, filename)
    return True

  new_transform_text = '(vec3(%.8g, %.8g, %.8g), quat4(%.8g, %.8g, %.8g, %.8g))' % \
          (transform[0][0], transform[0][1], transform[0][2], \
           transform[1][0], transform[1][1], transform[1][2], transform[1][3])
  output_text = contents[:open_bracket] + new_transform_text + contents[close_bracket+1:]

  print("Updating " + transform_name + " in file: " + filename)
  try:
    with open(filename, 'w') as f:
      f.write(output_text)
  except IOError:
    print >> sys.stderr, 'Failed to open lua file ' + filename
    return True
  return False
