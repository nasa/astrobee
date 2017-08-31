-- Copyright (c) 2017, United States Government, as represented by the
-- Administrator of the National Aeronautics and Space Administration.
--
-- All rights reserved.
--
-- The Astrobee platform is licensed under the Apache License, Version 2.0
-- (the "License"); you may not use this file except in compliance with the
-- License. You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
-- WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
-- License for the specific language governing permissions and limitations
-- under the License.

-- A set of utilities to manage 3D rotations

require "matrix"
require "string"

local matrix = require 'matrix'

function rot_matrix_z(angle)
  return matrix{
    {math.cos(angle), -math.sin(angle), 0.0},
    {math.sin(angle), math.cos(angle), 0.0},
    {0.0, 0.0, 1.0}
  }
end

function rot_matrix_y(angle)
  return matrix{
    {math.cos(angle), 0.0, math.sin(angle)},
    {0.0, 1.0, 0.0},
    {-math.sin(angle), 0.0, math.cos(angle)}
  }
end

function rot_matrix_x(angle)
  return matrix{
    {1.0, 0.0, 0.0},
    {0.0, math.cos(angle), -math.sin(angle)},
    {0.0, math.sin(angle), math.cos(angle)}
  }
end

function rot_matrix(angle, axis)
  rot = {}
  rot['X'] = rot_matrix_x
  rot['Y'] = rot_matrix_y
  rot['Z'] = rot_matrix_z
  return rot[string.upper(axis)](angle)
end

function matrix_equal(m1, m2)
  l1, c1 = matrix.size(m1)
  l2, c2 = matrix.size(m2)
  if (l1 ~= l2 or c1 ~= c2) then return false end
  for i=1,l1 do
    for j=1,c1 do
      if ( math.abs(m1[i][j]-m2[i][j]) > 1E-6 ) then return false end
    end
  end
  return true
end

function test_rotation_functions()

  m1 = matrix{{0.0, 1.0}, {2.0, 3.0}}
  m2 = matrix{{-1E-9, 1.0+1E-9}, {2.0, 3.0}}
  if not matrix_equal(m1, m2) then return false end

  m3 = matrix{{0.0, 1.0}, {2.1, 3.0}}
  if matrix_equal(m1, m3) then return false end

  m4 = matrix{{0.0, 1.0}, {2.0, 3.0}, {4.0, 5.0}}
  if matrix_equal(m1, m4) then return false end
  print "Test matrix_equal(): PASS"

  c30 = math.cos(math.rad(30))
  s30 = math.sin(math.rad(30))

  rx90_ref = matrix{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}
  rx90_test = rot_matrix(math.rad(90), 'x')
  if not matrix_equal(rx90_ref, rx90_test) then
    print("Test rx90 FAILED!")
    return false
  end

  rx30_ref = matrix{{1, 0, 0}, {0, c30, -s30}, {0, s30, c30}}
  rx30_test = rot_matrix(math.rad(30), 'x')
  if not matrix_equal(rx30_ref, rx30_test) then
    print("Test rx30 FAILED!")
    return false
  end

  ry90_ref = matrix{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}
  ry90_test = rot_matrix(math.rad(90), 'y')
  if not matrix_equal(ry90_ref, ry90_test) then
    print("Test ry90 FAILED!")
    return false
  end

  ry30_ref = matrix{{c30, 0, s30}, {0, 1, 0}, {-s30, 0, c30}}
  ry30_test = rot_matrix(math.rad(30), 'y')
  if not matrix_equal(ry30_ref, ry30_test) then
    print("Test ry30 FAILED!")
    return false
  end

  rz90_ref = matrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}
  rz90_test = rot_matrix(math.rad(90), 'z')
  if not matrix_equal(rz90_ref, rz90_test) then
    print("Test rz90 FAILED!")
    return false
  end

  rz30_ref = matrix{{c30, -s30, 0}, {s30, c30, 0}, {0, 0, 1}}
  rz30_test = rot_matrix(math.rad(30), 'z')
  if not matrix_equal(rz30_ref, rz30_test) then
    print("Test rz30 FAILED!")
    return false
  end

  print "Test rot_matrix(): PASS"

  return true
end

-- print(test_rotation_functions())
