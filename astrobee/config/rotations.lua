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

--[[
  Create a rotation matrix from ZYX Euler angles
  ZYX Euler angles represent a rotation around the moving axes in this order:
    - rotate around Z by the angle alpha
    - rotate around Y' (rotated Y) by the angle beta
    - rotate around X" (rotated X) by the angle gamma
  Note that this is the exact same transform around the fixed axes (XYZ static):
    - rotate around X by the angle gamma
    - rotate around Y by the angle beta
    - rotate around Z by the angle alpha
    --> RXYZ(gamma, beta, alpha) = RZ(alpha)*RY(beta)*RX(gamma)
--]]
function rot_matrix_from_euler(alpha, beta, gamma)
  ca = math.cos(alpha)
  cb = math.cos(beta);
  cg = math.cos(gamma);
  sa = math.sin(alpha);
  sb = math.sin(beta);
  sg = math.sin(gamma);
  return matrix{
    {ca*cb, ca*sb*sg-sa*cg, ca*sb*cg+sa*sg},
    {sa*cb, sa*sb*sg+ca*cg, sa*sb*cg-ca*sg},
    {-sb, cb*sg, cb*cg}
  }
end
euler2rmat = rot_matrix_from_euler

-- normalize a quaternion
function quat4_normalize(q)
  n = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
  return quat4(q.x/n, q.y/n, q.z/n, q.w/n)
end

--[[
  Create a rotation matrix from a quaternion
--]]
function rot_matrix_from_quat4(q)
  qn = quat4_normalize(q)
  x = qn.x
  y = qn.y
  z = qn.z
  s = qn.w
  qx2 = qn.x*qn.x
  qy2 = qn.y*qn.y
  qz2 = qn.z*qn.z
  qw2 = qn.w*qn.w
  return matrix {
    {1-2*qy2-2*qz2, 2*x*y-2*s*z, 2*x*z+2*s*y},
    {2*x*y+2*s*z, 1-2*qx2-2*qz2, 2*y*z-2*s*x},
    {2*x*z-2*s*y, 2*y*z+2*s*x, 1-2*qx2-2*qy2}
  }
end
quat2rmat = rot_matrix_from_quat4

--[[
  Create a rotation matrix from a quaternion
  (computer graphics solution by Shoemake 1994)
--]]
function quat4_from_rot_matrix(m)
  tr = m[1][1] + m[2][2] + m[3][3]
  q = quat4(0,0,0,1)
  if ( tr >= 0 ) then
    s4 = 2.0 * math.sqrt( tr + 1.0 ) -- s4 = 4*qw
    q.w = s4 / 4.0
    q.x = ( m[3][2] - m[2][3] ) / s4
    q.y = ( m[1][3] - m[3][1] ) / s4
    q.z = ( m[2][1] - m[1][2] ) / s4
  elseif ( m[1][1]>m[2][2] and m[1][1]>m[3][3]) then
    s4 = 2.0* math.sqrt( 1.0 + m[1][1] - m[2][2] - m[3][3] ) -- s4 = 4*qx
    q.w = ( m[3][2] - m[2][3] ) / s4
    q.x = s4 / 4.0
    q.y = ( m[1][2] + m[2][1] ) / s4
    q.z = ( m[3][1] + m[1][3] ) / s4
  elseif ( m[2][2] > m[3][3] ) then
    s4 = 2.0 * math.sqrt( 1.0 + m[2][2] - m[1][1] - m[3][3] ) -- s4 = 4*qy
    q.w = ( m[1][3] - m[3][1] ) / s4
    q.x = ( m[1][2] + m[2][1] ) / s4
    q.y = s4 / 4.0
    q.z = ( m[2][3] + m[3][2] ) / s4
  else
    s4 = 2.0 * math.sqrt( 1.0 + m[3][3] - m[1][1] - m[2][2]) -- s4 = 4*qz
    q.w = ( m[2][1] - m[1][2] ) / s4
    q.x = ( m[3][1] + m[1][3] ) / s4
    q.y = ( m[2][3] + m[3][2] ) / s4
    q.z = s4 / 4.0
  end
  return q
end
rmat2quat= quat4_from_rot_matrix

function printquat(q)
  print("quat(x,y,z,w)=(" .. q.x .. "," .. q.y .. "," .. q.z .. "," ..  q.w ..")")
end
