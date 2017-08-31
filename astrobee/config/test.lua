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

require "matrix"

print("Hello world")

local matrix = require 'matrix'
tun_ffp_veh_inertia_matrix = matrix{{.3277, 0, 0}, {0, .3277, 0}, {0, 0, .5303}};

print("inertia matrix type: ", type(tun_ffp_veh_inertia_matrix));
print("size: ", table.getn(tun_ffp_veh_inertia_matrix));
print("i[1] type: ",  type(tun_ffp_veh_inertia_matrix[1]));
print("i[1] size: ", table.getn(tun_ffp_veh_inertia_matrix[1]));
print(tun_ffp_veh_inertia_matrix[1]);
