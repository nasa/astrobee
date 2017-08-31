% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.

function points = gen_points_rail_wall(origin, rail_vector, wall_vector, point_delta, rail_length, rail_diam)
%returns a set of points looking like a rail in front of a wall
% origin = center of rail
% rail vector = direciton rail is pointed
% wall vector = direciton from rail to wall
% delta = distance from one point to next
% length = total length of rail
% diam = diameter of the rail

rail_vector     = (rail_vector/norm(rail_vector))';
point_spread    = -rail_length/2:point_delta:rail_length/2;

left_dir        = cross(rail_vector, wall_vector);
right_dir       = -left_dir;

points_center   = repmat(origin,length(point_spread),1)' + rail_vector*point_spread;
points_left     = repmat(origin+(left_dir*rail_diam),length(point_spread),1)' + rail_vector*point_spread;
points_right    = repmat(origin+(right_dir*rail_diam),length(point_spread),1)' + rail_vector*point_spread;

points_wall     = gen_points_wall(origin+(rail_diam*2*wall_vector), wall_vector, point_delta,(rail_length/point_delta)^2);

points          = [points_left'; points_center'; points_right'; points_wall];
end
