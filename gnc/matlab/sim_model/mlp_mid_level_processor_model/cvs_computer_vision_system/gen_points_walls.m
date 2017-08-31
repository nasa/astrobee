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

function points = gen_points_walls(wall_dist, point_delta)
%returns a set of walls surrounding the origin
	%will create a box with length 'dist'

walls = [ 1  0  0; ...
         -1  0  0; ...
          0  1  0; ...
          0 -1  0; ...
          0  0  1; ...
          0  0 -1]*wall_dist;

points = [];

for(j=-wall_dist:point_delta:wall_dist)
    for(k=-wall_dist:point_delta:wall_dist)
        new_points = [0,j,k; ...
                      0,j,k; ...
                      j,0,k; ...
                      j,0,k; ...
                      j,k,0; ...
                      j,k,0];
        points = [points; new_points+walls];
    end
end

points = unique(points,'rows');
points = single(points);