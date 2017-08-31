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

function points = gen_points_wall(origin, norm_vec, point_delta, num_points)
%returns a wall of points
% origin = wall center
% norm_vec = vector normal to the wall
% point delta = density of points
% num points = total number of points to have

cnt    = sqrt(num_points)/2;
    
%find two vectors in the plane 
if(acosd(abs(dot(norm_vec, [1 0 0]))) > 10)
    plane_vec1 = cross(norm_vec,[1 0 0]);
else
    plane_vec1 = cross(norm_vec,[0 1 0]);
end
plane_vec2 = cross(norm_vec, plane_vec1);

%use those vectors to populate the points
points = [];
for(j=-point_delta*cnt:point_delta:point_delta*cnt)
    for(k=-point_delta*cnt:point_delta:point_delta*cnt)
        new_points = [plane_vec1*j + plane_vec2*k + origin];
        points = [points; new_points];
    end
end

points = unique(points,'rows');
points = single(points);