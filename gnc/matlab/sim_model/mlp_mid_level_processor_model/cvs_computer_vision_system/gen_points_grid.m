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

function points = gen_points_grid(origin, dist, max_dist)
%returns a grid of points (including [0 0 0])
%spans from -max to + max with a distance between each point

i0 = origin(1);
j0 = origin(2);
k0 = origin(3);

points = [];
for (i=-max_dist+i0:dist:max_dist+i0)
    for(j=-max_dist+j0:dist:max_dist+j0)
        for(k=-max_dist+k0:dist:max_dist+k0)
            points = [points;[i,j,k]];
        end
    end
end

points = single(points);