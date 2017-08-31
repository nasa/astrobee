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

%import_map_points.m
function map_points = import_map_points(filename)
fid     = fopen(filename,'r');
text    = textscan(fid,'%s','Delimiter',',');
text    = text{1};
fclose(fid);

numbers     = single(str2double(text));
num_points  = length(numbers)/3;
map_points  = reshape(numbers, [3,num_points])';

max_dist    = 50;             %mapping accidently finds points way too far away, lets remove those
[row, ~]  = find(abs(map_points) > max_dist);

remove_rows = unique(row);
map_points(remove_rows,:) = [];

end