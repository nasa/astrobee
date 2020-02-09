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


function [pos, att, num_features, t] = import_ml_features(fname)

fid = fopen(fname);
tline = fgetl(fid);
tline = fgetl(fid);

pos = [];
att = [];
num_features = [];
t = [];

while ischar(tline)
  arr = regexp(tline,'\,+','split'); 
  current_pos = str2double(arr(6:8));
  t = [t; str2double(arr{1})];
  current_pos = str2double(arr(6:8));
  current_att = str2double(arr(9:12));
  
  if all([current_pos current_att] == 0) % If there are no features pos and att are all zero
      num_features = [num_features; 0];
      pos = [pos; [NaN NaN NaN]];
      att = [att; [NaN NaN NaN NaN]];
  else
      num_features = [num_features; (length(arr)-12)/5];
      pos = [pos; current_pos];
      att = [att; current_att];
  end
  
  tline = fgetl(fid);
end