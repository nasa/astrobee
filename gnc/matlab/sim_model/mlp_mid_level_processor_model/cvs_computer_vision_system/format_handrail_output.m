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

%format_handrail_output.m

% inputs:
%   num_pts_out    = Required number of points to be reported
%   points_in_iss  = ALL [x,y,z] position of points in the ISS frame
%   points_in_cam  = ALL [u,v] pixel location of points in the camera
%   valid_in       = indicates which points are valid
%
% outputs:
%   points_out_iss = output [x,y,z] position of points in the ISS frame     
%   points_out_cam = output [x,y,z] pixel location of points in the camera frame
%   valid_out      = indicates which values of the output are valid!

function [points_out_iss, points_out_cam, valid_out] = format_handrail_output(num_pts_out, points_in_iss, points_in_cam, valid_in)
%#codegen

%initialize the outputs
points_out_iss                      = zeros(num_pts_out,3,'single');
points_out_cam                      = zeros(num_pts_out,3,'single');
valid_out                           = zeros(num_pts_out,1,'single');

%find how many pts are valid, and determine how that compares to required number
valid_pts                           = (valid_in==1);
num_valid                           = length(valid_in(valid_pts));
needed                              = num_pts_out-num_valid;

if(needed>=0)       %if exactly the right ammount or need more, just report what we got! (rest will be zeros)
    points_out_iss(1:num_valid,:) =   points_in_iss(valid_pts,:);
    points_out_cam(1:num_valid,:) =   points_in_cam(valid_pts,:);
    valid_out(1:num_valid,:)      =   valid_in(valid_pts);
else                %if we have too many points, take a random sample
    temp                            =   points_in_iss(valid_pts,:); 
    random_order                    =   randperm(length(temp));
    points_out_iss(1:num_pts_out,:) =   temp(random_order(1:num_pts_out),:);
    temp                            =   points_in_cam(valid_pts,:);
    points_out_cam(1:num_pts_out,:) =   temp(random_order(1:num_pts_out),:);
    valid_out(1:num_pts_out,:)      =   ones(num_pts_out,1);
end
