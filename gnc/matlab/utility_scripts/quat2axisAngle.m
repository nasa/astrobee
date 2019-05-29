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

function [axis, angle] = quat2axisAngle(q)
% Derived from Eq. 2.124 in "Fundamentals of Spacecraft Attitude and
% Determination and Control", Markley and Crassidis
angle = 2*acos(q(:,4));
sin_angle = sin(angle);
axis = q(:,1:3)./sin_angle;

% Find any place the angle is close to 180, and approximate using the
% quaternion vector postion
axis(angle > 179*pi/180, :) = q(angle > 179*pi/180,1:3);

% Find any place you have a null quaternion and set the axis to x
min_angle_indx = find(angle < 1E-12);
axis(min_angle_indx, :) = repmat([1 0 0], length(min_angle_indx), 1);

axis = axis./rssrow(axis);


