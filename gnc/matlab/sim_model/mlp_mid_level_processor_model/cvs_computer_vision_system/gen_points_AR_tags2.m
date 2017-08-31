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

% gen_points_AR_tags2.m
% Computes the position of AR targets on the dock in granite lab frame from
% the specified AR tag positions

function points_out = gen_points_AR_tags2()

unit_mm2m = 0.001;

%% Frames:
% AR frame    = AR tag frame - Centered in middle of AR tags
%          +X = right (when facing the tags)
%          +Y = up
%          +Z = out from the tags 
% Dock frame  = flat surface of the docks - Centered in middle of two docks
%          +X = away from dock
%          +Y = left (when facing dock)
%          +Z = down
% granite lab = room coordinate frame - Centered in middle of granite table
%          +X = toward wall with door
%          +Y = toward wall with windows
%          +Z = down

DCM_AR2dock             = [ 0, 0, 1; ...                                	%[DCM]  Converts points in AR frame to dock frame
                           -1, 0, 0; ...
                            0,-1, 0];
P_AR_dock_dock          = [0 0 0];                                         	%[m]    Position of the AR frame relative to the dock frame expressed in the dock frame                

DCM_dock2granite        = [ cosd(-25),        0,-sind(-25); ...             %[DCM]  Converts points in dock frame into the granite frame
                                    0,        1,         0; ...
                            sind(-25),        0, cosd(-25)];
P_dock_granite_granite  = [ -0.7053    0.3105   -0.8378];                  	%[m]    Position of dock frame relative to the granite frame expressed in the granite frame

%% AR tag locations
P_tags_AR_AR    = [  71,  48, 0;  91,  48, 0;  71,  28, 0;  91,  28, 0; ... %[m]    Position of AR tags relative to AR frame expressed in the AR frame
                    -91, -28, 0; -71, -28, 0; -91, -48, 0; -71, -48, 0; ... %       [top left; top right; bottom left; bottom right]
                    151, -15, 0; 171, -15, 0; 151, -35, 0; 171, -35, 0; ...
                   -171,  35, 0;-151,  35, 0;-171,  15, 0;-151,  15, 0; ...
                     91, -15, 0; 131, -15, 0;  91, -55, 0; 131, -55, 0; ...
                   -131,  55, 0; -91,  55, 0;-131,  15, 0; -91,  15, 0; ...
                    131,  48, 0; 171,  48, 0; 131,   8, 0; 131,   8, 0; ...
                   -171,  -8, 0;-131,  -8, 0;-171, -48, 0;-131, -48, 0; ...
                    -48,  48, 0;  48,  48, 0; -48, -48, 0;  48, -48, 0]*unit_mm2m;

%convert to dock frame
P_tags_dock_dock = DCM_AR2dock*P_tags_AR_AR' + repmat(P_AR_dock_dock', 1, length(P_tags_AR_AR)); % intermediate: P_tags_AR_dock = DCM_AR2dock*P_tags_AR_AR

%convert to granite frame
P_tags_granite_granite = DCM_dock2granite*P_tags_dock_dock + repmat(P_dock_granite_granite', 1, length(P_tags_AR_AR)); % intermediate: P_tags_dock_granite = DCM_dock2granite*P_tags_dock_dock

points_out = single(P_tags_granite_granite');
