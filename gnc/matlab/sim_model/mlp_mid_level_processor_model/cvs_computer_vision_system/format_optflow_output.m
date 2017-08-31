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

%format_visOD_output.m

% inputs:
%   num_pts_out    = Required number of points to be reported
%   points_in iss  = ALL [x,y,z] position of points in the ISS frame
%   points_in_cam  = ALL [u,v] pixel location of points in the camera
%   valid_in       = indicates which points are valid
%   num_augments    = Required number of histories kept for Optical Flow (ASSUMED to be >=2)
%   flow_ids       = ID tag for each optical flow point (OF only)
%
% outputs:
%   observati_out  = output [u,v] pixel location of points in the camera
%   valid_out      = indicates which values of the output are valid!
%   ids_out        = optical flow IDs

function [observations_out, valid_out, ids_out, registration_pulse] = format_optflow_output(num_points_out, points_in_cam, valid_in, num_augmentations, flow_ids)
%#codegen

num_points_out          = 50;           %if number of points out changes, change this number.  
augmentation_order      = uint8([2 10 13 15]);    %order of the replacement augmentations
        %note: real FSW has a fancy way to determine the last augmentation to replace that changes between 13, 14 or 15.  Here we simplify to just take 15 every time
report_augmentations    = 0;
num_augmentations       = double(num_augmentations);
        
%Define Persistance Variables
persistent int_id_hist;
persistent int_observations;
persistent int_valid_flag;
persistent int_registration_number;
persistent int_initalization_complete;

%Either Initalize the variables (1st time only), or shift the history over
if isempty(int_id_hist)
    int_id_hist                                 = zeros(num_points_out, 1, 'single');
    int_observations                            = zeros(num_points_out, 2, num_augmentations, 'single');
    int_valid_flag                              = zeros(num_points_out, num_augmentations, 'uint8');
    int_registration_number                     = uint8(1);
    int_initalization_complete                  = 0;
else  
    %until 16 augmentations have been captured, just keep incrementing number.  Then after that, start rotating through the registration numbers
    if(int_initalization_complete<augmentation_order(4))   
        int_initalization_complete              = int_initalization_complete+1;
        int_observations                        = cat(3, zeros(num_points_out,2,1,'single'), int_observations(:,:,1:num_augmentations-1));
        int_valid_flag                          = cat(2, zeros(num_points_out,1,'single'), int_valid_flag(:,1:num_augmentations-1));
        int_registration_number                 = uint8(1);
    else
        switch int_registration_number                      %the registration pulse should rotated between augmentations [0, 2, 6, 15]
            case augmentation_order(1)
                int_registration_number                                         = augmentation_order(2); %remove the 3rd augmentation (zero based index)
                int_observations(1:num_points_out, 1:2,1:num_augmentations)     = cat(3, zeros(num_points_out,2,1,'single'), int_observations(:,:,1:int_registration_number-1), int_observations(:,:,int_registration_number+1:num_augmentations));
                int_valid_flag(1:num_points_out, 1:num_augmentations)           = cat(2, zeros(num_points_out,1,'single'), int_valid_flag(:,1:int_registration_number-1), int_valid_flag(:,int_registration_number+1:num_augmentations));
            case augmentation_order(2)
                int_registration_number                                         = augmentation_order(3); %remove the 7th augmentation (zero based index)
                int_observations(1:num_points_out, 1:2,1:num_augmentations)     = cat(3, zeros(num_points_out,2,1,'single'), int_observations(:,:,1:int_registration_number-1), int_observations(:,:,int_registration_number+1:num_augmentations));
                int_valid_flag(1:num_points_out, 1:num_augmentations)           = cat(2, zeros(num_points_out,1,'single'), int_valid_flag(:,1:int_registration_number-1), int_valid_flag(:,int_registration_number+1:num_augmentations));
            case augmentation_order(3)
                int_registration_number                                         = augmentation_order(4); %remove the last augmentation (zero based index)
                int_observations(1:num_points_out, 1:2,1:num_augmentations)     = cat(3, zeros(num_points_out,2,1,'single'), int_observations(:,:,1:num_augmentations-1));
                int_valid_flag(1:num_points_out, 1:num_augmentations)           = cat(2, zeros(num_points_out,1,'single'), int_valid_flag(:,1:num_augmentations-1));
            otherwise
                report_augmentations            = 1; %after capturing the last image, set flag to report
                int_registration_number                                         = augmentation_order(1); %remove the first augmentation  (zero based index)
                int_observations(1:num_points_out, 1:2,1:num_augmentations)     = cat(3, zeros(num_points_out,2,1,'single'), int_observations(:,:,2:num_augmentations));
                int_valid_flag(1:num_points_out, 1:num_augmentations)           = cat(2, zeros(num_points_out,1,'single'), int_valid_flag(:,2:num_augmentations));
        end
    end
end

%pull out the currently valid points as marked by the valid flag input
valid_ids                                       = single(flow_ids(valid_in==1,:));
valid_points_in_cam                             = single(points_in_cam(valid_in==1,:));

%Find unique (and valid) 2D points in the cam frame.  Can be thought of as either:
%   a) points blocked by other points are not visible to the camera, despite technically being in the FOV of the camera
%   b) Only can have 1 point reported per pixel location
[valid_points_in_cam, unique_rows, ~]           = unique(valid_points_in_cam,'rows', 'stable');   %finds unique rows, and 'stable' preserves the current order instead of sorting
valid_ids                                       = valid_ids(unique_rows);

%If any historical valid points are currently valid, capture this info
[~, hist_idx, new_idx]                          = intersect(int_id_hist, valid_ids);
if(~isempty(hist_idx))
    int_observations(hist_idx,:,1)              = valid_points_in_cam(new_idx,:);
    int_valid_flag(hist_idx,1)                  = ones(length(hist_idx),1,'uint8');
end

%Determine which points need to be replaced.  Keep any points that are still valid this history,
replace_these_points                            = (int_valid_flag(:,1)==0);
num_points_needed                               = min(num_points_out, sum(replace_these_points));

%Replace any points needed
if(num_points_needed > 0)
    %initialize new slots with zeros (since we may not fill it, want the remaining to be zeros already)
    new_ids                                     = zeros(num_points_needed, 1, 'single');
    new_observations                            = zeros(num_points_needed, 2, num_augmentations, 'single');
    new_valid_flags                             = zeros(num_points_needed, num_augmentations, 'uint8');
    
    %find the unused valid points (points not already in augmentations)
    [~,~,use_these_points]                      = setxor(int_id_hist, valid_ids);
    num_points_avail                            = length(use_these_points);
    
    if(num_points_avail > 0)
        %shuffle the points avaliable
        random_order                            = randperm(num_points_avail);
        shuffled_ids                            = use_these_points(random_order);
        shuffled_valid_points                   = valid_points_in_cam(shuffled_ids,:);
    
        %determine if num_points_avail>num_points needed, then verify its <num_points_out to make simulink happy
        num_points_to_use                           = min(num_points_needed, num_points_avail);
      
        %capture the points
        new_ids(1:num_points_to_use)                = valid_ids(shuffled_ids(1:num_points_to_use));
        new_observations(1:num_points_to_use,:,:)   = cat(3, shuffled_valid_points(1:num_points_to_use,:), zeros(num_points_to_use,2,num_augmentations-1,'single'));
        new_valid_flags(1:num_points_to_use,:)      = cat(2, ones(num_points_to_use, 1, 'uint8'), zeros(num_points_to_use, num_augmentations-1, 'uint8'));
    end
    
    int_id_hist(replace_these_points)               = new_ids(1:num_points_needed);
    int_observations(replace_these_points,:,:)      = new_observations(1:num_points_needed,:,:);
    int_valid_flag(replace_these_points,:)          = new_valid_flags(1:num_points_needed,:);
end

%sort the outputs by ID number
[int_id_hist, sort_idx]                         = sort(int_id_hist);
int_observations                                = int_observations(sort_idx,:,:);
int_valid_flag                                  = int_valid_flag(sort_idx,:);

valid_out                                       = zeros(num_points_out, num_augmentations, 'uint8'); 
%Determine which points to report
if(report_augmentations) %if we just replaced the last augmentation
    valid_out(:,augmentation_order+1)           = int_valid_flag(:,augmentation_order+1);
    report_augmentations                        = 0;
end %else send all zeros

%populate outputs
ids_out                                         = int_id_hist;
observations_out                                = int_observations;
registration_pulse                              = int_registration_number;
