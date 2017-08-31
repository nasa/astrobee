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

%csv_computer_vision_system_prep.m
%% Noise Warning
if ~tun_cvs_noise_on
    warning('Vision System Noise OFF')
end

%% Sample Times
cvs_landmark_process_time   = round((1/cvs_landmark_process_hz)/astrobee_time_step_size)*astrobee_time_step_size;
cvs_AR_process_time         = round((1/cvs_AR_process_hz)/astrobee_time_step_size)*astrobee_time_step_size;
cvs_optflow_process_time    = round((1/cvs_optflow_process_hz)/astrobee_time_step_size)*astrobee_time_step_size;
cvs_handrail_process_time   = round((1/cvs_handrail_process_hz)/astrobee_time_step_size)*astrobee_time_step_size;

cvs_optflow_ids             = single(1:size(cvs_optflow_map_iss,1))';                             %[-] Assign ID numbers to every optical flow point
cvs_handrail_vector         = quat_rotation_vec(cvs_handrail_vector_nom,eulers_to_quat(cvs_handrail_misalign(3),cvs_handrail_misalign(2),cvs_handrail_misalign(1))); %[-] misalign the handrail
cvs_handrail_vector         = cvs_handrail_vector/norm(cvs_handrail_vector);

%map error:
temp_stored_rng             = rng; %save off and change rng
rng(cvs_noise_seed+cvs_noise_seed);
cvs_landmark_1s_noise       = sqrt((cvs_landmark_max_error^2)/3)/3;         %[m] 1sigma value for noise on position of landmarks 
cvs_landmark_map_error      = single(randn(size(cvs_landmark_map_iss))*cvs_landmark_1s_noise);                %[m] error to be added to the map
cvs_AR_1s_noise             = sqrt((cvs_AR_max_error^2)/3)/3;               %[m] 1sigma value for noise on position of AR tags
cvs_AR_map_error            = single(randn(size(cvs_AR_map_iss))*cvs_AR_1s_noise);                      %[m] error to be added to the map
cvs_optflow_1s_noise        = sqrt((cvs_optflow_max_error^2)/3)/3;          %[m] 1sigma value for noise on position of world points 
cvs_optflow_map_error       = single(randn(size(cvs_optflow_map_iss))*cvs_optflow_1s_noise);                 %[m] error to be added to the map
cvs_handrail_1s_noise       = sqrt((cvs_handrail_max_error^2)/3)/3;         %[m] 1sigma value for noise on position of landmarks 
cvs_handrail_map_error      = single(randn(size(cvs_handrail_map_iss))*cvs_handrail_1s_noise);                %[m] error to be added to the map
rng(temp_stored_rng); %restore rng values

%Noise values: assume worst case (max) position error is if all [xyz] are equally wrong. Then divide by 3 to find 1 sigma value
cvs_landmark_pixel_noise    = sqrt((cvs_landmark_max_pixel_error^2)/3)/3;   %[pixels] 1sigma value for pixel noise on landmarks
cvs_AR_pixel_noise          = sqrt((cvs_AR_max_pixel_error^2)/3)/3;         %[pixels] 1sigma value for pixel noise on AR tags
cvs_optflow_pixel_noise     = sqrt((cvs_optflow_max_pixel_error^2)/3)/3;    %[pixels] 1sigma value for pixel noise on optical flow
cvs_handrail_pixel_noise    = sqrt((cvs_landmark_max_pixel_error^2)/3)/3;   %[pixels] 1sigma value for pixel noise on handrails

%adjust the given valid mask times to the simulation start time
cvs_landmark_valid_times    = [cvs_landmark_valid_times(:,1)+ini_time_seconds, cvs_landmark_valid_times(:,2)+ini_time_nanoseconds];
cvs_AR_valid_times          = [cvs_AR_valid_times(:,1)+ini_time_seconds, cvs_AR_valid_times(:,2)+ini_time_nanoseconds];
cvs_optflow_valid_times     = [cvs_optflow_valid_times(:,1)+ini_time_seconds, cvs_optflow_valid_times(:,2)+ini_time_nanoseconds];
cvs_handrail_valid_times    = [cvs_handrail_valid_times(:,1)+ini_time_seconds, cvs_handrail_valid_times(:,2)+ini_time_nanoseconds];

