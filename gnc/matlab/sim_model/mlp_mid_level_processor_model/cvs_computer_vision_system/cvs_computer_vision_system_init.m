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

%csv_computer_vision_system_init.m

%% Camera Parameters
% camera positions set in gnc.config
% camera orientations set in gnc.config
% image width/height set in gnc.config
% image focal lengths set in gnc.config
% image offsets set in gnc.config

cvs_navcam_pointing             = single([0, 0, 1]);                            %[unit vector] Vector pointing in the direction of the camera FOV, relative to the camera frame
cvs_navcam_P_B_B_error          = single([0 0 0]);                              %[m] Error on the position of the camera in the body frame 
cvs_navcam_Q_B2navcan_error     = single(eulers_to_quat(0, 0, 0));              %[m] Error on the Quaternion from camera to body frame
cvs_navcam_min_dist             = single(.2);                                   %[m] min distance a point can be seen by the camera
cvs_navcam_max_dist             = single(100);                                  %[m] max distance a point can be seen by the camera

cvs_dockcam_pointing            = single([0, 0, 1]);                            %[unit vector] Vector pointing in the direction of the camera FOV, relative to the camera frame
cvs_dockcam_P_B_B_error         = single([0 0 0]);                              %[m] Error on the position of the camera in the body frame
cvs_dockcam_Q_B2dockcam_error   = single(eulers_to_quat(0, 0, 0));              %[m] Error on the Quaternion from camera to body frame
cvs_dockcam_min_dist            = single(.2);                                   %[m] min distance a point can be seen by the camera
cvs_dockcam_max_dist            = single(100);                                  %[m] max distance a point can be seen by the camera

cvs_perchcam_pointing           = single([0, 0, 1]);                            %[unit vector] Vector pointing in the direction of the camera FOV, relative to the camera frame
cvs_perchcam_P_B_B_error        = single([0 0 0]);                              %[m] Error on the position of the camera in the body frame
cvs_perchcam_Q_B2perchcam_error = single(eulers_to_quat(0, 0, 0));              %[m] Error on the Quaternion from camera to body frame
cvs_perchcam_min_dist           = single(.2);                                   %[m] min distance a point can be seen by the camera
cvs_perchcam_max_dist           = single(100);                                  %[m] max distance a point can be seen by the camera

%% Image processing parameters
cvs_landmark_process_hz     = 3;                                            %[Hz] Time interval at which landmark images are processed
cvs_landmark_num_out        = single(50);                                   %[-] Number of landmark observations to send
cvs_landmark_map_filename   = [ASTROBEE_ROOT '/sim_model/mlp_mid_level_processor_model/cvs_computer_vision_system/granite_table_map_points.txt'];
cvs_landmark_map_iss        = import_map_points(cvs_landmark_map_filename); %[m] Imports the landmark points relative to the ISS frame
% cvs_landmark_map_iss        = gen_points_grid([0 0 0],2,10);                %[m] Generates the fixed world points relative to the ISS coordinate frame
cvs_landmark_max_pixel_error= 2;                                            %[pixels] maximum noise error in pixels - Note: must be type double
cvs_landmark_max_error      = 0.05;                                         %[m] maximum position error in map - Note: must be type double
cvs_landmark_valid_mask     = [true, true, true];                        	%[-] true = use model's output, false = force to zero, testing only
cvs_landmark_valid_times    = uint32([0,0; 30,0; 31,0]);                    %[s,ms] Timing to use cooresponding valid_masks, testing only

cvs_AR_process_hz           = 6;                                            %[Hz] Time interval at which AR images are processed
cvs_AR_num_out              = cvs_landmark_num_out;                         %[-] Number of AR tag observations to report
% cvs_AR_map_iss              = gen_points_AR_tags([0,-.5,-.4], [0,1,0]);     %[m] Wall perpendicular to Y-axis, origin [0,-.5,-.4] 
cvs_AR_map_iss              = gen_points_AR_tags2;                          %[m] Generate fixed AR tags for granite table 
cvs_AR_max_pixel_error      = 2;                                            %[pixels] maximum noise error in pixels - Note: must be type double
cvs_AR_max_error            = 0.01;                                         %[m] maximum position error in map - Note: must be type double
cvs_AR_valid_mask           = [true, true, true];                           %[-] true = use model's output, false = force to zero, testing only
cvs_AR_valid_times          = uint32([0,0; 30,0; 31,0]);                    %[s,ms] Timing to use cooresponding valid_masks, testing only

cvs_optflow_process_hz      = 3.6765;                                           %[Hz] Time interval at which OF images are processed
cvs_optflow_num_out         = single(50); %NOTE: must change in format_optflow_output.m as well %[-] Number of OF observations to keep (number of points)
cvs_optflow_num_hist        = single(16);                                    %[-] Number of histories to keep for the optical flow
cvs_optflow_map_iss         = gen_points_walls(4.5,.35);                    %[m] Generates the fixed world points relative to the ISS coordinate frame
cvs_optflow_max_pixel_error = .5;    % 0.2                                      %[pixels] maximum noise error in pixels - Note: must be type double
cvs_optflow_max_error       = 0.01;   %optflow has 0 map error              %[m] maximum position error in map - Note: must be type double
cvs_optflow_valid_mask     = [true, true, true];                        	%[-] true = use model's output, false = force to zero, testing only
cvs_optflow_valid_times    = uint32([0,0; 30,0; 31,0]);                    %[s,ms] Timing to use cooresponding valid_masks, testing only

cvs_handrail_process_hz     = 5;                                            %[Hz] Time interval at which Handrail images are processed
cvs_handrail_num_out        = cvs_landmark_num_out;                         %[-] Number of handrail observations to report
cvs_handrail_vector_nom     = single([0,1,0]);                              %[-] Vector of the pointing direction of the handrail
cvs_handrail_misalign       = single([0;0;0]);                              %[-] handrail misalignment
cvs_handrail_map_iss        = gen_points_rail_wall([-.4,0,-.4],cvs_handrail_vector_nom,[-1,0,0],.05,1,.1); %[m] Generates the fixed world points relative to the ISS coordinate frame
cvs_handrail_3d_knowledge   = uint8(1);                                     % flag that allows the filter to include knowledge along the handrail vector
cvs_handrail_noise_var      = 0.01^2;                                       %[-] Variance of error for the 3D cam output from the handrail model
cvs_handrail_max_pixel_error= 2;                                            %[pixels] maximum noise error in pixels - Note: must be type double
cvs_handrail_max_error      = 0.05;                                         %[m] maximum position error in map - Note: must be type double
cvs_handrail_valid_mask     = [true, true, true];                        	%[-] true = use model's output, false = force to zero, testing only
cvs_handrail_valid_times    = uint32([0,0; 30,0; 31,0]);                    %[s,ms] Timing to use cooresponding valid_masks, testing only

%% General Noise Parameters
%tun_cvs_noise_on set in gnc.config
cvs_noise_seed              = 123;                                          %[unitless] seed for noise generator

%% create the null messages for landmarks/handrails since only one can be sent at a time
cvs_landmark_null_message.cvs_timestamp_sec     = zeros(1,'uint32');
cvs_landmark_null_message.cvs_timestamp_nsec    = zeros(1,'uint32');
cvs_landmark_null_message.cvs_landmarks         = zeros(50,3,'single');
cvs_landmark_null_message.cvs_observations      = zeros(50,2,'single');
cvs_landmark_null_message.cvs_valid_flag        = zeros(50,1,'uint8');

cvs_handrail_null_message.cvs_timestamp_sec     = zeros(1,'uint32');
cvs_handrail_null_message.cvs_timestamp_nsec    = zeros(1,'uint32');
cvs_handrail_null_message.cvs_landmarks         = zeros(50,3,'single');
cvs_handrail_null_message.cvs_observations      = zeros(50,3,'single');
cvs_handrail_null_message.cvs_valid_flag        = zeros(50,1,'uint8');
cvs_handrail_null_message.cvs_3d_knowledge_flag = zeros(1,1,'uint8');
cvs_handrail_null_message.cvs_handrail_local_pos = zeros(3,1,'single');
cvs_handrail_null_message.cvs_handrail_local_quat = single([0 0 0 1]');
cvs_handrail_null_message.cvs_handrail_update_global_pose_flag = zeros(1,1,'uint8');
