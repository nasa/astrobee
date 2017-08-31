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

% Astrobee augmented state estimator (ase) initialization file.  Configures
% parameters used in the library file augmented_state_estimator.

%% Number of Augmented States
ase_of_num_aug = Simulink.Parameter(int32(16)); % Change here and need to update, cov_diag, of_quat_ISS2cam, and of_P_cam_ISS_ISS in the kfl_msg
ase_of_num_aug.CoderInfo.StorageClass = 'Custom';
ase_of_num_aug.CoderInfo.CustomStorageClass = 'Define';
ase_of_num_aug.CoderInfo.Alias = 'ASE_OF_NUM_AUG';

%also, change the cvs_optflow_num_hist, as well as the optical flow bus def
ase_ml_num_features = Simulink.Parameter(int32(50));
ase_ml_num_features.CoderInfo.StorageClass = 'Custom';
ase_ml_num_features.CoderInfo.CustomStorageClass = 'Define';
ase_ml_num_features.CoderInfo.Alias = 'ASE_ML_NUM_FEATURES';

ase_of_num_features = Simulink.Parameter(int32(50));
ase_of_num_features.CoderInfo.StorageClass = 'Custom';
ase_of_num_features.CoderInfo.CustomStorageClass = 'Define';
ase_of_num_features.CoderInfo.Alias = 'ASE_OF_NUM_FEATURES';

ase_aug_state_bitmask = uint32(bitsll(1,ase_of_num_aug.Value+1)-2);  %bitmask used in est_estimator/camera_update to determine if all augmented states are valid

%% Known Bias Parameters
ase_gyro_fixed_bias             = single([0 0 0]);
ase_accel_fixed_bias            = single([0 0 0]);

%% Data Types
ase_cov_datatype = Simulink.AliasType;
ase_cov_datatype.BaseType = 'single';

%% Predictor Parameters
ase_ts                          = astrobee_fsw_step_size;                   %[sec]

%% State enumnerations
ase_status_converged            = uint8(0);
ase_status_acquiring            = uint8(1);
ase_status_diverged             = uint8(2);

ase_local_mode_map              = uint8(0);
ase_local_mode_docking          = uint8(1);
ase_local_mode_perching         = uint8(2);

% KFL Status
% 1 = ML Update
% 2 = OF Update

%% Earth Parameters (for ground testing only)
ase_earth_rate                  = single(0*[0, 0, 1]*((360/23.9344699)/3600)*pi/180);   % Earth rate, rad/sec, in ECEF
ase_pos_vec                     = single(0*1.0e+06*[-2.6922, -4.2987, 3.8541]);         % N269 Lab position in ECEF (lla2ecef([37.415117, -122.058621, 0]))

%% Noise Parameters              

% Handrail Depth Sensor Errors
ase_hr_distance_r               = 0.01;                                     % [meters]
ase_hr_r_mag                    = 0.005;                                    % [meters]

 %% Measurments
ase_H                           = single([eye(4) zeros(4,12);zeros(3,13), eye(3)]);
ase_H_error                     = single([eye(3) zeros(3,12);zeros(3,12), eye(3)]);

ase_minumum_resid_thresh        = 1E-2; % Magnitude of the residual below which it is no longer worh it to itterate on the update step
 
ase_max_update_iterations       = 1;                                    
ase_max_of_update_iterations    = 1;                                    
 
 %% Initial Conditions
% tun_ase_state_ic_quat_ISS2B   = single([0 0 0 1]); moved to gnc.config
% tun_ase_state_ic_omega_B_ISS_B = single([0 0 0]); moved to gnc.config
ase_state_ic_gyro_bias = single([0 0 0]);
% tun_ase_state_ic_V_B_ISS_ISS = single([0 0 0]); moved to gnc.config
ase_state_ic_A_B_ISS_ISS = single([0 0 0]);
ase_state_ic_accel_bias = single([0 0 0]);
% ase_state_ic_P_B_ISS_ISS = single([0 0 0]); moved to gnc.config
ase_state_ic_confidence = ase_status_converged; % change this
ase_state_ic_aug_state_enum = uint8(0);
ase_state_ic_ml_quat_ISS2cam = single([0 0 0 1]);
ase_state_ic_ml_P_cam_ISS_ISS = single([0 0 0]);
ase_state_ic_status = ase_status_acquiring;
 
%% Indices
% Indices inside the covariance (P) matrix which are associated with the IMU, Mapped Landmarks, and Optical Flow
ase_imu_p_indx                  = 1:15;
ase_ml_p_indx                   = 16:21;
% [Q bias_gyro V bias_accel P]

