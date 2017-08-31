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

% Load rosbag data in the current working directory
% To Do:
%   Add delta of valid features (i.e. rejected features)
%%  Load CSVs

% Hack to deal with offsets in the dock room frame coordinates
% truth_offset = [.009166 -0.0253, .00753];
truth_offset = [0 0 0];

filepath = [pwd filesep];
%csv_file_list = cellstr(ls([filepath '*.csv']));
csv_file_list = dir([filepath '*.csv']);
csv_file_list = {csv_file_list.name};

posixEpoch = '01/01/1970 00:00:00';
% Sort of a hack
%  % Initialize Structs to zero
out_cmd_msg = [];
out_act_msg_vpp = [];
out_ctl_msg = [];
out_env_msg = [];
out_kfl_msg = [];
out_cvs_registration_pulse = [];
out_imu_msg = [];
out_cvs_landmark_msg = [];
out_act_msg = [];

simData = 0; % Set variable which tells the system this is data off the robot

for current_file_num=1:length(csv_file_list)
    current_file_name = csv_file_list{current_file_num};        %find the full file name
    listing = dir(current_file_name);                           % Get info on the file
    if listing.bytes == 0  % If the file is empty,
        continue;           % skip it
    end
%     dot_index = strfind(current_file_name,'.');                 %find the name of the data
%     underscore_indx = strfind(current_file_name,'_');  
%     current_file_ID = [current_file_name(1:dot_index(end)-1)];
    
    
    if ~isempty(regexp(current_file_name, 'pmc_actuator_command.csv', 'ONCE'))
        rawData = importdata(current_file_name);
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        out_act_msg = ab_populate_ts_field(time_data, rawData.data(:,[1,8]), 'act_impeller_speed_cmd', out_act_msg);
        out_act_msg = ab_populate_ts_field(time_data, rawData.data(:,[2:7,9:14]), 'act_servo_pwm_cmd', out_act_msg);
        % Need to verify order of servos
        
    
    
    elseif ~isempty(regexp(current_file_name, 'localization_mapped_landmarks_features.csv', 'ONCE'))
        try 
        rawData = importdata(current_file_name);
        time_data = ab_processTimeData(rawData.data(:,1));
        validPoints = isfinite(rawData.data(:,13:5:258));
        out_cvs_landmark_msg = ab_populate_ts_field(time_data, validPoints, 'traj_pos', out_cvs_landmark_msg);
        catch
            fprintf(2, 'Unable to import localization mapped landmark features\n');
        end
                
                
    elseif ~isempty(regexp(current_file_name, 'ctl_traj.csv', 'ONCE'))
        rawData = csvread(current_file_name, 1, 0);
        time_data = ab_processTimeData(rawData(:,1));
        
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,3:5), 'traj_pos', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,6:9), 'traj_quat', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,10:12), 'traj_vel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,13:15), 'traj_omega', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,16:18), 'traj_accel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,19:21), 'traj_alpha', out_cmd_msg);
        
        
    elseif ~isempty(regexp(current_file_name, 'ctl_shaper.csv', 'ONCE'))
        rawData = csvread(current_file_name, 1, 0);
        time_data = ab_processTimeData(rawData(:,1));
        
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,3:5), 'cmd_pos', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,6:9), 'cmd_quat', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,10:12), 'cmd_vel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,13:15), 'cmd_omega', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,16:18), 'cmd_accel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData(:,19:21), 'cmd_alpha', out_cmd_msg);
        
        
%     elseif ~isempty(regexp(current_file_name, 'mapped_landmarks_registration.csv', 'ONCE'))
%         rawData = csvread(current_file_name, 1, 0);
%         time_data = ab_processTimeData(rawData(:,1));
%         
%         out_cvs_registration_pulse = ab_populate_ts_field(time_data, rawData(:,2), 'out_cvs_registration_pulse', out_cvs_registration_pulse);
%         
        
        
    elseif ~isempty(regexp(current_file_name, 'ekf.csv', 'ONCE'))
        rawData = importdata([filepath current_file_name],',',1); 
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3), 'P_B_ISS_ISS', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,4:7), 'quat_ISS2B', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,8:10), 'V_B_ISS_ISS', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,11:13), 'omega_B_ISS_B', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,14:16), 'gyro_bias', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,17:19), 'A_B_ISS_ISS', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,20:22), 'accel_bias', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,23:37), 'cov_diag', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,38), 'confidence', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,39), 'aug_state_enum', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,40), 'kfl_status', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,41), 'update_OF_tracks_cnt', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,42), 'update_ML_features_cnt', out_kfl_msg);
        
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,43:45), 'ml_P_cam_ISS_ISS', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,46:48), 'ml_quat_ISS2cam', out_kfl_msg);
        
        mahal_dist = rawData.data(:,78:127); 
        mahal_dist(mahal_dist == 0) = NaN;
        out_kfl_msg = ab_populate_ts_field(time_data, mahal_dist, 'ml_mahal_distance', out_kfl_msg);
        
        
        
        
        
    elseif ~isempty(regexp(current_file_name, 'ctl_command.csv', 'ONCE'))
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3), 'body_force_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,4:6), 'body_torque_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,7), 'ctl_status', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,8:10), 'pos_err', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,11:13), 'pos_err_int', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,14:16), 'att_err', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,17:19), 'att_err_int', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,20), 'att_err_mag', out_ctl_msg);

        
        
    elseif ~isempty(regexp(current_file_name, 'truth.csv', 'ONCE')) % files that are not just numbers
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3) + repmat(truth_offset,length(rawData.data), 1) , 'P_B_ISS_ISS', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,4:7), 'Q_ISS2B', out_env_msg);
        fprintf(2, 'Warning: A truth offset of %f, %f, %f was applied to the truth signal\n',  truth_offset);
        
    elseif ~isempty(regexp(current_file_name, 'truth_accel.csv', 'ONCE')) % files that are not just numbers
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3) + repmat(truth_offset,length(rawData.data), 1) , 'A_B_ISS_B', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,4:6), 'alpha_B_ISS_B', out_env_msg);
        fprintf(2, 'Warning: A truth offset of %f, %f, %f was applied to the truth signal\n',  truth_offset);
        
    elseif ~isempty(regexp(current_file_name, 'truth_twist.csv', 'ONCE')) % files that are not just numbers
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        %out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3) + repmat(truth_offset,length(rawData.data), 1) , 'A_B_ISS_B', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,4:6), 'omega_B_ECI_B', out_env_msg);
        fprintf(2, 'Warning: A truth offset of %f, %f, %f was applied to the truth signal\n',  truth_offset);
%     elseif ~isempty(regexp(current_file_name, 'fam.csv', 'ONCE')) % files that are not just numbers
%         rawData = importdata([filepath current_file_name],',',1);   %import the data
%         time_data = str2num(cell2mat(rawData.textdata(2:end,1)));   %capture time data (in textdata)
%         current_file_ID = 'fam';
    
    elseif ~isempty(regexp(current_file_name, 'hw_imu.csv', 'ONCE'))
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        time_data = ab_processTimeData(str2num(cell2mat(rawData.textdata(2:end,1))));   %capture time data (in textdata)
        
        out_imu_msg = ab_populate_ts_field(time_data, rawData.data(:,14:16), 'imu_omega_B_ECI_sensor', out_imu_msg);
        out_imu_msg = ab_populate_ts_field(time_data, rawData.data(:,26:28), 'imu_A_B_ECI_sensor', out_imu_msg);
        
        
   
    else
        fprintf('Unknown File: %s\n', current_file_name)
    end
    
end

% Converts time series to telemData
astrobee_load_sim_data;