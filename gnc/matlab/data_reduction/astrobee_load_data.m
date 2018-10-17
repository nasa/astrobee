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

% Astrobee Load Data
%
%
%%  Load CSVs

filepath = [pwd filesep];
%csv_file_list = cellstr(ls([filepath '*.csv']));
csv_file_list = dir([filepath '*.csv']);
csv_file_list = {csv_file_list.name};

if isempty(csv_file_list)
    fprintf(2,'No CSV data files found to process\n');
    return;
end

posixEpoch = '01/01/1970 00:00:00';

% Clear Structs to zero
clear out_*

simData = 0; % Set variable which tells the system this is data off the robot
vive_lock = false; % Semaphore to deal with truth data possibly coming from multiple sources

for current_file_num=1:length(csv_file_list)
    current_file_name = csv_file_list{current_file_num};        %find the full file name
    listing = dir(current_file_name);                           % Get info on the file
    if listing.bytes == 0  % If the file is empty,
        continue;           % skip it
    end
    
    if ~isempty(strfind(current_file_name, 'gnc_ekf'))
        rawData = importdata([filepath current_file_name],',',1);
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        
        % Sometimes we see duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_kfl_msg = [];
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
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,43:45), 'hr_P_hr_ISS_ISS', out_kfl_msg);
        out_kfl_msg = ab_populate_ts_field(time_data, rawData.data(:,46:49), 'hr_quat_ISS2hr', out_kfl_msg);
        
        mahal_dist = rawData.data(:,50:99);
        mahal_dist(mahal_dist == 0) = NaN;
        out_kfl_msg = ab_populate_ts_field(time_data, mahal_dist, 'ml_mahal_distance', out_kfl_msg);
        fprintf('Loaded %s\n', current_file_name);
        
    elseif ~isempty(strfind(current_file_name, 'gnc_ctl_command'))
        
        out_ctl_msg = [];
        if ~exist('out_cmd_msg', 'var'); out_cmd_msg = []; end
        
        rawData = importdata([filepath current_file_name],',',1);
        
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        
         % Sometimes we see duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3), 'body_force_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,4:6), 'body_torque_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,7:9), 'body_accel_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,10:12), 'body_alpha_cmd', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,13), 'ctl_status', out_ctl_msg);         
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,14:16), 'pos_err', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,17:19), 'pos_err_int', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,20:22), 'att_err', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,23:25), 'att_err_int', out_ctl_msg);
        out_ctl_msg = ab_populate_ts_field(time_data, rawData.data(:,26), 'att_err_mag', out_ctl_msg);
        
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,27), 'cmd_mode', out_cmd_msg);
        fprintf('Loaded %s\n', current_file_name);
        
    
    elseif ~isempty(strfind(current_file_name, 'vive'))
        out_env_msg = [];
        out_debug = [];
        vive_lock = true;
        rawData = importdata([filepath current_file_name],',',1);
        rawTimeData = rawData.data(:,1);
        time_data = ab_processTimeData(rawTimeData);   %capture time data (in textdata)
        % Some truth systems have duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        true_axis_angle_axis = rawData.data(:,5:7);
        true_axis_angle_angle = rssrow(rawData.data(:,5:7));
        true_axis_angle_axis = true_axis_angle_axis ./ repmat(true_axis_angle_angle, 1, 3);
        small_indx = find(abs(true_axis_angle_angle) < 1E-3);
        true_axis_angle_axis(small_indx,:) = repmat([1 0 0], length(small_indx), 1);
        true_axis_angle_angle(small_indx,:) = 0;
        truth_quat = axis_angle_to_quat(true_axis_angle_axis, true_axis_angle_angle);
        out_debug = ab_populate_ts_field(time_data, rawData.data(:,5:7), 'Att_Axis_Angle', out_debug);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,2:4), 'P_B_ISS_ISS', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, truth_quat, 'Q_ISS2B', out_env_msg);

        
        fprintf('Loaded %s\n', current_file_name);
        
    elseif ~isempty(strfind(current_file_name, 'loc_truth_pose'))
        if vive_lock
            fprintf('Using Vive Data instead of loc_truth_pose data')
            continue;
        end
        if ~exist('out_env_msg', 'var'); out_env_msg = []; end
        rawData = importdata([filepath current_file_name],',',1);
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        % Some truth systems have duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3), 'P_B_ISS_ISS', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,4:7), 'Q_ISS2B', out_env_msg);
        
        fprintf('Loaded %s\n', current_file_name);
 
        
    elseif ~isempty(strfind(current_file_name, 'loc_truth_twist'))
        if vive_lock
            fprintf('Using Vive Data instead of loc_truth_twist data')
            continue;
        end
        if ~exist('out_env_msg', 'var'); out_env_msg = []; end
        rawData = importdata([filepath current_file_name],',',1);
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        % Some truth systems have duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,1:3), 'V_B_ISS_ISS', out_env_msg);
        out_env_msg = ab_populate_ts_field(time_data, rawData.data(:,4:6), 'omega_B_ISS_B', out_env_msg);
        
        fprintf('Loaded %s\n', current_file_name);
    
        
    elseif ~isempty(strfind(current_file_name, 'hw_imu'))
        out_imu_msg = [];
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        
        out_imu_msg = ab_populate_ts_field(time_data, rawData.data(:,14:16), 'imu_omega_B_ECI_sensor', out_imu_msg);
        out_imu_msg = ab_populate_ts_field(time_data, rawData.data(:,26:28), 'imu_A_B_ECI_sensor', out_imu_msg);
        fprintf('Loaded %s\n', current_file_name);
        
       
        
        
        elseif ~isempty(strfind(current_file_name, 'hw_pmc_command'))
        out_act_msg = [];
        rawData = importdata([filepath current_file_name],',',1);
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        % Some truth systems have duplicate timestamps
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_act_msg = ab_populate_ts_field(time_data, rawData.data(:,[2:7, 9:14]), 'act_servo_pwm_cmd', out_act_msg);
        out_act_msg = ab_populate_ts_field(time_data, rawData.data(:,[1 8]), 'act_impeller_spped_cmd', out_act_msg);
        
        fprintf('Loaded %s\n', current_file_name);
        
    elseif ~isempty(strfind(current_file_name, 'gnc_control_feedback'))
        if ~exist('out_cmd_msg', 'var'); out_cmd_msg = []; end
        
        rawData = importdata([filepath current_file_name],',',1);   %import the data
        rawTimeData = rawData.textdata(2:end,3);
        mat_out = zeros(length(rawTimeData),1); for ii=1:length(rawTimeData); mat_out(ii) = str2double(rawTimeData{ii}); end
        time_data = ab_processTimeData(mat_out);   %capture time data (in textdata)
        
        duplicate_time_indx = find(diff(time_data) == 0);
        rawData.data(duplicate_time_indx,:) = [];
        time_data(duplicate_time_indx) = [];
        
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,3:5), 'traj_pos', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,6:9), 'traj_quat', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,10:12), 'traj_vel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,13:15), 'traj_omega', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,16:18), 'traj_accel', out_cmd_msg);
        out_cmd_msg = ab_populate_ts_field(time_data, rawData.data(:,19:21), 'traj_alpha', out_cmd_msg);
        
        fprintf('Loaded %s\n', current_file_name);
    end
    
    
end


astrobee_load_sim_data; % Convert to telemData

% Set the truth signals to be used as the masterTime base when
% interpolating
out_env_msg.Q_ISS2B.masterTime = true;
out_env_msg.P_B_ISS_ISS.masterTime = true;


%astrobee_post_process;
