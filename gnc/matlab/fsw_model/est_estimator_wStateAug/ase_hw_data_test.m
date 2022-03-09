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

% Define the data path and truth paths respectively by defining the following variables in the workspace:
% data_path_in
% truth_path_in

warning off MATLAB:table:ModifiedVarnames;
if ~exist('data_path_in', 'var')
    data_path =  'ekf_inputs.csv';
    truth_path = 'ground_truth.csv';
else
    data_path = data_path_in;
    truth_path = truth_path_in;
end


% reload parameters if changed
astrobee_version = 'p4';
if ~exist('disable_config_overwrite', 'var') || ~disable_config_overwrite
    tunable_init;
    ase_augmented_state_estimator_init;
    ase_augmented_state_estimator_prep;
end

numOF_pts = 50;
numML_pts = 50;

%% Telem Data
rawData = csvread(data_path);
%rawData = rawData(1:6000, :);
mlpTime = rawData(:,1) + rawData(:,2)*1E-9;
mlpTime = mlpTime(mlpTime > 0);
mlpTime = mlpTime - mlpTime(1);

%% Truth Data
truthData = readtable(truth_path); % read as a table becasue there is text
truthData = table2cell(truthData);
if size(truthData) == [0 0]
    truthTime = [];
    truthPos = [];
    truthQuat = [];
else
    truthTime = [truthData{:,1}]'*1E-9;
    truthPos = cell2mat(truthData(:,5:7));
    truthQuat = cell2mat(truthData(:, 8:11));
    %% Sync up truth data
    truthTime = truthTime - truthTime(1);
    %truthTime = truthTime + mlpTime(end) - truthTime(end);
    
    out_env_msg.P_B_ISS_ISS = timeseries(truthPos, truthTime);
    out_env_msg.Q_ISS2B = timeseries(truthQuat, truthTime);
    
end

% Bus ICs
cvs_registration_pulse_IC = Simulink.Bus.createMATLABStruct('cvs_registration_pulse');

n = size(rawData,1);
%of1
% ase_accel_fixed_bias = single([-0.0275    0.0339   -9.8173]);
% ase_gyro_fixed_bias = single(1e-3 * [-1.1599    0.0446   -0.5281]);
%of2
%ase_accel_fixed_bias = single([-0.0156    0.0360   -9.8146]);
%ase_gyro_fixed_bias = single(1e-3 * [-1.1653    -0.01316   -0.7607]);

%original_flat_fast
% ase_accel_fixed_bias = single([-0.0076    0.03759   -9.8149]);
% ase_gyro_fixed_bias = single(1e-3 * [-1.0787    0.00   0.4470]);
ase_accel_fixed_bias = single([-0.1425    0.0852   -9.8201]);
ase_gyro_fixed_bias = single(1e-3 * [-1.0787    0.00   0.4470]);

% ase_enable_of = 1;
% ase_Q_imu(10:12,10:12) = 1E-12*eye(3,3,'single');
% ase_of_r_mag = single((10 * ase_inv_focal_length)^2);

% Might have to find the lowest time of all the times
% imu_t = rawData(:,1)+rawData(:,2)*1E-9;
% t0 = imu_t(1);
% imu_t = imu_t - t0;
%imu_t = mlpTime;
% we have to do this because otherwise matlab deletes points that are
% slighlty off...
imu_t = 0:0.016:0.016* (size(mlpTime, 1) - 1);
% if we don't do this matlab idiotically rounds things with floating point
% errors, causing us to drop registration pulses
imu_t = imu_t + 1e-6;
truthTime = truthTime + imu_t(end) - mlpTime(end);

% we have to add a repeat of the first at zero because matlab is the
% dumbest program ever and will randomly add zeros as the first entry
% causing us to rapidly accelerate in z
test_imu_timestamp_sec = timeseries([0; uint32(rawData(:,1))], [0 imu_t]);
test_imu_timestamp_nsec = timeseries([0; uint32(rawData(:,2))], [0 imu_t]);
test_imu_A_B_ECI_sensor = timeseries([single(rawData(1,6:8)); single(rawData(:,6:8))], [0 imu_t]);
test_omega_B_ECI_sensor = timeseries([single(rawData(1,3:5)); single(rawData(:,3:5))], [0 imu_t]);


% No timestamp for registration found, using IMU timestamp for now
reg_t =  imu_t;
test_cvs_landmark_pulse = timeseries(uint8(rawData(:, 9)), reg_t);
test_cvs_optical_flow_pulse = timeseries(uint8(rawData(:,10)), reg_t);


% ml_t = rawData(:,11)+rawData(:,12)*1E-9;
% ml_t = ml_t - t0;
ml_t = imu_t;

test_cvs_timestamp_sec = timeseries(uint32(rawData(:,11)), ml_t);
test_cvs_timestamp_nsec = timeseries(uint32(rawData(:,12)), ml_t);

landmark_pos = single(rawData(:,13:15));
landmark_quat = single(rawData(:,16:19));
cam_to_body = tun_abp_q_body2navcam;
cam_to_body(4) = -cam_to_body(4);
landmark_quat = quatmult(landmark_quat, repmat(cam_to_body, size(landmark_quat, 1), 1));
landmark_pos = landmark_pos - quat_rotation_vec(repmat(tun_abp_p_navcam_body_body_sim, size(landmark_pos, 1), 1), landmark_quat); % Look at this value closer
landmark_pos = timeseries(landmark_pos, ml_t);
landmark_quat = timeseries(landmark_quat, ml_t);
landmarks = single(rawData(:,20:169));
test_cvs_landmarks = timeseries(reshape(landmarks',50,3, n), ml_t);
test_obs = single(rawData(:,170:269));
test_cvs_observations = timeseries(reshape(test_obs',50,2, n), ml_t);
test_cvs_valid_flag = timeseries(uint8(rawData(:,270:319)), ml_t);
if exist('disable_ml', 'var') && disable_ml
    test_cvs_valid_flag.Data(:, :) = 0;
end
%test_cvs_valid_flag.Data(40 * 62.5:end, :) = 0;
%test_cvs_landmark_pulse.Data(1000:end, :) = 0;

% of_t = rawData(:,313)+rawData(:,314)*1E-9;
% of_t = of_t - t0;
of_t = imu_t;


test_OF_cvs_timestamp_sec = timeseries(uint32(rawData(:,320)), of_t);
test_OF_cvs_timestamp_nsec = timeseries(uint32(rawData(:,321)), of_t);

OF_obs = single(rawData(:,322:1921));
test_OF_cvs_observations = timeseries(reshape(OF_obs', 50,2,16,n), of_t);

OF_valid = uint8(rawData(:,1922:2721));

% % handrail part
hr_t = imu_t;
test_hr_pulse = timeseries(uint8(rawData(:, 2722)), hr_t);
test_hr_timestamp_sec = timeseries(uint32(rawData(:,2723)), hr_t);
test_hr_timestamp_nsec = timeseries(uint32(rawData(:,2724)), hr_t);

test_hr_obs = single(rawData(:,2725:2874));
test_hr_observations = timeseries(reshape(test_hr_obs',50, 3, n), hr_t);
test_hr_valid_flag = timeseries(uint8(rawData(:,2875:2924)), hr_t);
test_hr_rep_valid_flag = timeseries(uint8(rawData(:,2875)), hr_t);
hr_local_pos = single(rawData(:,2925:2927));
test_hr_local_pos = timeseries(reshape(hr_local_pos',3,1, n), hr_t);
hr_local_quat = single(rawData(:,2928:2931));
test_hr_local_quat = timeseries(reshape(hr_local_quat',4,1, n), hr_t);

test_hr_3d_knowledge_flag = timeseries(uint8(rawData(:,2932)), hr_t);
test_hr_update_global_pose_flag = timeseries(uint8(rawData(:,2933)), hr_t);
	
test_localization_mode_cmd = timeseries(uint8(rawData(:,2934)), hr_t);

% invalid_corner = OF_obs >
test_OF_cvs_valid_flag = timeseries(reshape(OF_valid', 50,16,n), of_t);
if exist('disable_of', 'var') && disable_of
    test_OF_cvs_valid_flag.Data(:, :) = 0;
end

%% Configure Initial Conditions
if size(truthPos, 1) > 1
    tun_ase_state_ic_P_B_ISS_ISS = single(truthPos(1,:));
    tun_ase_state_ic_quat_ISS2B = single(truthQuat(1,:));
else
    first_obs_row = find(landmark_pos.Data(:, 1), 1);
    tun_ase_state_ic_P_B_ISS_ISS = landmark_pos.Data(first_obs_row, :);
    tun_ase_state_ic_quat_ISS2B = landmark_quat.Data(first_obs_row, :);
end

q_ISS2B = quaternion(tun_ase_state_ic_quat_ISS2B);
p_imu_body_iss = rotate_b2a(q_ISS2B,tun_abp_p_imu_body_body);
tun_ase_state_ic_P_EST_ISS_ISS = tun_ase_state_ic_P_B_ISS_ISS + p_imu_body_iss;

%% Setup Sim Parameters
test_stop_time = imu_t(end);

%% Run Sim
if ~exist('show_results', 'var') || show_results
    tic;
end
sim('ase_augmented_state_estimator_hrn');
if ~exist('show_results', 'var') || show_results
    toc;
end

% prev_times_sec = test_cvs_timestamp_sec.Data(1:end-1);
% prev_times_nsec = test_cvs_timestamp_nsec.Data(1:end-1);
% changed = zeros(size(test_cvs_timestamp_sec.Data, 1), 1);
% changed(2:end) = or(prev_times_sec ~= test_cvs_timestamp_sec.Data(2:end), ...
%                     (prev_times_nsec ~= test_cvs_timestamp_nsec.Data(2:end)));
% ml_update = changed;
ml_update = out_kfl_msg.kfl_status.Data == 1;
of_update = out_kfl_msg.kfl_status.Data == 2;
ml_times = out_kfl_msg.ml_mahal_distance.Time(ml_update);
of_times = out_kfl_msg.of_mahal_distance.Time(of_update);
reg_times = ml_times;
% find the registration times of the previous registration pulse
indices = find(ml_update);
for i=1:size(ml_times, 1)
    reg_time = 1;
    for j = indices(i)-1:-1:1
        if out_cvs_reg_pulse.cvs_landmark_pulse.Data(j)
            reg_time = out_cvs_reg_pulse.cvs_landmark_pulse.Time(j);
            break;
        end
    end
    reg_times(i) = reg_time;
end

landmark_pos = getsamples(landmark_pos, ml_update);
landmark_quat = getsamples(landmark_quat, ml_update);

landmark_pos.Time = reg_times;
landmark_quat.Time = landmark_pos.Time;

%% Plots
if exist('fusco_plots', 'var')
    figure; plot(out_env_msg.P_B_ISS_ISS, '--', 'LineWidth', 2)
    hold_on; plot_shaded_error_bar(out_kfl_msg.P_B_ISS_ISS.Time, out_kfl_msg.P_B_ISS_ISS.Data, sqrt(out_kfl_msg.cov_diag.Data(:,13:15))); title('Position'); ylabel('m');
   
    figure; plot_shaded_error_bar(out_kfl_msg.accel_bias.Time, out_kfl_msg.accel_bias.Data, sqrt(out_kfl_msg.cov_diag.Data(:,10:12))); title('Accel Bias'); ylabel('m/s^2');
    figure; plot_shaded_error_bar(out_kfl_msg.accel_bias.Time, out_kfl_msg.gyro_bias.Data, sqrt(out_kfl_msg.cov_diag.Data(:,4:6))); title('Gyro Bias'); ylabel('rad/s');
    
end

% find the time difference between truth and ekf
if size(truthTime, 1) > 0
    truthOffset = 0;
    angular_rmse = inf;
    for i=-3:0.1:3
        new_times = resample(out_kfl_msg.quat_ISS2B, truthTime+i);
        errors = quat_error(new_times.Data, truthQuat) / 1000 * 180 / pi;
        errors(isnan(errors)) = [];
        error = sqrt(sum(errors.^2) / size(errors, 1));
        if error < angular_rmse
            angular_rmse = error;
            truthOffset = i;
        end
    end
    truthTime = truthTime + truthOffset;
    
    pos_truth_times = resample(out_kfl_msg.P_B_ISS_ISS, truthTime);
    pos_errors = sqrt(sum((pos_truth_times.Data - truthPos)' .^ 2));
    pos_errors(isnan(pos_errors)) = [];
    pos_rmse = sqrt(sum(pos_errors.^2) / size(pos_errors, 2));
    if ~exist('show_results', 'var') || show_results
        fprintf('Pos RMSE: %g Quat RMSE: %g\n', pos_rmse, angular_rmse);
    end
end

if exist('show_graphs', 'var') && show_graphs
    figure; plot(out_kfl_msg.P_B_ISS_ISS);
    hold_on; plot(truthTime, truthPos, '--'); plot(landmark_pos, 'd'); hold off;
    
    figure; plot(out_kfl_msg.quat_ISS2B.Time, quat_to_eulers(out_kfl_msg.quat_ISS2B.Data) * 180 / pi);
    hold_on; plot(truthTime, quat_to_eulers(truthQuat) * 180 / pi, 'x');
    plot(landmark_quat.Time, quat_to_eulers(landmark_quat.Data) * 180 / pi, 'd'); hold off;
    
    nonzero_ml = out_kfl_msg.update_ML_features_cnt.Data(:, 1)> 0;
    nonzero_of = out_kfl_msg.update_OF_tracks_cnt.Data(:, 1) > 0;
    title('Number of Observations');
    figure; plot(out_kfl_msg.update_ML_features_cnt.Time(nonzero_ml), out_kfl_msg.update_ML_features_cnt.Data(nonzero_ml), 'ro');
    hold on; plot(out_kfl_msg.update_OF_tracks_cnt.Time(nonzero_of), out_kfl_msg.update_OF_tracks_cnt.Data(nonzero_of), 'bx'); hold off;
    
    figure; hold on;
    title('ML Mahalanobis Distances');
    boxplot(out_kfl_msg.ml_mahal_distance.Data(ml_update, :)', 'labels', ml_times, 'positions', ml_times);
    hold off;
%     figure; hold on;
%     title('OF Mahalanobis Distances');
%     boxplot(out_kfl_msg.of_mahal_distance.Data(of_update, :)', 'labels', of_times, 'positions', of_times);
%     hold off;
end
