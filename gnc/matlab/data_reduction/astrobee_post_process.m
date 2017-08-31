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

%% Post process the astrobee data
% Takes in simulation or astrobee data.  Simulation data can be packed by
% running astrobee_load_sim_data, actual test data can be packaged into this
% format by running astrobee_load_test_data
%
% Currently assumes the data is packed in time series structures in the
% workspace.  Supports:
% 'overview' - Overview plots
% 'est' - Estimater Specific plots
% 'cmd' - Command shaper plots
% 'mahal' - Plot mahalanobis distance
% 'stats' - generate stats
% 'traj_ctl' - Plots internal system errors off the trajector (shaper + ctl)
% 'docking' - 
% 'saveMCdata' - Save off monte carlo test data
% 
%
% To Do:
% Add rejected count due to mahal
%
%
function astrobee_post_process(varargin)
%% Import From Base
% The try statement is cheesy but easy
try; out_ex_time_msg = evalin('base', 'out_ex_time_msg'); end;
try; out_cvs_reg_pulse = evalin('base', 'out_cvs_reg_pulse'); end;
try; out_cvs_landmark_msg = evalin('base', 'out_cvs_landmark_msg'); end;
try; out_cvs_optflow_msg = evalin('base', 'out_cvs_optflow_msg'); end;
try; out_cvs_handrail_msg = evalin('base', 'out_cvs_handrail_msg'); end;
try; out_cmc_msg = evalin('base', 'out_cmc_msg'); end;
try; out_imu_msg = evalin('base', 'out_imu_msg'); end;
try; out_env_msg = evalin('base', 'out_env_msg'); end;
try; out_vpp_msg = evalin('base', 'out_vpp_msg'); end;
try; out_bpm_msg = evalin('base', 'out_bpm_msg'); end;
try; out_kfl_msg = evalin('base', 'out_kfl_msg'); end;
try; out_kfl_P = evalin('base', 'out_kfl_P'); end;
try; out_cmd_msg = evalin('base', 'out_cmd_msg'); end;
try; out_act_msg_vpp = evalin('base', 'out_act_msg_vpp'); end;
try; out_act_msg = evalin('base', 'out_act_msg'); end;
try; out_ctl_msg = evalin('base', 'out_ctl_msg'); end;
astrobee_version = evalin('base', 'astrobee_version');
simData = evalin('base', 'simData');
try; current_file_name = evalin('base', 'current_file_name'); end;



%% Configure
%simData = 1;
estPlots = 1;
saveFigs = 0;
writeStats = 0;

% Determine which data sets exist in the workspace
ds_on.cmd = exist('out_cmd_msg', 'var')  && ~isempty(out_cmd_msg);
ds_on.env = exist('out_env_msg', 'var') && ~isempty(out_env_msg);
ds_on.kfl = exist('out_kfl_msg', 'var') && ~isempty(out_kfl_msg);
ds_on.act = exist('out_act_msg', 'var') && ~isempty(out_act_msg);

% Run any init files we need variables from
ASTROBEE_ROOT = evalin('base', 'ASTROBEE_ROOT');
tunable_init;

%% Parse Input Arguments
plotConfig = [];
if nargin == 0
    plotConfig.overview = 1;
else
    if any(strcmpi(varargin, 'overview')); plotConfig.overview = 1; end
    if any(strcmpi(varargin, 'est')); plotConfig.est = 1; end
    if any(strcmpi(varargin, 'ctl')); plotConfig.ctl = 1; end
    if any(strcmpi(varargin, 'cmd')); plotConfig.cmd = 1; end
    if any(strcmpi(varargin, 'rqmnt')); plotConfig.rqmnt = 1; end
    if any(strcmpi(varargin, 'stats')); plotConfig.stats = 1; end
end

%% Derive Data
calcData = deriveAstroBeeData;
t0 = calcData.t0;

try; out_env_msg = evalin('base', 'out_env_msg'); end;
assignin('base', 'calcData', calcData);

%% Save Monte Carlo Data if desired
if any(strcmpi(varargin, 'saveMCdata'))
    errorStats = calcData.error;
    assignin('base', 'errorStats', errorStats);
end
%% Overview Plots
if ds_on.cmd && isfield(plotConfig, 'overview')
    
    plotError(out_env_msg.P_B_ISS_ISS, out_cmd_msg.traj_pos, 'Meters', 'Position', 'true', 'traj', '--', t0)
    plotError(out_kfl_msg.P_B_ISS_ISS, out_cmd_msg.traj_pos, 'Meters', 'Commanded Position', 'est', 'traj', '--', t0)
    
    % Plot attitude error
    %plotQuatsError(out_env_msg.veh_quat_eci2body.Time,out_env_msg.veh_quat_eci2body.Data, out_cmd_msg.cmd_curr_quat.Time, out_cmd_msg.cmd_curr_quat.Data, '', 'Commanded Attitude', 'true', 'cmd','--')
    %plotQuatsError3(out_env_msg.Q_ISS2B, out_cmd_msg.traj_quat, out_cmd_msg.cmd_quat, 'Commanded Attitude', 'true', 'traj','cmd', '--', ':', t0)
    plotQuatsError(out_env_msg.Q_ISS2B, out_cmd_msg.traj_quat, 'Commanded Attitude', 'true', 'traj', '--', t0);
    
    figure; plot(calcData.error.att.total_euler.convert_rad2deg, 't0', t0, 'Title', 'Attitude Error off commanded trajectory');
    grid on; %title('Attitude Error off commanded trajectory');
    legend('\theta_{error}', '\phi_{error}', '\psi_{error}');
    xlabel('time'); ylabel('Deg')
    
end

%% Docking plots
% Still a work in progress

% Position of the midpoint between the dock lances at the same height as
% the dock lances
% 
% n = length(out_env_msg.Q_ISS2B.time);
% t = out_env_msg.Q_ISS2B.time;
% P_dock_ISS = [-.49+.02921-5.75*.0254,.57,-.8573];
% dock_normal_vec = telemData(t, repmat([1 0 0], n, 1)); % Unit vector normal to the astrobee adaptor, points towards astrobee from the dock in the world frame
% 
% % Center point between the receptacles of the dock adaptor in the astrobee
% % body frame.  In the same plane as the entrance to the dock receptacles.
% P_dockAdaptor_body_body = [-5.75*0.0254, 0, 0];
% P_dockAdaptor_body_ISS = telemData(out_env_msg.Q_ISS2B.time, rotate_b2a(out_env_msg.Q_ISS2B.data,P_dockAdaptor_body_body));
% P_dockAdaptor_ISS_ISS = P_dockAdaptor_body_ISS + out_env_msg.P_B_ISS_ISS;
% 
% dock_error_ISS = repmat(P_dock_ISS, length(P_dockAdaptor_ISS_ISS.data), 1) - P_dockAdaptor_ISS_ISS.data;
% dock_error_ISS = telemData(out_env_msg.Q_ISS2B.time, dock_error_ISS);
% 
% % Calc in track and out of track error for dock
% dock_inTrack_error = telemData(t, dot(dock_error_ISS.data, dock_normal_vec.data, 2));
% dock_outTrack_error = dock_error_ISS - telemData(t, (repmat(dock_inTrack_error.data, 1, 3).*dock_normal_vec.data));



%% Actuator Plots
if ds_on.act && isfield(plotConfig, 'overview')
    % Blower System
    figure; 
    nozzle_cmds_r = telemData(out_act_msg.act_servo_pwm_cmd.time, out_act_msg.act_servo_pwm_cmd.data(:,1:6));
    nozzle_cmds_l = telemData(out_act_msg.act_servo_pwm_cmd.time, out_act_msg.act_servo_pwm_cmd.data(:,7:12));
    subplot(2, 1, 1); plot(nozzle_cmds_r, 't0',t0); title('Nozzle PWM Commands');
    ylabel('PWM, Right Side PM');
    legend(cellstr(num2str([1:6]', 'Nozzle %-d')));
    subplot(2, 1, 2); plot(nozzle_cmds_l, 't0',t0); title('Nozzle PWM Commands');
    legend(cellstr(num2str([7:12]', 'Nozzle %-d')));
    ylabel('PWM, Left Side PM');
    %set(h(6:12), 'LineStyle', '--');
    %ylabel('PWM'); legend(cellstr(num2str([1:12]', 'Nozzle %-d')));
    
end

%% Controller Plots
if isfield(plotConfig, 'ctl')
    plotError(out_kfl_msg.P_B_ISS_ISS, out_cmd_msg.cmd_pos, 'Meters', 'Control Position', 'est', 'cmd', '--', t0)
    plotQuatsError(out_kfl_msg.quat_ISS2B, out_cmd_msg.cmd_quat, 'Control Attitude', 'est', 'cmd', '--', t0)
    
    figure; plot(out_ctl_msg.att_err_int, 't0', t0); legend('X_{int}', 'Y_{int}', 'Z_{int}'); 
    title('Attitude Integrated Error'); xlabel('seconds')
    
    % Mode
    figure; plot(out_ctl_msg.ctl_status, 't0', t0); xlabel('seconds'); title('Control Mode');
end

%% Trajectory Control Errors
% Knowledge of how far astrobee is off the commanded trajectory, different
% than what we call control error because that is the output of the shaper
% to commanded.  This rolls up hangoff error and shaper response errors.

if any(strcmpi(varargin, 'traj_ctl'))
    plotError(out_cmd_msg.traj_pos, out_kfl_msg.P_B_ISS_ISS, 'Meters', 'Trajectory Position Command', 'traj', 'est', '--', t0)
    plotError(out_cmd_msg.traj_vel, out_kfl_msg.V_B_ISS_ISS, 'M/s', 'Trajectory Velocity Command', 'traj', 'est', '--', t0)
    plotQuatsError(out_cmd_msg.traj_quat, out_kfl_msg.quat_ISS2B, 'Attitude Command', 'traj', 'est', '--', t0)
    plotError(out_cmd_msg.traj_omega.convert_rad2deg, out_kfl_msg.omega_B_ISS_B.convert_rad2deg, 'deg/sec', 'Trajectory Body Rates Command', 'traj', 'est', '--', t0)

end
%% Shaper Plots
if isfield(plotConfig, 'cmd')
    plotError(out_cmd_msg.traj_pos, out_cmd_msg.cmd_pos, 'Meters', 'Position Command', 'traj', 'cmd', '--', t0)
    plotQuatsError(out_cmd_msg.traj_quat, out_cmd_msg.cmd_quat, 'Attitude Command', 'traj', 'cmd', '--', t0)
end

%% Estimator Plots
if isfield(plotConfig, 'est')
    
    % Attitude
    figure; subplot(2,1,1);
    plot(calcData.est_euler_deg, 't0', t0);
    hold_on;
    plot(calcData.truth_euler_deg, '--', 't0', t0)
    title('Attitude Truth/Estimate Quaternion'); grid on;
    ylabel('Euler Angles, Deg'); xlabel('seconds')
    legend('\phi_{est}','\theta_{est}','\psi_{est}','\phi_{truth}', '\theta_{truth}', '\psi_{truth}')
    r = gca;
    set(gcf, 'Name', r.Title.String)
    
    subplot(2,1,2);
    plot(calcData.error.att.knowledge_mag.convert_rad2deg, 't0', t0);
    title('Attitude Error'); grid on;
    ylabel('Error Magnitude, Deg'); xlabel('seconds')
    
    %figure; plot(out_kfl_msg.ml_mahal_distance)
    
    % Body Rates
    plotError(out_kfl_msg.omega_B_ISS_B.convert_rad2deg, out_env_msg.omega_B_ISS_B.convert_rad2deg, 'Deg/sec', 'Body Rates Knowledge Error', 'est', 'truth', ':', t0)
    
    % % Add bias plot, rework with new IMU model reported bias
    % figure%(3); clf;
    % plot(out_kfl_msg.gyro_bias.Time, out_kfl_msg.gyro_bias.Data*180/pi);
    % hold_on; title('Bias Estimation')
    % plot([out_kfl_msg.gyro_bias.Time(1) out_kfl_msg.gyro_bias.Time(end)], [xim_gyro_bias_ic'; xim_gyro_bias_ic']*180/pi, ':');
    % legend('est_x', 'est_y', 'est_z', 'true_x', 'true_y', 'true_z'); ylabel('Deg/sec')
    
    % Position Error
    plotError(out_kfl_msg.P_B_ISS_ISS, out_env_msg.P_B_ISS_ISS, 'Meters', 'Position Knowledge', 'est', 'truth', ':', t0)
    
    % Velocity
    plotError(out_kfl_msg.V_B_ISS_ISS, out_env_msg.V_B_ISS_ISS, 'm/s', 'Velocity Knowledge (Global)', 'est', 'truth', ':', t0)
    
    plotError(calcData.true_lin_vel_body, calcData.est_lin_vel_body, 'm/s', 'Velocity Knowledge (Body)', 'true', 'est', '--', t0)
    
    % Acceleration
    if simData
        plotError(out_kfl_msg.A_B_ISS_ISS, out_env_msg.A_B_ISS_ISS, 'm/s^2', 'Acceleration Knowledge', 'est', 'truth', ':', t0)
    end
    %
    
    
    figure; plot(out_kfl_msg.update_ML_features_cnt, 'ro','MarkerFaceColor', 'r', 't0', t0); hold on;
    plot(out_kfl_msg.update_OF_tracks_cnt, 'bo','MarkerFaceColor', 'b', 't0', t0);
    title('Number of features used for vision update'); ylabel('# of observations')
    r = gca; legend('ML Features', 'OF Features')
    set(gcf, 'Name', r.Title.String)
    
    % Kfl Status
    figure; plot(out_kfl_msg.kfl_status, 'o', 't0', t0)
    set(gca, 'YTick', [0 1 2]);
    set(gca, 'YTickLabel', {'IMU', 'ML', 'OF'});
    title('Est Update Type')
    r = gca;
    set(gcf, 'Name', r.Title.String)
    
    % Acceleration Bias
    figure%(7); clf;
    plot(out_kfl_msg.accel_bias.time-t0, out_kfl_msg.accel_bias.data);
    title('Accel Bias Estimation'); ylabel('M/s^2'); grid on;
    r = gca;
    set(gcf, 'Name', r.Title.String)
    if simData
        hold_on;
        plot(out_imu_msg.imu_accel_bias, ':');
        legend('est_x', 'est_y', 'est_z', 'true_x', 'true_y', 'true_z');
    else
        legend('est_x', 'est_y', 'est_z');
    end
    
     % Gyro Bias
    figure%(7); clf;
    plot(out_kfl_msg.gyro_bias.time-t0, out_kfl_msg.gyro_bias.data*180/pi);
    title('Gyro Bias Estimation'); ylabel('Deg/s'); grid on;
    r = gca;
    set(gcf, 'Name', r.Title.String)
    if simData
        hold_on;
        q_body2imu = quaternion(tun_abp_quat_body2imu);
        true_imu_bias_body = telemData(out_imu_msg.imu_gyro_bias.time,  rotate_b2a(q_body2imu, out_imu_msg.imu_gyro_bias));
        plot(true_imu_bias_body.convert_rad2deg, ':');
        legend('est_x', 'est_y', 'est_z', 'true_x', 'true_y', 'true_z');
    else
        legend('est_x', 'est_y', 'est_z');
    end
    
    
end

if isfield(plotConfig, 'mahal')
    figure; plot(out_kfl_msg.ml_mahal_distance,'v', 't0', t0)
    title('Mahalanobis Distance')
    r = gca;
    set(gcf, 'Name', r.Title.String)
end

%% Process All Figures
all_axes_h = findobj('type', 'axes');
if ~isempty(all_axes_h)
    linkaxes(all_axes_h, 'x'); % Link all the X axes
end

% Could apply the figure titles here

%% Stats
% Dependent on current_file_name having been generated by
% astrobee_load_p3_data.m
if isfield(plotConfig, 'stats')
    if simData
        titleParsed = {datestr(now, 30)};
    else
        titleParsed = strsplit(current_file_name, '_');
    end
    
    % Stats
    fid_array = 1;
    if writeStats
        fid_array = [fid_array fopen([titleParsed{1} '_stats.txt'], 'w')];
    end
    
    for ii = 1:length(fid_array)
        if ds_on.cmd && ds_on.env
            fprintf(fid_array(ii),'\n');
            fprintf(fid_array(ii),'True Position Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (m): %f, %f, %f\n', maxerror(calcData.error.pos.total));
            fprintf(fid_array(ii),'Mean (m): %f, %f, %f\n', mean(calcData.error.pos.total));
            fprintf(fid_array(ii),'Mean + 3 Sigma (m): %f, %f, %f\n', mean3sig(calcData.error.pos.total));
            fprintf(fid_array(ii),'Standard Deviation (m): %f, %f, %f\n\n', std(calcData.error.pos.total));
            
            % Calc Euler as well
            fprintf(fid_array(ii),'True Attitude Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (deg): %f\n', maxerror(calcData.error.att.total_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Mean (deg): %f\n', mean(calcData.error.att.total_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Mean + 3 Sigma (deg): %f\n', mean3sig(calcData.error.att.total_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Standard Deviation (deg): %f\n\n', std(calcData.error.att.total_mag.convert_rad2deg));
        end
        
        if ds_on.kfl && ds_on.env
            fprintf(fid_array(ii),'Position Knowledge Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (m): %f, %f, %f\n', maxerror(calcData.error.pos.knowledge));
            fprintf(fid_array(ii),'Mean (m): %f, %f, %f\n', mean(calcData.error.pos.knowledge));
            fprintf(fid_array(ii),'Mean + 3 Sigma (m): %f, %f, %f\n', mean3sig(calcData.error.pos.knowledge));
            fprintf(fid_array(ii),'Standard Deviation (m): %f, %f, %f\n\n', std(calcData.error.pos.knowledge));
            
            fprintf(fid_array(ii),'Velocity Knowledge Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (m/s): %f, %f, %f\n', maxerror(calcData.error.velocity.knowledge));
            fprintf(fid_array(ii),'Mean (m/s): %f, %f, %f\n', mean(calcData.error.velocity.knowledge));
            fprintf(fid_array(ii),'Mean + 3 Sigma (m/s): %f, %f, %f\n', mean3sig(calcData.error.velocity.knowledge));
            fprintf(fid_array(ii),'Standard Deviation (m/s): %f, %f, %f\n\n', std(calcData.error.velocity.knowledge));
            
            fprintf(fid_array(ii),'Attitude Knowledge Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (deg): %f\n', maxerror(calcData.error.att.knowledge_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Mean (deg): %f\n', mean(calcData.error.att.knowledge_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Mean + 3 Sigma (deg): %f\n', mean3sig(calcData.error.att.knowledge_mag.convert_rad2deg));
            fprintf(fid_array(ii),'Standard Deviation (deg): %f\n\n', std(calcData.error.att.knowledge_mag.convert_rad2deg));
        end
        
        if ds_on.cmd && ds_on.env
            [dataMax, dataMin, dataMean] = calcTPMvalues(calcData.error.pos.total.data,95);
            fprintf(fid_array(ii),'95 True Position Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (m): %f, %f, %f\n', dataMax);
            fprintf(fid_array(ii),'Min  Error (m): %f, %f, %f\n', dataMin);
            fprintf(fid_array(ii),'Mean Error (m): %f, %f, %f\n\n', dataMean);
            
            % Calc Euler as well
            [dataMax, dataMin, dataMean] = calcTPMvalues(calcData.error.att.total_mag.convert_rad2deg.data,95);
            fprintf(fid_array(ii),'95 True Attitude Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (deg): %f\n', dataMax);
            fprintf(fid_array(ii),'Min  Error (deg): %f\n', dataMin);
            fprintf(fid_array(ii),'Mean Error (deg): %f\n\n', dataMean);
        end
        
        if ds_on.kfl && ds_on.env
            [dataMax, dataMin, dataMean] = calcTPMvalues(calcData.error.pos.knowledge.data,95);
            fprintf(fid_array(ii),'95 Position Knowledge Error Stats:\n');
            fprintf(fid_array(ii),'Max  Error (m): %f, %f, %f\n', dataMax);
            fprintf(fid_array(ii),'Min  Error (m): %f, %f, %f\n', dataMin);
            fprintf(fid_array(ii),'Mean Error (m): %f, %f, %f\n\n', dataMean);
            
            fprintf(fid_array(ii),'95 Velocity Knowledge Error Stats:\n');
            [dataMax, dataMin, dataMean] = calcTPMvalues(calcData.error.velocity.knowledge.data,95);
            fprintf(fid_array(ii),'Max  Error (m/s): %f, %f, %f\n', dataMax);
            fprintf(fid_array(ii),'Min  Error (m/s): %f, %f, %f\n', dataMin);
            fprintf(fid_array(ii),'Mean Error (m/s): %f, %f, %f\n\n', dataMean);
            
            fprintf(fid_array(ii),'95 Attitude Knowledge Error Stats:\n');
            [dataMax, dataMin, dataMean] = calcTPMvalues(calcData.error.att.knowledge_mag.convert_rad2deg.data,95);
            fprintf(fid_array(ii),'Max  Error (deg): %f\n', dataMax);
            fprintf(fid_array(ii),'Min  Error (deg): %f\n', dataMin);
            fprintf(fid_array(ii),'Mean Error (deg): %f\n\n', dataMean);
        end
    end
    
    if writeStats
        fclose(fid_array(2));
    end
end

%% Save the figs if necesarry
% Dependent on current_file_name having been generated by
% astrobee_load_p3_data.m
if saveFigs
    fig_Handles = findobj('type', 'fig');
    savefig(fig_Handles, [titleParsed{1} '_figs'], 'compact')
end





