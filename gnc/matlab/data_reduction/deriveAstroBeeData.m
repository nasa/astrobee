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

function calcData = deriveAstroBeeData
% Derive data from astrobee telemetry logs
%% Import Data from the base workspace
% Find out which data sets are "on", exist in the workspace
ds_on = evalin('caller', 'ds_on');
simData = evalin('base', 'simData');


firstTime = 0;
if ds_on.cmd
    out_cmd_msg = evalin('base', 'out_cmd_msg');
    % If the cmd message is present, use the first command as t0
    firstTime = out_cmd_msg.traj_quat.time(find(rssrow(out_cmd_msg.traj_quat.data.value) ~= 0, 1, 'first'));
end

if ds_on.env
    out_env_msg = evalin('base', 'out_env_msg');
    firstTime = max(firstTime, out_env_msg.P_B_ISS_ISS.time(1));
end

if ds_on.kfl
    out_kfl_msg = evalin('base', 'out_kfl_msg');
    firstTime = max(firstTime, out_kfl_msg.omega_B_ISS_B.time(1));
end

calcData.t0 = inf;
%% Do all ENV only calcs now
if ds_on.env
    if ~simData % If it is vehicle data derive the velocities
        out_env_msg.V_B_ISS_ISS = telemData(out_env_msg.P_B_ISS_ISS.time(2:end), diff(out_env_msg.P_B_ISS_ISS.data)./ repmat(diff(out_env_msg.P_B_ISS_ISS.time), 1,3));
        out_env_msg.omega_B_ISS_B = telemData(out_env_msg.Q_ISS2B.time, rate_from_quat(out_env_msg.Q_ISS2B.time, out_env_msg.Q_ISS2B.data.value));
    end
    % Export this data back to the workspace
    assignin('base', 'out_env_msg', out_env_msg);
    
    out_env_msg = structfun(@(a) timeRange(a, firstTime, inf), out_env_msg, 'UniformOutput', false);
    calcData.t0 = min(calcData.t0, out_env_msg.P_B_ISS_ISS.time(1));
    
    calcData.truth_euler_deg = telemData(out_env_msg.Q_ISS2B.time, convert_to_eulers(out_env_msg.Q_ISS2B.data) *180/pi);
    
    calcData.true_lin_vel_body = telemData(out_env_msg.Q_ISS2B.time, rotate_a2b(out_env_msg.Q_ISS2B.data, out_env_msg.V_B_ISS_ISS.data));
    
end


%% Docking Calcs
%% Docking plots
% Still a work in progress
if ds_on.env
    % Position of the midpoint between the dock lances at the same height as
    % the dock lances
    %
    n = length(out_env_msg.Q_ISS2B.time);
    t = out_env_msg.Q_ISS2B.time;
    P_dock_ISS = [-.49+.02921-5.75*.0254,.57,-.8573];
    dock_normal_vec = telemData(t, repmat([1 0 0], n, 1)); % Unit vector normal to the astrobee adaptor, points towards astrobee from the dock in the world frame
    
    % Center point between the receptacles of the dock adaptor in the astrobee
    % body frame.  In the same plane as the entrance to the dock receptacles.
    P_dockAdaptor_body_body = [-5.75*0.0254, 0, 0];
    P_dockAdaptor_body_ISS = telemData(out_env_msg.Q_ISS2B.time, rotate_b2a(out_env_msg.Q_ISS2B.data,P_dockAdaptor_body_body));
    P_dockAdaptor_ISS_ISS = P_dockAdaptor_body_ISS + out_env_msg.P_B_ISS_ISS;
    
    dock_error_ISS = repmat(P_dock_ISS, length(P_dockAdaptor_ISS_ISS.data), 1) - P_dockAdaptor_ISS_ISS.data;
    dock_error_ISS = telemData(out_env_msg.Q_ISS2B.time, dock_error_ISS);
    
    % Calc in track and out of track error for dock
    dock_inTrack_error = telemData(t, dot(dock_error_ISS.data, dock_normal_vec.data, 2));
    dock_outTrack_error = dock_error_ISS - telemData(t, (repmat(dock_inTrack_error.data, 1, 3).*dock_normal_vec.data));
    
    calcData.error.docking.inTrack = dock_inTrack_error;
    calcData.error.docking.inTrack_final = dock_inTrack_error.data(end);
    calcData.error.docking.outOfTrack = dock_outTrack_error.mag;
    calcData.error.docking.outOfTrack_final = calcData.error.docking.outOfTrack.data(end);
    calcData.error.docking.outOfTrack_vector = dock_outTrack_error;
    
    % Define cups in the body frame and lances in the ISS frame
    P_cup1_B_B = [-5.75, 0, 1.25]*0.0254;
    P_cup2_B_B = [-5.75, 0, -1.25]*0.0254;
    P_lance1_ISS_ISS = [-.49+.02921-5.75*.0254, .57,-.8573+1.25*0.0254];
    P_lance2_ISS_ISS = [-.49+.02921-5.75*.0254, .57,-.8573-1.25*0.0254];
    
    % Rotate the cups into the ISS frame
    P_cup1_body_ISS = telemData(out_env_msg.Q_ISS2B.time, rotate_b2a(out_env_msg.Q_ISS2B.data,P_cup1_B_B));
    P_cup2_body_ISS = telemData(out_env_msg.Q_ISS2B.time, rotate_b2a(out_env_msg.Q_ISS2B.data,P_cup2_B_B));
    
    % Shift the cup origins to be relative to ISS C.F.
    P_cup1_ISS_ISS = P_cup1_body_ISS + out_env_msg.P_B_ISS_ISS;
    P_cup2_ISS_ISS = P_cup2_body_ISS + out_env_msg.P_B_ISS_ISS;
    
    % Calc errors
    cup1_error_ISS = repmat(P_lance1_ISS_ISS, length(P_cup1_ISS_ISS.data), 1) - P_cup1_ISS_ISS.data;
    cup1_error_ISS = telemData(out_env_msg.Q_ISS2B.time, cup1_error_ISS);
    cup2_error_ISS = repmat(P_lance2_ISS_ISS, length(P_cup2_ISS_ISS.data), 1) - P_cup2_ISS_ISS.data;
    cup2_error_ISS = telemData(out_env_msg.Q_ISS2B.time, cup2_error_ISS);
    
    % Calc in track and out of track error for dock
    cup1_inTrack_error = telemData(t, dot(cup1_error_ISS.data, dock_normal_vec.data, 2));
    cup1_outTrack_error = cup1_error_ISS - telemData(t, (repmat(cup1_inTrack_error.data, 1, 3).*dock_normal_vec.data));
    cup2_inTrack_error = telemData(t, dot(cup2_error_ISS.data, dock_normal_vec.data, 2));
    cup2_outTrack_error = cup2_error_ISS - telemData(t, (repmat(cup2_inTrack_error.data, 1, 3).*dock_normal_vec.data));
    
    
    calcData.error.docking.cup1_inTrack = cup1_inTrack_error;
    calcData.error.docking.cup1_inTrack_final = cup1_inTrack_error.data(end);
    calcData.error.docking.cup1_outOfTrack = cup1_outTrack_error.mag;
    calcData.error.docking.cup1_outOfTrack_final = calcData.error.docking.cup1_outOfTrack.data(end);
    calcData.error.docking.cup1_outOfTrack_vector = cup1_outTrack_error;
    
    calcData.error.docking.cup2_inTrack = cup2_inTrack_error;
    calcData.error.docking.cup2_inTrack_final = cup2_inTrack_error.data(end);
    calcData.error.docking.cup2_outOfTrack = cup2_outTrack_error.mag;
    calcData.error.docking.cup2_outOfTrack_final = calcData.error.docking.cup2_outOfTrack.data(end);
    calcData.error.docking.cup2_outOfTrack_vector = cup2_outTrack_error;
    
end

%% DO all KFL only calcs now
if ds_on.kfl
    out_kfl_msg = structfun(@(a) timeRange(a, firstTime, inf), out_kfl_msg, 'UniformOutput', false);
    calcData.t0 = min(calcData.t0, out_kfl_msg.omega_B_ISS_B.time(1));
    
    calcData.est_euler_deg = telemData(out_kfl_msg.quat_ISS2B.time, convert_to_eulers(out_kfl_msg.quat_ISS2B.data) *180/pi);
    calcData.est_lin_vel_body = telemData(out_kfl_msg.quat_ISS2B.time, rotate_a2b(out_kfl_msg.quat_ISS2B.data, out_kfl_msg.V_B_ISS_ISS.data));

end

%% Do all the CMD only calcs now
if ds_on.cmd
    out_cmd_msg = structfun(@(a) timeRange(a, firstTime, inf), out_cmd_msg, 'UniformOutput', false);
    calcData.t0 = min(calcData.t0, out_cmd_msg.traj_quat.time(1));
end
%calcData.t0 = min(min(out_env_msg.P_B_ISS_ISS.time), min(out_kfl_msg.omega_B_ISS_B.time));

%% Knowledge Errors
if ds_on.kfl && ds_on.env
    calcData.error.att.knowledge_mag = out_env_msg.Q_ISS2B - out_kfl_msg.quat_ISS2B;
    calcData.error.att.knowledge_quat = out_env_msg.Q_ISS2B * (-out_kfl_msg.quat_ISS2B); % Takes a long time to calc
    calcData.error.att.knowledge_euler = telemData(calcData.error.att.knowledge_quat.time, convert_to_eulers(calcData.error.att.knowledge_quat.data));
    calcData.error.pos.knowledge = out_env_msg.P_B_ISS_ISS - out_kfl_msg.P_B_ISS_ISS;
    calcData.error.velocity.knowledge = out_env_msg.V_B_ISS_ISS - out_kfl_msg.V_B_ISS_ISS;
end

%% Total Errors
if ds_on.env && ds_on.cmd
    % Total Errors
    calcData.error.att.total_mag = out_env_msg.Q_ISS2B - out_cmd_msg.traj_quat;
    calcData.error.att.total_quat = out_env_msg.Q_ISS2B * (-out_cmd_msg.traj_quat); % Takes a long time to calc
    calcData.error.att.total_euler = telemData(calcData.error.att.total_quat.time, convert_to_eulers(calcData.error.att.total_quat.data));
    
    calcData.error.pos.total = out_env_msg.P_B_ISS_ISS - out_cmd_msg.traj_pos;
end

%% Trajectory Control Error
if ds_on.kfl && ds_on.cmd
    % Total Errors
    calcData.error.att.traj_ctl_mag = out_cmd_msg.traj_quat - out_kfl_msg.quat_ISS2B;
    calcData.error.att.traj_ctl_quat = out_cmd_msg.traj_quat * (-out_kfl_msg.quat_ISS2B); % Takes a long time to calc
    calcData.error.att.traj_ctl_euler = telemData(calcData.error.att.traj_ctl_quat.time, convert_to_eulers(calcData.error.att.traj_ctl_quat.data));
    calcData.error.omega.traj_ctl = out_cmd_msg.traj_omega - out_kfl_msg.omega_B_ISS_B;
    
    calcData.error.pos.traj_ctl = out_cmd_msg.traj_pos - out_kfl_msg.P_B_ISS_ISS;
    calcData.error.velocity.traj_ctl = out_cmd_msg.traj_vel - out_kfl_msg.V_B_ISS_ISS;
    
end


%% Control Errors
if ds_on.kfl && ds_on.cmd
    calcData.error.att.control_mag = out_kfl_msg.quat_ISS2B - out_cmd_msg.traj_quat;
    calcData.error.att.control_quat = out_kfl_msg.quat_ISS2B * (-out_cmd_msg.traj_quat); % Takes a long time to calc
    calcData.error.att.control_euler = telemData(calcData.error.att.control_quat.time, convert_to_eulers(calcData.error.att.control_quat.data));
    
    calcData.error.pos.control = out_kfl_msg.P_B_ISS_ISS - out_cmd_msg.traj_pos;
end

