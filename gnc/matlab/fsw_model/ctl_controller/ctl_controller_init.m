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

% Astrobee controller (CTL) initialization file.  Configures
% parameters used in the library file ctl_controller.
%

%% Block Configuration
ctl_ts                      = astrobee_fsw_step_size;

%% Modes
ctl_idle_mode               = uint8(0);
ctl_stopping_mode           = uint8(1);
ctl_nominal_mode            = uint8(2);
ctl_stopped_mode            = uint8(3);

%% Velocity Loop
ctl_linear_vel_limit = single(10000); %m/s,  0.5 m/s reads(3.GNC.22), need to rework

%% Rate Loop (angular rates)
ctl_omega_limit             = single(10000*pi/180*ones(3,1)); %rad/sec Still TBD (3.GNC.23)

%% Thresholds
ctl_stopping_vel_thresh     = single(.01^2); % Linear velocity threshold squared below which the vehicle is considered to be stopped, (m/s)^2
ctl_stopping_omega_thresh   = single(.01^2); % Angular velocity threshold squared below which the vehicle is considered to be stopped, (rad/s)^2

%% Moved to lua conig file [ASTROBEE_ROOT '/../../management/astrobee/config/gnc.config']
% tun_ctl_bypass_cmd_shaper = uint8(0);
% ctl_att_ki                = single([0.0013, 0.0013, 0.0013]);
% ctl_att_kp                = single([0.2667, 0.2667, 0.2667]);
% ctl_pos_ki                = single([0.0167, 0.0167, 0.0]);
% ctl_pos_kp                = single([0.5333, 0.5333, 0.0]);
% ctl_pch_pos_k             = 8; % 2
% ctl_pch_att_k             = 20; % 8
% ctl_max_flap_theta        = single(45*pi/180); % unrestricted = 64.23 deg
% ctl_pch_max_area          = .75*[0.0027, 0.0027, 0.0015 ,0.0015 ,0.0015, 0.0015 ... % Max area before PCH starts clamping commands
%                                  0.0027, 0.0027, 0.0015, 0.0015, 0.0015, 0.0015];
