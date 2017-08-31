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
%
% Proto 4 INIT
% Init file containing the delta's between proto 4 and flight
% Dependencies:
%       astrobee_init has been run first and default parameters are loaded in the current workspace
%
%
%% Debugging
% cvs_noise_on = false;

astrobee_version = 'p4';
tunable_init;

%% Enviroment
% env_granite_lab_flag            = boolean(1);                       %[flag] Indicates if configuration should be for granite lab testing
env_grav_disturb_on             = boolean(0);                       %[flag] Turns on/off the gravity disturbance model (to be used in granite table setup only!)
env_rot_effects_on              = boolean(0);                       %[flag] Turns on/off the rotation effects in the dynamic model
% Moved to gnc.config
%tun_env_drag_disturb_on         = boolean(0);                       %[flag] Turns on/off the drag disturbance model


%% Physical Properties

%% Control Gains
%% Position Loop
% Note, control architecture described in TBD, .75, 6 .001
% Moved to gnc.config
% ctl_pos_kp = single(4*ones(1,3)); % Note, this is Kp in the traditional sense, actual Kp calculated in prep, 18
% ctl_pos_ki = single( .1/100*ones(1,3) ); % Note, this is Ki in the traditional sense, actual Ki calculated in prep
ctl_pos_sat_upper = single( 0.5 );
ctl_pos_sat_lower = single( -0.5 );

%.8,4,.01
%% Velocity Loop
ctl_linear_vel_limit = single( 10000 ); %m/s,  0.5 m/s reads(3.GNC.22), need to rework
%ctl_vel_kd = single( 25*ones(1,3) ); %2.5

%% Linear Output
ctl_linear_force_limit = 1*100;

%% Attitude Loop
% kp = .4, kd=1
% Moved to gnc.config
% ctl_att_kp = single( 8*.1*ones(1,3)); % Note, this is Kp in the traditional sense, actual Kp calculated in prep
% ctl_att_ki = single( 4*.0001*ones(1,3) ); % Note, this is Ki in the traditional sense, actual Ki calculated in prep

ctl_att_sat_upper = single( 0.5 );
ctl_att_sat_lower = single( -0.5 );

%% Rate Loop (angular rates)
ctl_omega_limit = single( 10000*pi/180*ones(3,1) ); %rad/sec Still TBD (3.GNC.23)
ctl_omega_kd = single( 3*1*ones(3,1) );

%% Earth Parameters (for ground testing only)
ase_earth_rate = single(0*[0, 0, 1]*((360/23.9344699)/3600)*pi/180); % Earth rate, rad/sec, in ECEF
ase_pos_vec = single(0*1.0e+06*[-2.6922, -4.2987, 3.8541]); % N269 Lab position in ECEF (lla2ecef([37.415117, -122.058621, 0]))
ase_gravity_accel = single(0*[0,0,-9.81]);

ase_accel_fixed_bias = single(0*[-0.0292, -0.0325,0.0349]);
epson_accel_bias_ic = single([0 0 0]);
