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

%env_environment_model_test.m

%% Setup
sim_end_time                    = 3600*2;

ini_time_nanoseconds = 0;
ini_time_seconds = 0;

init_omega_B_ISS_B              = single([0, 0, 0]);                % Initial body rates relative to ISS
init_Q_ISS2B                    = single([0, 0, 0, 1]);             % Initial body quaternion, ISS to Body CF

init_P_B_ISS_ISS                = single([10, 0, 0]);                %[m] Initial body velocity
init_V_B_ISS_ISS                = single([0, 0, -0.0113]);                %[m/s] Initial body posistion


env_granite_lab_flag            = boolean(0);                       %[flag] Indicates if configuration should be for granite lab testing
env_grav_disturb_on             = boolean(0);                       %[flag] Turns on/off the gravity disturbance model (to be used in granite table setup only!)
env_drag_disturb_on             = boolean(0);                       %[flag] Turns on/off the drag disturbance model
env_rot_effects_on              = boolean(1);                       %[flag] Turns on/off the rotation effects in the dynamic model
env_max_ext_air_vel             = single(0.2);                      %[m/s^2]  Maximum air velocity in ISS: 0.076-0.20 covers ~70% of cases (extreme cases are near vents and along walls)
env_max_ext_air_omega           = single(0.1);                      %[rad/s] maximum disturbance rotation

env_environment_model_prep;

in_prop_torque.signals.dimensions   = 3;
in_prop_torque.signals.values       = [0, 0, 0;
%                                       0, 0, 0.0094;                %[N*m] = (desired alpha)*ffp_veh_inertia_matrix
                                       0, 0, 0;
                                       0, 0, 0];
in_prop_torque.time                 = [0; 5; 8];

in_prop_force.signals.dimensions    = 3;
in_prop_force.signals.values        = [0, 0, 0;
%                                      1.2, 0, 0;                     %[N] = (desired accel)* ffp_veh_mass
                                       0, 0, 0;
                                       0, 0, 0];
in_prop_force.time                  = [0; 30; 40];

%% run
sim('env_environment_model_hrn');

%% plot
close all;

% figure; 
% subplot(211); plot(out_time_msg.timestamp_sec);
% title('nanoseconds');
% subplot(212); plot(out_time_msg.timestamp_nsec);
% title('seconds');

figure;
subplot(311);plot(out_env_msg.P_B_ISS_ISS);
title('Position'); legend('x','y','z');
subplot(312);plot(out_env_msg.V_B_ISS_ISS);
title('Velocity'); legend('x','y','z');
subplot(313);plot(out_env_msg.A_B_ISS_ISS);
title('Accel'); legend('x','y','z');

figure;
subplot(311);plot(out_env_msg.Q_ISS2B);
title('Quat'); legend('x','y','z');
subplot(312);plot(out_env_msg.omega_B_ISS_B);
title('Omega'); legend('x','y','z');
subplot(313);plot(out_env_msg.alpha_B_ISS_B);
title('Alpha'); legend('x','y','z');