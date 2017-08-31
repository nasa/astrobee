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

%env_environment_model_init.m
% enviroment model initialization file

%% Control
env_grav_disturb_on             = boolean(0);                           %[flag] Turns on/off the gravity disturbance model (to be used in granite table setup only!)
env_rot_effects_on              = boolean(1);                           %[flag] Turns on/off the rotation effects in the dynamic model

% Moved to gnc.config
%tun_env_drag_disturb_on             = boolean(1);                           %[flag] Turns on/off the drag disturbance model
%note: to keep drag on, but remove the external airflow, set max value below to 0

%% Error
env_ini_position_error          = single(0.05);                         %[m] Error in the initial position
env_ini_orientation_error_angle = single(3*pi/180);                     %[rad] Error in the intial orienetaion

%% Linear Drag Parameters
%Constants
const_air_density               = single(1.2);                          %[kg/m^3] density of air in ISS with temperature ~20C

%Drag coefficients
env_cube_drag_coeff             = single(1.05);
env_ang_cube_drag_coeff         = single(0.80);
env_cube_ref_area               = single(0.092903);                     %[m^2]
env_ang_cube_ref_area           = env_cube_ref_area*sqrt(2);            %[m^2]

%Drag force = (1/2)*(drag coeff)*(air density)*(ref area)*(velocity ^2)
%the variables below represent (1/2)*(drag coeff)*(air density)*(ref area), to get drag, simply multiple by velocity^2
env_cube_force_drag_coeff       = (1/2)*env_cube_drag_coeff*env_cube_ref_area*const_air_density;
env_ang_cube_drag_coeff         = (1/2)*env_ang_cube_drag_coeff*env_ang_cube_ref_area*const_air_density;
%since the difference between a cube face, and angular cube is so small
%....simply take the average and use that as the coeff.
env_avg_drag_coeff              = mean([env_cube_force_drag_coeff, env_ang_cube_drag_coeff]);  

% External Air Velocity Errors 
env_ext_air_vel_variance        = [0.01 0.01 0.01];                     %[m/s^2] (note: variance must be type double for random number block)
env_ext_air_vel_seed            = [1 2 3];
env_max_ext_air_vel             = single(0.2);                          %[m/s^2]  Maximum air velocity in ISS: 0.076-0.20 covers ~70% of cases (extreme cases are near vents and along walls)

%% rotational drag and air error
env_rotational_drag_coeff       = 0.001;                                %[-] (similar to drag coeff) Fractional amount of rotational velocity used in external disturbance torque
env_ext_air_omega_variance      = [0.01 0.01 0.01];                     %[rad/s] rotational noise
env_max_ext_air_omega           = single(0.1);                          %[rad/s] maximum disturabance rotation

%% Gravity
const_gravity_local             = single([0, 0,-9.80665]);              %[m/s^2] Gravity defined in the local room frame (Z axis points up)
