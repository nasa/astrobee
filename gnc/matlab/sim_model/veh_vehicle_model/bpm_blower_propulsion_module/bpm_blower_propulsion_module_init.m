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

%bpm_blower_propulsion_module_init.m

%% General
bpm_use_bpm                         = single(0);                            %[flag] 1= use BPM, 0 = use VPP

%tun_bpm_noise_on_flag               = single(1);                            %[flag] 1=noise on, 0 = noise off

%% impeller motor parameters
%Maxon EC45flat-30W brushless motor with Hall sensors
bpm_imp_ctl_ts                      = 0.016;                                 %[sec] Time step size for the impeller speed PID controller
bpm_imp_ctl_kp                      = single(0.2);                        %[-] P gain for impeller speed PID controller
bpm_imp_ctl_ki                      = single(0.1);                         %[-] I gain for impeller speed PID controller
bpm_imp_ctl_kd                      = single(0.05);                        %[-] D gain for impeller speed PID controller
bpm_rate_limit                      = single(200*2*pi/60);                  % [rad/sec] Rate limit change of impeller speed command
bpm_imp_ctl_filt_n                  = single(1.0);                          %[-] Filter Coefficient for impeller speed PID controller 
bpm_imp_speed_filt_num              = single(1.0);                          %[-] Transfer function numerator for current impeller speed feedback
bpm_imp_speed_filt_den              = single(1.0);                          %[-] Transfer function denominator for current impeller speed feedback
bpm_imp_cmd_filt_num                = single(1.0);                          %[-] Transfer function numerator for commanded impeller speed
bpm_imp_cmd_filt_den                = single(1.0);                          %[-] Transfer function denominator for commanded impeller speed

bpm_imp_max_voltage                 = single(16.6);                         %[Volt] Maximum voltage for the impeller motor
bpm_imp_nom_voltage                 = single(12);                           %[Volt] Nominal voltage for the impeller motor
bpm_imp_nom_torque                  = single(0.0555);                       %[Nm] Nominal torque at voltage
bpm_imp_nom_curr                    = single(2.03);                         %[A] Nominal current at voltage
bpm_imp_nom_speed                   = single(2940*2*pi/60);                 %[rad/s] Nominal speed at voltage
bpm_imp_noload_curr                 = single(0.144);                        %[A] No load current
bpm_imp_noload_speed                = single(4380*2*pi/60);                 %[rad/s] No load speed
%imp motor friction coeff in prep.m

bpm_imp_motor_torque_k              = single(0.0255);                       %[Nm/A] Impeller Torque constant
bpm_imp_motor_speed_k               = single(374*2*pi/60);                  %[(rad/s)/V] Impeller Speed constant
bpm_imp_motor_r                     = single(1.2);                          %[ohm] Impeller Motor internal resistance
bpm_imp_motor_l                     = single(0.0056);                       %[H] Impeller Motor internal inductance
bpm_imp_motor_inertia               = single(0.00000925);                   %[kg*m^2] Impeller motor intertia
bpm_imp_rotor_inertia               = single(0.005);                        %[kg*m^2] Impeller rotor inertia

%% servo parameters
%MKS DS92A+ servo
bpm_servo_ctl_ts                    = 0.016;                                 %[sec] Time step size for the servo PID controller
bpm_servo_ctl_kp                    = single(5.0);                          %[-] P gain for servo PID controller
bpm_servo_ctl_ki                    = single(1.5);                          %[-] I gain for servo PID controller
bpm_servo_ctl_kd                    = single(0.8);                          %[-] D gain for servo PID controller
bpm_servo_ctl_filt_n                = single(1.0);                          %[-] Filter Coefficient for servo PID controller

bpm_servo_max_voltage               = single(6);                            %[Volt] Saturation voltage for the servo controller output (voltage servo is running at)
bpm_servo_peak_torque               = single(0.2893);                       %[Nm] from Datasheet @6V
bpm_servo_peak_speed                = single(0.058/60);                     %[sec/deg] from Datahsheet @6V
bpm_servo_peak_curr                 = single(1.5);                          %[amp] (WAG) Peak current draw of servo
bpm_servo_noload_curr               = single(0.01);                         %[amp] (WAG) Servo no load current draw
bpm_servo_motor_friction_coeff      = single(0.0000003);                    %[Nm*s] (WAG)  Due to Viscous Friction

bpm_servo_motor_gear_ratio          = single(1/100);                        %[-] (WAG) Servo gear ratio
bpm_servo_motor_backlash_deadband   = single(1*pi/180);                     %[rad] (WAG) Servo gearbox backlash deadband
bpm_servo_motor_gear_box_inertia    = single(.000000025);                   %[kg*m^2] (Tuned in analysis_servo_motor_test.m) Servo gearbox inertia.  
%servo motor R and K in prep.m

%% Impeller parameters
bpm_impeller_inertia                = single(0.001);                        %[kg*m^2] Moment of inertia of the blower system
bpm_impeller_inertia_error          = single(0);                            %[kg*m^2] Error in the MOI of the impellers
bpm_impeller_init_speed             = single(0);                            %[rad/s] Initial condition for blower speed
bpm_impeller_eq_density             = single(0.001);

bmp_PM1_impeller_orientation_error  = single([0 0 0]);                      %[vector] PM1 Orientation Error of the impeller rotational pointing unit vector
bmp_PM2_impeller_orientation_error  = single([0 0 0]);                      %[vector] PM2 Orientation Error of the impeller rotational pointing unit vector
bpm_PM1_zero_thrust_area_error      = single([0]);                          %[m^2] PM1 Error associated with the leakage area (zero thrust)
bpm_PM2_zero_thrust_area_error      = single([0]);                          %[m^2] PM2 Error associated with the leakage area (zero thrust)
 
%TODO: add the drag parameters (need to add drag model in still)

%% Nozzle Parameters
bpm_PM1_P_nozzle_B_B_error          = zeros(6,3,'single');                  %[m] PM1 Position offset error of the nozzles
bpm_PM2_P_nozzle_B_B_error          = zeros(6,3,'single');                  %[m] PM2 Position offset error of the nozzles
bpm_PM1_nozzle_orientation_az_error = zeros(6,1,'single');                  %[rad] PM1 Nozzle Orientation Azimuth Error
bpm_PM2_nozzle_orientation_az_error = zeros(6,1,'single');                  %[rad] PM2 Nozzle Orientation Azimuth Error
bpm_PM1_nozzle_orientation_el_error = zeros(6,1,'single');                  %[rad] PM1 Nozzle Orientation Elevation Error
bpm_PM2_nozzle_orientation_el_error = zeros(6,1,'single');                  %[rad] PM2 Nozzle Orientation Elevation Error
bpm_PM1_nozzle_discharge_coeff_error= zeros(6,1,'single');                  %[-] PM1 Error associated with the discharge coefficient
bpm_PM2_nozzle_discharge_coeff_error= zeros(6,1,'single');                  %[-] PM2 Error associated with the discharge coefficient
bpm_PM1_nozzle_thrust_noise_1s      = zeros(6,1);                           %[N] PM1 1-sigma value on noise for thrust
bpm_PM2_nozzle_thrust_noise_1s      = zeros(6,1);                           %[N] PM2 1-sigma value on noise for thrust
bpm_PM1_nozzle_noise_feedback_gain  = zeros(6,1);                           %[-] PM1 gain on the feedback on nozzle thrust noise
bpm_PM2_nozzle_noise_feedback_gain  = zeros(6,1);                           %[-] PM2 gain on the feedback on nozzle thrust noise
bpm_PM1_randn_noise_seed            = 254;                                  %[-] PM1 Noise seed for the randn block
bpm_PM2_randn_noise_seed            = 255;                                  %[-] PM2 Noise seed for the randn block

%% rotor speed sensor
bpm_sensor_sf                       = single(1);                            %[-] Scale factor for the speed sensor
bpm_sensor_sigma_u                  = .00001;                               %[-] Sigma U noise for the speed sensor
bpm_sensor_sigma_v                  = .00001;                               %[-] Sigma V noise for the speed sensor
bpm_sensor_rand_seed                = 1337;                                 %[-] Rand seed for the noise parameters
bpm_sensor_resolution               = single(.1);                           %[-] Resolution per bit for the speed sensor
bpm_sensor_max                      = single(10000);                        %[rpm] Maximum output of the speed sensor
bpm_sensor_min                      = single(0);                            %[rpm] Minimum output of the speed sensor
bpm_sensor_delay                    = 1;                                    %[sec] Delay time of the speed sensor

