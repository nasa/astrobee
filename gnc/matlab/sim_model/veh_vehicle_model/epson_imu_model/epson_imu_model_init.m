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

% Epson IMU Model init 
% M-G362PDC1
% Parameters derived from Datasheet

%% General
%epson_Q_B2sensor_nom            = [0 0 0 1];                            %[quat] nominal orientation of the sensor
% Replaced with tun_abp_quat_body2imu

epson_imu_misalignment          = [0.01;0.01;0.01];                     %[rad]
% epson_P_sensor_B_B_nom        Set in tunable file
epson_P_sensor_B_B_error        = [0.01,0.01,0.01];                     %[m]
epson_output_delay              = 0;                                    %[steps] number of descrite steps to delay output

% Moved to gnc.config
%tun_epson_report_truth          = 0;                                    %[flag] report truth gyro/accel measurements with no noise/bias/misalignment

epson_no_rot_effects            = 1;                                    %[flag] remove the rotation effects from the accel reading

%% Accel Properties
epson_accel_unit                = (1/9.80665)*10^3;                     %[mg/(m/s^2)] output is in milli-g's, model is m/s^2

epson_accel_upper_sat           = 3000/epson_accel_unit;                %[m/s^2]
epson_accel_lower_sat           = -3000/epson_accel_unit;               %[m/s^2]
epson_accel_resolution          = 0.125/epson_accel_unit;               %[(m/s^2)/LSB]
% epson_accel_temp_res_coeff      = 20;                        		    %[ppm/degC]  Note: currently unused in model
epson_accel_nonlinearity_coeff  = 0.001;                                %[0.1% of FS]

epson_accel_orth_misalignment   = [0.03;0.03;0.03]*pi/180;              %[rad]
epson_accel_bias_ic             = 0*[8,8,8]/epson_accel_unit;           %[m/s^2]
epson_accel_temp_bias_coeff     = [0.02,0.02,0.02]/epson_accel_unit;    %[(m/s^2)/degC]

epson_accel_bias_stab           = 0.1/epson_accel_unit;                 %[m/s^2];
epson_accel_velocity_rw         = 0.04*sqrt(1/3600);                    %[(m/s)/sqrt(sec)] Note: datasheet discrepency in units (only value in m/s??)
epson_accel_noise_density       = 0.1/epson_accel_unit;                 %[(m/s^2)/sqrt(hz)]
epson_accel_noise_seed          = [4 5 6];                              %[-]

epson_accel_3db_bandwidth       = 180;                                  %[Hz]
epson_accel_sf_coef             = 1;                                    %[-] None specified.

% Filter is unused since the 3db bandwidth (180Hz) is higher than our operating frequency (100Hz)
epson_accel_filt_num            = [1];
epson_accel_filt_den            = [1];

% values taken from the imu allan variance analysis 
epson_accel_sigma_u             = sqrt(3/540)*(0.04/epson_accel_unit);  %[(m/s^2)/sec^(3/2)] points taken from datasheet allan variance: 540s, 0.04mG
epson_accel_sigma_v             = epson_accel_velocity_rw;              %[(m/s)/sqrt(sec)]
epson_accel_sigma_w             = 1.5*epson_accel_bias_stab;            %[(m/s^2)/sec]
epson_accel_markov_tau          = .08;                                  %[sec]

%% Gyro Properties
epson_gyro_unit                 = 180/pi;                               %[rad] output is in radians

epson_gyro_upper_sat            = 150/epson_gyro_unit;                  %[rad/s]
epson_gyro_lower_sat            = -150/epson_gyro_unit;                 %[rad/s]
epson_gyro_resolution           = 0.005/epson_gyro_unit;                %[rad/s/LSB]        
% epson_gyro_temp_res_coeff       = 10;                                 %[ppm/degC]  Note: currently unused in model
epson_gyro_nonlinearity_coeff   = 0.001;                                %[0.1% of FS]

epson_gyro_orth_misalignment    = [0.1;0.1;0.1]/epson_gyro_unit;        %[rad]
epson_gyro_bias_ic              = [0.1,0.1,0.1]/epson_gyro_unit;        %[rad/s]
epson_gyro_temp_bias_coeff      = [0.001,0.001,0.001]/epson_gyro_unit;  %[rad/s/degC]

epson_gyro_linear_accel_bias_coeff = (0.01/epson_gyro_unit)*(epson_accel_unit*10^-3);    %[(rad/s)/(m/s^2)] Note: this value is a maximum.

epson_gyro_bias_stab            = (3/epson_gyro_unit)*(1/3600);         %[rad/sec]
epson_gyro_angular_rw           = (0.1/epson_gyro_unit)*sqrt(1/3600);   %[rad/sqrt(sec)] 0.1 deg/sqrt(hr)
epson_gyro_noise_density        = 0.002/epson_gyro_unit;                %[rad/s/sqrt(hz)] 1 sigma
epson_gyro_noise_seed           = [1 2 3];                              %[-] 

epson_gyro_3db_bandwidth        = 180;                                  %[Hz]
epson_gyro_sf_coef              = 1;                                    %[-] none specified.

% Filter is unused since the 3db bandwidth (180Hz) is higher than our operating frequency (100Hz)
epson_gyro_filt_num             = [1];
epson_gyro_filt_den             = [1];

%values taken from the imu allan variance analysis
epson_gyro_sigma_u              = 1.25*sqrt(3/500)*(3*(1/3600))/epson_gyro_unit; %[rad/sec^(3/2)] points taken from datasheet allan variance: 500s, 3deg/hour
epson_gyro_sigma_v              = epson_gyro_angular_rw;                %[rad/sqrt(sec)]
epson_gyro_sigma_w              = 2.2*epson_gyro_bias_stab;             %[rad/sec]
epson_gyro_markov_tau           = .04;                                  %[sec]
